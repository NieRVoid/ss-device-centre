/**
 * @file remote_control.c
 * @brief MQTT-based remote control for room management system
 * 
 * Implementation of the remote control component using the mqtt_manager component
 * 
 * Updated to use the mqtt_manager component for MQTT operations.
 * Optimized for memory efficiency.
 * 
 * @date 2025-03-20
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "remote_control.h"
#include "mqtt_manager.h"

static const char *TAG = "remote-control";

// Replace macro with inline function for better maintainability
static inline esp_err_t require_component_init(const char *name, bool condition) {
    if (!condition) {
        ESP_LOGE(TAG, "%s component must be initialized first", name);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

// MQTT topic paths (suffixes added to prefix)
#define TOPIC_STATUS "status"
#define TOPIC_COMMAND "command"

// MQTT command strings
#define CMD_STR_REQUEST_STATUS "request_status"
#define CMD_STR_SET_OCCUPIED "set_occupied"
#define CMD_STR_SET_VACANT "set_vacant"
#define CMD_STR_RESTART_DEVICE "restart"
#define CMD_STR_CONFIG_UPDATE "config_update"

// Memory tracking variables
static size_t s_total_topics_memory = 0;
static size_t s_total_user_props_memory = 0;

// Topic buffer optimization
#define MAX_TOPIC_LEN 64
typedef struct {
    char prefix[MAX_TOPIC_LEN];
    char status_topic[MAX_TOPIC_LEN];
    char command_topic[MAX_TOPIC_LEN];
} topic_buffers_t;

typedef struct {
    SemaphoreHandle_t mutex;
    esp_timer_handle_t report_timer;
    
    // Use pre-allocated buffer instead of dynamic allocation
    topic_buffers_t topics;
    bool using_static_topics;
    
    // Only use these pointers for dynamically allocated topics that exceed buffer size
    char *topic_prefix;
    char *status_topic;
    char *command_topic;
    
    bool running;
    int command_handler_id;
    int mqtt_connect_callback_id;
    
    // Callback functions and context
    remote_command_callback_t command_callback;
    void *command_callback_ctx;
    status_request_callback_t status_callback;
    void *status_callback_ctx;
    
    // Configuration
    uint32_t status_interval_sec;

    // User properties for MQTT messages
    mqtt_user_property_t *user_properties;
    int user_property_count;
    bool owns_user_properties;
} remote_control_t;

static remote_control_t remote_control = {0};

// Function prototypes
static void report_timer_callback(void *arg);
static esp_err_t publish_status_internal(const room_status_t *status);
static void mqtt_message_handler(const char *topic, int topic_len, 
                               const char *data, int data_len, 
                               void *user_data);
static void mqtt_connect_handler(void *user_data);
static void process_mqtt_command(const char *command_str, const char *payload, size_t payload_len);
static remote_command_t parse_mqtt_command(const char *command);
static esp_err_t prepare_topics(const char *prefix);

/**
 * @brief Initialize the remote control component
 */
esp_err_t remote_control_init(const remote_control_config_t *config)
{
    esp_err_t ret = ESP_OK;
    
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing remote control component");
    
    // Verify MQTT manager is initialized first
    ret = require_component_init("MQTT Manager", mqtt_manager_is_initialized());
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Reset memory tracking
    s_total_topics_memory = 0;
    s_total_user_props_memory = 0;
    
    // Create mutex for operations
    remote_control.mutex = xSemaphoreCreateMutex();
    if (remote_control.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Store callbacks
    remote_control.command_callback = config->command_callback;
    remote_control.command_callback_ctx = config->command_callback_ctx;
    remote_control.status_callback = config->status_callback;
    remote_control.status_callback_ctx = config->status_callback_ctx;
    remote_control.status_interval_sec = config->status_interval_sec;
    
    // Initialize topic pointers to NULL
    remote_control.topic_prefix = NULL;
    remote_control.status_topic = NULL;
    remote_control.command_topic = NULL;
    remote_control.using_static_topics = false;
    
    // Prepare topics (using static buffers when possible)
    ret = prepare_topics(config->topic_prefix);
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    // Store user properties
    if (config->user_properties && config->user_property_count > 0) {
        size_t props_size = config->user_property_count * sizeof(mqtt_user_property_t);
        remote_control.user_properties = malloc(props_size);
        if (!remote_control.user_properties) {
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
        memcpy(remote_control.user_properties, config->user_properties, props_size);
        remote_control.user_property_count = config->user_property_count;
        remote_control.owns_user_properties = true;
        s_total_user_props_memory = props_size;
    } else {
        remote_control.user_properties = NULL;
        remote_control.user_property_count = 0;
        remote_control.owns_user_properties = false;
    }

    // Log topic information
    ESP_LOGI(TAG, "Status topic: %s", remote_control.using_static_topics ? 
                                     remote_control.topics.status_topic : 
                                     remote_control.status_topic);
    ESP_LOGI(TAG, "Command topic: %s", remote_control.using_static_topics ? 
                                      remote_control.topics.command_topic : 
                                      remote_control.command_topic);
    
    // Ensure the MQTT manager is connected
    if (!mqtt_manager_is_connected()) {
        ESP_LOGW(TAG, "MQTT Manager not connected - ensure it's initialized before using remote control");
    }
    
    // Create periodic reporting timer
    const esp_timer_create_args_t timer_args = {
        .callback = &report_timer_callback,
        .name = "mqtt_report"
    };
    
    ret = esp_timer_create(&timer_args, &remote_control.report_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create report timer: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    return ESP_OK;
    
cleanup:
    if (!remote_control.using_static_topics) {
        if (remote_control.topic_prefix) {
            free(remote_control.topic_prefix);
            s_total_topics_memory -= strlen(remote_control.topic_prefix) + 1;
        }
        if (remote_control.status_topic) {
            free(remote_control.status_topic);
            s_total_topics_memory -= strlen(remote_control.status_topic) + 1;
        }
        if (remote_control.command_topic) {
            free(remote_control.command_topic);
            s_total_topics_memory -= strlen(remote_control.command_topic) + 1;
        }
    }
    if (remote_control.user_properties && remote_control.owns_user_properties) {
        free(remote_control.user_properties);
        s_total_user_props_memory = 0;
    }
    if (remote_control.mutex) vSemaphoreDelete(remote_control.mutex);
    if (remote_control.report_timer) esp_timer_delete(remote_control.report_timer);
    
    return ret;
}

/**
 * @brief Prepare topic strings, using static buffers when possible
 */
static esp_err_t prepare_topics(const char *prefix)
{
    size_t prefix_len = strlen(prefix);
    
    // Try to use static buffers first
    if (prefix_len < MAX_TOPIC_LEN - 10) { // Leave room for suffixes
        // Use static buffers
        remote_control.using_static_topics = true;
        
        // Copy prefix
        strcpy(remote_control.topics.prefix, prefix);
        
        // Create full topic paths
        snprintf(remote_control.topics.status_topic, MAX_TOPIC_LEN, "%s%s", 
                 prefix, TOPIC_STATUS);
        
        snprintf(remote_control.topics.command_topic, MAX_TOPIC_LEN, "%s%s", 
                 prefix, TOPIC_COMMAND);
                 
        return ESP_OK;
    } 
    
    // Fall back to dynamic allocation for long topics
    remote_control.using_static_topics = false;
    
    // Allocate topic strings
    remote_control.topic_prefix = strdup(prefix);
    if (!remote_control.topic_prefix) {
        return ESP_ERR_NO_MEM;
    }
    s_total_topics_memory += prefix_len + 1;
    
    // Create full topic paths
    size_t status_topic_len = prefix_len + strlen(TOPIC_STATUS) + 1; // +1 for '/'
    remote_control.status_topic = malloc(status_topic_len + 1);      // +1 for null terminator
    if (!remote_control.status_topic) {
        free(remote_control.topic_prefix);
        s_total_topics_memory -= prefix_len + 1;
        return ESP_ERR_NO_MEM;
    }
    s_total_topics_memory += status_topic_len + 1;
    
    snprintf(remote_control.status_topic, status_topic_len + 1, "%s%s", 
             prefix, TOPIC_STATUS);
    
    size_t command_topic_len = prefix_len + strlen(TOPIC_COMMAND) + 1;
    remote_control.command_topic = malloc(command_topic_len + 1);
    if (!remote_control.command_topic) {
        free(remote_control.topic_prefix);
        free(remote_control.status_topic);
        s_total_topics_memory -= (prefix_len + 1 + status_topic_len + 1);
        return ESP_ERR_NO_MEM;
    }
    s_total_topics_memory += command_topic_len + 1;
    
    snprintf(remote_control.command_topic, command_topic_len + 1, "%s%s", 
             prefix, TOPIC_COMMAND);
    
    return ESP_OK;
}

/**
 * @brief Start the remote control background task
 */
esp_err_t remote_control_start(void)
{
    ESP_LOGI(TAG, "Starting remote control component");
    
    // Get the appropriate command topic based on allocation type
    const char *command_topic = remote_control.using_static_topics ? 
                              remote_control.topics.command_topic : 
                              remote_control.command_topic;
    
    // Register for the command topic with MQTT manager
    int handler_id;
    esp_err_t ret = mqtt_manager_register_handler(
        command_topic,
        1, // QoS 1
        mqtt_message_handler,
        NULL,
        remote_control.user_properties,
        remote_control.user_property_count,
        &handler_id
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    remote_control.command_handler_id = handler_id;
    remote_control.running = true;
    
    // Start periodic reporting timer
    esp_timer_start_periodic(remote_control.report_timer, 
                            remote_control.status_interval_sec * 1000000); // convert to microseconds
    
    // If MQTT is already connected, publish an initial status
    if (mqtt_manager_is_connected() && remote_control.status_callback) {
        room_status_t status = remote_control.status_callback(
            remote_control.status_callback_ctx);
        publish_status_internal(&status);
    }
    
    return ESP_OK;
}

/**
 * @brief Stop the remote control component
 */
esp_err_t remote_control_stop(void)
{
    ESP_LOGI(TAG, "Stopping remote control component");
    
    remote_control.running = false;
    
    // Stop timer
    esp_timer_stop(remote_control.report_timer);
    
    // Unregister from MQTT topics
    esp_err_t ret = mqtt_manager_unregister_handler(remote_control.command_handler_id);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to unregister MQTT handler: %s", esp_err_to_name(ret));
    }
    
    return ESP_OK;
}

/**
 * @brief MQTT message handler callback
 */
static void mqtt_message_handler(const char *topic, int topic_len, 
                               const char *data, int data_len, 
                               void *user_data)
{
    ESP_LOGI(TAG, "Received message on topic: %.*s", topic_len, topic);
    ESP_LOGI(TAG, "Data: %.*s", data_len, data);
    
    // Use stack buffer for small commands, heap for larger ones
    char cmd_buf[64]; // Stack buffer for typical commands
    char *command;
    bool heap_allocated = false;
    
    if (data_len < sizeof(cmd_buf) - 1) {
        // Use stack buffer
        command = cmd_buf;
    } else {
        // Use heap for larger payloads
        command = malloc(data_len + 1);
        if (!command) {
            ESP_LOGE(TAG, "Failed to allocate memory for command");
            return;
        }
        heap_allocated = true;
    }
    
    memcpy(command, data, data_len);
    command[data_len] = '\0';
    
    // Process the command
    process_mqtt_command(command, data, data_len);
    
    // Free heap-allocated buffer if used
    if (heap_allocated) {
        free(command);
    }
}

/**
 * @brief MQTT connection event handler
 */
static void mqtt_connect_handler(void *user_data)
{
    ESP_LOGI(TAG, "MQTT connected - publishing initial status");
    
    // Send initial status if callback is available
    if (remote_control.status_callback) {
        room_status_t status = remote_control.status_callback(
            remote_control.status_callback_ctx);
        publish_status_internal(&status);
    }
}

/**
 * @brief Parse command string into remote_command_t enum
 */
static remote_command_t parse_mqtt_command(const char *command)
{
    // Optimized version using lookup table approach
    struct {
        const char *cmd_str;
        remote_command_t cmd_enum;
    } cmd_map[] = {
        {CMD_STR_REQUEST_STATUS, REMOTE_CMD_REQUEST_STATUS},
        {CMD_STR_SET_OCCUPIED, REMOTE_CMD_SET_OCCUPIED},
        {CMD_STR_SET_VACANT, REMOTE_CMD_SET_VACANT},
        {CMD_STR_RESTART_DEVICE, REMOTE_CMD_RESTART_DEVICE},
        {CMD_STR_CONFIG_UPDATE, REMOTE_CMD_CONFIG_UPDATE},
        {NULL, REMOTE_CMD_UNKNOWN}
    };
    
    for (int i = 0; cmd_map[i].cmd_str != NULL; i++) {
        if (strcmp(command, cmd_map[i].cmd_str) == 0) {
            return cmd_map[i].cmd_enum;
        }
    }
    
    return REMOTE_CMD_UNKNOWN;
}

/**
 * @brief Process MQTT command messages
 */
static void process_mqtt_command(const char *command_str, const char *payload, size_t payload_len)
{
    ESP_LOGI(TAG, "Processing command: %s", command_str);
    
    remote_command_t cmd = parse_mqtt_command(command_str);
    
    // Handle request_status internally
    if (cmd == REMOTE_CMD_REQUEST_STATUS) {
        ESP_LOGI(TAG, "Command: Request status");
        if (remote_control.status_callback) {
            room_status_t status = remote_control.status_callback(
                remote_control.status_callback_ctx);
            publish_status_internal(&status);
        }
    }
    
    // Forward all commands to callback (including request_status)
    if (remote_control.command_callback) {
        remote_control.command_callback(cmd, payload, remote_control.command_callback_ctx);
    }
}

/**
 * @brief Periodic timer callback for status reporting
 */
static void report_timer_callback(void *arg)
{
    ESP_LOGI(TAG, "Periodic status report timer triggered");
    
    if (remote_control.status_callback) {
        room_status_t status = remote_control.status_callback(
            remote_control.status_callback_ctx);
        publish_status_internal(&status);
    }
}

/**
 * @brief Internal function to publish room status
 */
static esp_err_t publish_status_internal(const room_status_t *status)
{
    if (!mqtt_manager_is_connected() || status == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Take mutex to ensure thread safety
    if (xSemaphoreTake(remote_control.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex for publishing");
        return ESP_ERR_TIMEOUT;
    }
    
    // Convert room state to string (using constant strings to avoid allocations)
    const char *state_str;
    switch (status->state) {
        case ROOM_STATE_VACANT:
            state_str = "vacant";
            break;
        case ROOM_STATE_OCCUPIED:
            state_str = "occupied";
            break;
        default:
            state_str = "unknown";
            break;
    }
    
    // Get source name or default
    const char *source = (status->source_name != NULL) ? status->source_name : "unknown";
    
    // Format status as JSON - use a reasonably sized buffer on stack
    char json_buffer[256];
    snprintf(json_buffer, sizeof(json_buffer), 
             "{"
             "\"state\":\"%s\","
             "\"count\":%d,"
             "\"count_reliable\":%s,"
             "\"source\":\"%s\","
             "\"reliability\":%d,"
             "\"timestamp\":%lld"
             "}",
             state_str,
             status->occupant_count,
             status->count_reliable ? "true" : "false",
             source,
             status->source_reliability,
             esp_timer_get_time() / 1000 // microseconds to milliseconds
    );
    
    // Get appropriate status topic based on allocation type
    const char *status_topic = remote_control.using_static_topics ? 
                             remote_control.topics.status_topic : 
                             remote_control.status_topic;
    
    // Publish the status using MQTT manager
    ESP_LOGI(TAG, "Publishing status: %s", json_buffer);
    int msg_id = mqtt_manager_publish(
        status_topic,
        json_buffer, 
        -1,  // use null-terminator
        1,   // QoS 1
        1,   // retain
        remote_control.user_properties,  // user properties
        remote_control.user_property_count
    );
    
    xSemaphoreGive(remote_control.mutex);
    
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish status");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Status published successfully, msg_id=%d", msg_id);
    return ESP_OK;
}

/**
 * @brief Publish room status update
 */
esp_err_t remote_control_publish_status(const room_status_t *status)
{
    return publish_status_internal(status);
}

/**
 * @brief Request immediate status update
 */
esp_err_t remote_control_request_update(void)
{
    if (!remote_control.status_callback) {
        return ESP_ERR_INVALID_STATE;
    }
    
    room_status_t status = remote_control.status_callback(remote_control.status_callback_ctx);
    return publish_status_internal(&status);
}

/**
 * @brief Get memory usage of the remote control component
 */
size_t remote_control_get_memory_usage(void)
{
    return s_total_topics_memory + s_total_user_props_memory;
}

/**
 * @brief Deinitialize the remote control component
 */
esp_err_t remote_control_deinit(void)
{
    // Stop the component first
    esp_err_t ret = remote_control_stop();
    
    // Clean up resources
    if (!remote_control.using_static_topics) {
        if (remote_control.topic_prefix) {
            free(remote_control.topic_prefix);
            remote_control.topic_prefix = NULL;
        }
        if (remote_control.status_topic) {
            free(remote_control.status_topic);
            remote_control.status_topic = NULL;
        }
        if (remote_control.command_topic) {
            free(remote_control.command_topic);
            remote_control.command_topic = NULL;
        }
    }
    
    if (remote_control.user_properties && remote_control.owns_user_properties) {
        free(remote_control.user_properties);
        s_total_user_props_memory = 0;
    }
    remote_control.user_properties = NULL;
    
    if (remote_control.mutex) {
        vSemaphoreDelete(remote_control.mutex);
        remote_control.mutex = NULL;
    }
    
    if (remote_control.report_timer) {
        esp_timer_delete(remote_control.report_timer);
        remote_control.report_timer = NULL;
    }
    
    // Reset memory tracking
    s_total_topics_memory = 0;
    s_total_user_props_memory = 0;
    
    // Zero out the structure
    memset(&remote_control, 0, sizeof(remote_control_t));
    
    return ret;
}
