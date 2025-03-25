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
    void *command_callback_ctx;ommand_callback;
    status_request_callback_t status_callback;
    status_request_callback_t status_callback;
    void *status_callback_ctx;
    
    // Configuration
    uint32_t status_interval_sec;

    // User properties for MQTT messages
    mqtt_user_property_t *user_properties;
    int user_property_count;properties;
} remote_control_t;} remote_control_t;

static remote_control_t remote_control = {0};static remote_control_t remote_control = {0};

// Function prototypes
static void report_timer_callback(void *arg);
static esp_err_t publish_status_internal(const room_status_t *status);s);
static void mqtt_message_handler(const char *topic, int topic_len, en, 
                               const char *data, int data_len,  int data_len, 
                               void *user_data);
static void mqtt_connect_handler(void *user_data);
static void process_mqtt_command(const char *command_str, const char *payload, size_t payload_len);char *payload, size_t payload_len);
static remote_command_t parse_mqtt_command(const char *command);r *command);
static esp_err_t prepare_topics(const char *prefix);static esp_err_t prepare_topics(const char *prefix);

/**
 * @brief Initialize the remote control component@brief Initialize the remote control component
 */
esp_err_t remote_control_init(const remote_control_config_t *config)sp_err_t remote_control_init(const remote_control_config_t *config)
{
    esp_err_t ret = ESP_OK;esp_err_t ret = ESP_OK;
    
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;   return ESP_ERR_INVALID_ARG;
    }}
    
    ESP_LOGI(TAG, "Initializing remote control component");ESP_LOGI(TAG, "Initializing remote control component");
    
    // Reset memory trackinginitialized and return error if not
    s_total_topics_memory = 0;ed()) {
    s_total_user_props_memory = 0;    ESP_LOGE(TAG, "MQTT Manager must be initialized before remote control");
    TE;
    // Create mutex for operations
    remote_control.mutex = xSemaphoreCreateMutex();
    // Reset memory tracking
    s_total_topics_memory = 0;to create mutex");
    s_total_user_props_memory = 0;   return ESP_ERR_NO_MEM;
    }
    // Create mutex for operations
    remote_control.mutex = xSemaphoreCreateMutex();
    if (remote_control.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");lback_ctx;
        return ESP_ERR_NO_MEM;
    }
    remote_control.status_interval_sec = config->status_interval_sec;
    // Store callbacks
    remote_control.command_callback = config->command_callback;L
    remote_control.command_callback_ctx = config->command_callback_ctx;
    remote_control.status_callback = config->status_callback;
    remote_control.status_callback_ctx = config->status_callback_ctx;
    remote_control.status_interval_sec = config->status_interval_sec;remote_control.using_static_topics = false;
    
    // Initialize topic pointers to NULLn possible)
    remote_control.topic_prefix = NULL;(config->topic_prefix);
    remote_control.status_topic = NULL;) {
    remote_control.command_topic = NULL;   goto cleanup;
    remote_control.using_static_topics = false;}
    
    // Prepare topics (using static buffers when possible)y in mqtt_manager
    ret = prepare_topics(config->topic_prefix);
    if (ret != ESP_OK) {qtt_manager
        goto cleanup;onfig->user_properties, config->user_property_count)) {
    }m, don't duplicate
    l.user_properties = config->user_properties;
            remote_control.user_property_count = config->user_property_count;ore user properties
            remote_control.owns_user_properties = false;
        } else {r_property_t);
            // Create our own copy(props_size);
            size_t props_size = config->user_property_count * sizeof(mqtt_user_property_t);!remote_control.user_properties) {
            remote_control.user_properties = malloc(props_size);
            if (!remote_control.user_properties) {
                ret = ESP_ERR_NO_MEM;   }
                goto cleanup;        memcpy(remote_control.user_properties, config->user_properties, props_size);
            }property_count = config->user_property_count;
            memcpy(remote_control.user_properties, config->user_properties, props_size);
            remote_control.user_property_count = config->user_property_count;
            remote_control.owns_user_properties = true;
            s_total_user_props_memory = props_size;
        }
    } else {
        remote_control.user_properties = NULL;// Log topic information
        remote_control.user_property_count = 0;e_control.using_static_topics ? 
        remote_control.owns_user_properties = false;mote_control.topics.status_topic : 
    }
SP_LOGI(TAG, "Command topic: %s", remote_control.using_static_topics ? 
    // Log topic information                                  remote_control.topics.command_topic : 
    ESP_LOGI(TAG, "Status topic: %s", remote_control.using_static_topics ? remote_control.command_topic);
                                     remote_control.topics.status_topic : 
                                     remote_control.status_topic);
    ESP_LOGI(TAG, "Command topic: %s", remote_control.using_static_topics ? ected()) {
                                      remote_control.topics.command_topic :   ESP_LOGW(TAG, "MQTT Manager not connected - ensure it's initialized before using remote control");
                                      remote_control.command_topic);}
    
    // Ensure the MQTT manager is connectedeporting timer
    if (!mqtt_manager_is_connected()) {
        ESP_LOGW(TAG, "MQTT Manager not connected - ensure it's initialized before using remote control");report_timer_callback,
    }   .name = "mqtt_report"
    };
    // Create periodic reporting timer
    const esp_timer_create_args_t timer_args = {ret = esp_timer_create(&timer_args, &remote_control.report_timer);
        .callback = &report_timer_callback,ret != ESP_OK) {
        .name = "mqtt_report" timer: %s", esp_err_to_name(ret));
    };
    
    ret = esp_timer_create(&timer_args, &remote_control.report_timer);
    if (ret != ESP_OK) {n ESP_OK;
        ESP_LOGE(TAG, "Failed to create report timer: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    f (remote_control.topic_prefix) {
    return ESP_OK;x);
    ote_control.topic_prefix) + 1;
cleanup:
    if (!remote_control.using_static_topics) {f (remote_control.status_topic) {
        if (remote_control.topic_prefix) {       free(remote_control.status_topic);
            free(remote_control.topic_prefix);en(remote_control.status_topic) + 1;
            s_total_topics_memory -= strlen(remote_control.topic_prefix) + 1;
        }ic) {
        if (remote_control.status_topic) {       free(remote_control.command_topic);
            free(remote_control.status_topic);pic) + 1;
            s_total_topics_memory -= strlen(remote_control.status_topic) + 1;
        }}
        if (remote_control.command_topic) {control.user_properties) {
            free(remote_control.command_topic);       free(remote_control.user_properties);
            s_total_topics_memory -= strlen(remote_control.command_topic) + 1;        s_total_user_props_memory = 0;
        } }
    });
    if (remote_control.user_properties && remote_control.owns_user_properties) { if (remote_control.report_timer) esp_timer_delete(remote_control.report_timer);
        free(remote_control.user_properties);
        s_total_user_props_memory = 0;   return ret;
    }
    if (remote_control.mutex) vSemaphoreDelete(remote_control.mutex);
    if (remote_control.report_timer) esp_timer_delete(remote_control.report_timer);
    
    return ret;
})

/**= strlen(prefix);
 * @brief Prepare topic strings, using static buffers when possible
 */ry to use static buffers first
static esp_err_t prepare_topics(const char *prefix) - 10) { // Leave room for suffixes
{
    size_t prefix_len = strlen(prefix);ics = true;
    
    // Try to use static buffers first
    if (prefix_len < MAX_TOPIC_LEN - 10) { // Leave room for suffixesfix, prefix);
        // Use static buffers
        remote_control.using_static_topics = true; topic paths
          snprintf(remote_control.topics.status_topic, MAX_TOPIC_LEN, "%s%s", 
        // Copy prefix             prefix, TOPIC_STATUS);
        strcpy(remote_control.topics.prefix, prefix);
        topic, MAX_TOPIC_LEN, "%s%s", 
        // Create full topic paths             prefix, TOPIC_COMMAND);
        snprintf(remote_control.topics.status_topic, MAX_TOPIC_LEN, "%s%s", 
                 prefix, TOPIC_STATUS);
        
        snprintf(remote_control.topics.command_topic, MAX_TOPIC_LEN, "%s%s", 
                 prefix, TOPIC_COMMAND);/ Fall back to dynamic allocation for long topics
                 se;
        return ESP_OK;
    } 
    
    // Fall back to dynamic allocation for long topics
    remote_control.using_static_topics = false;
    
    // Allocate topic strings
    remote_control.topic_prefix = strdup(prefix);
    if (!remote_control.topic_prefix) {/ Create full topic paths
        return ESP_ERR_NO_MEM;TOPIC_STATUS) + 1; // +1 for '/'
    }remote_control.status_topic = malloc(status_topic_len + 1);      // +1 for null terminator
    s_total_topics_memory += prefix_len + 1;
    refix);
    // Create full topic paths    s_total_topics_memory -= prefix_len + 1;
    size_t status_topic_len = prefix_len + strlen(TOPIC_STATUS) + 1; // +1 for '/'
    remote_control.status_topic = malloc(status_topic_len + 1);      // +1 for null terminator
    if (!remote_control.status_topic) {c_len + 1;
        free(remote_control.topic_prefix);
        s_total_topics_memory -= prefix_len + 1;status_topic_len + 1, "%s%s", 
        return ESP_ERR_NO_MEM;
    }
    s_total_topics_memory += status_topic_len + 1;ize_t command_topic_len = prefix_len + strlen(TOPIC_COMMAND) + 1;
    opic_len + 1);
    snprintf(remote_control.status_topic, status_topic_len + 1, "%s%s", if (!remote_control.command_topic) {
             prefix, TOPIC_STATUS);
    opic);
    size_t command_topic_len = prefix_len + strlen(TOPIC_COMMAND) + 1;    s_total_topics_memory -= (prefix_len + 1 + status_topic_len + 1);
    remote_control.command_topic = malloc(command_topic_len + 1);_ERR_NO_MEM;
    if (!remote_control.command_topic) {   }
        free(remote_control.topic_prefix);    s_total_topics_memory += command_topic_len + 1;
        free(remote_control.status_topic); 
        s_total_topics_memory -= (prefix_len + 1 + status_topic_len + 1);_topic_len + 1, "%s%s", 
        return ESP_ERR_NO_MEM;          prefix, TOPIC_COMMAND);
    }
    s_total_topics_memory += command_topic_len + 1;   return ESP_OK;
    
    snprintf(remote_control.command_topic, command_topic_len + 1, "%s%s", 
             prefix, TOPIC_COMMAND);
    
    return ESP_OK;
}

/**
 * @brief Start the remote control background task
 */allocation type
esp_err_t remote_control_start(void)d_topic = remote_control.using_static_topics ? 
{           remote_control.topics.command_topic : 
    ESP_LOGI(TAG, "Starting remote control component"); remote_control.command_topic;
    
    // Get the appropriate command topic based on allocation typeith MQTT manager
    const char *command_topic = remote_control.using_static_topics ? 
                              remote_control.topics.command_topic :  mqtt_manager_register_handler(
                              remote_control.command_topic;  command_topic,
        1, // QoS 1
    // Register for the command topic with MQTT managerdler,
    int handler_id;
    esp_err_t ret = mqtt_manager_register_handler(rol.user_properties,
        command_topic,   remote_control.user_property_count,
        1, // QoS 1    &handler_id
        mqtt_message_handler,
        NULL,
        remote_control.user_properties,if (ret != ESP_OK) {
        remote_control.user_property_count,ster MQTT handler: %s", esp_err_to_name(ret));
        &handler_id
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT handler: %s", esp_err_to_name(ret));
        return ret;
    }
    ol.report_timer, 
    remote_control.command_handler_id = handler_id;                       remote_control.status_interval_sec * 1000000); // convert to microseconds
    remote_control.running = true;
    r MQTT connection events
    // Start periodic reporting timer   mqtt_connect_callback_id = mqtt_manager_register_connect_handler(mqtt_connect_handler, NULL);
    esp_timer_start_periodic(remote_control.report_timer,     
                            remote_control.status_interval_sec * 1000000); // convert to microseconds // If MQTT is already connected, publish an initial status
    te_control.status_callback) {
    // If MQTT is already connected, publish an initial status     room_status_t status = remote_control.status_callback(
    if (mqtt_manager_is_connected() && remote_control.status_callback) {allback_ctx);
        room_status_t status = remote_control.status_callback(       publish_status_internal(&status);
            remote_control.status_callback_ctx);
        publish_status_internal(&status);
    }
    
    return ESP_OK;
}
brief Stop the remote control component
/**
 * @brief Stop the remote control component
 */
esp_err_t remote_control_stop(void)
{
    ESP_LOGI(TAG, "Stopping remote control component");remote_control.running = false;
    
    remote_control.running = false;   // Stop timer
        esp_timer_stop(remote_control.report_timer);
    // Stop timer 
    esp_timer_stop(remote_control.report_timer);
     esp_err_t ret = mqtt_manager_unregister_handler(remote_control.command_handler_id);
    // Unregister from MQTT topics
    esp_err_t ret = mqtt_manager_unregister_handler(remote_control.command_handler_id);esp_err_to_name(ret));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to unregister MQTT handler: %s", esp_err_to_name(ret));   
    }
    _connect_callback_id);
    return ESP_OK;
}

/**
 * @brief MQTT message handler callback
 */brief MQTT message handler callback
static void mqtt_message_handler(const char *topic, int topic_len, 
                               const char *data, int data_len, ndler(const char *topic, int topic_len, 
                               void *user_data)     const char *data, int data_len, 
{                   void *user_data)
    ESP_LOGI(TAG, "Received message on topic: %.*s", topic_len, topic);
    ESP_LOGI(TAG, "Data: %.*s", data_len, data);topic: %.*s", topic_len, topic);
    : %.*s", data_len, data);
    // Use stack buffer for small commands, heap for larger ones
    char cmd_buf[64]; // Stack buffer for typical commandsffer for small commands, heap for larger ones
    char *command;cmd_buf[64]; // Stack buffer for typical commands
    bool heap_allocated = false;
    ool heap_allocated = false;
    if (data_len < sizeof(cmd_buf) - 1) {
        // Use stack buffer 1) {
        command = cmd_buf;
    } else {    command = cmd_buf;
        // Use heap for larger payloads
        command = malloc(data_len + 1);
        if (!command) {    command = malloc(data_len + 1);
            ESP_LOGE(TAG, "Failed to allocate memory for command");
            return; "Failed to allocate memory for command");
        }
        heap_allocated = true;   }
    }       heap_allocated = true;
        }
    memcpy(command, data, data_len); 
    command[data_len] = '\0';
     command[data_len] = '\0';
    // Process the command
    process_mqtt_command(command, data, data_len);   // Process the command
    
    // Free heap-allocated buffer if used
    if (heap_allocated) {
        free(command);
    }
}

/**
 * @brief MQTT connection event handler**
 */ * @brief MQTT connection event handler
static void mqtt_connect_handler(void *user_data)
{
    ESP_LOGI(TAG, "MQTT connected - publishing initial status");
    ;
    // Send initial status if callback is available   
    if (remote_control.status_callback) {
        room_status_t status = remote_control.status_callback(te_control.status_callback) {
            remote_control.status_callback_ctx); = remote_control.status_callback(
        publish_status_internal(&status);callback_ctx);
    }tus_internal(&status);
}

/**
 * @brief Parse command string into remote_command_t enum
 */
static remote_command_t parse_mqtt_command(const char *command)
{ remote_command_t parse_mqtt_command(const char *command)
    // Optimized version using lookup table approach
    struct {
        const char *cmd_str;
        remote_command_t cmd_enum;
    } cmd_map[] = {emote_command_t cmd_enum;
        {CMD_STR_REQUEST_STATUS, REMOTE_CMD_REQUEST_STATUS}, cmd_map[] = {
        {CMD_STR_SET_OCCUPIED, REMOTE_CMD_SET_OCCUPIED},    {CMD_STR_REQUEST_STATUS, REMOTE_CMD_REQUEST_STATUS},
        {CMD_STR_SET_VACANT, REMOTE_CMD_SET_VACANT}, REMOTE_CMD_SET_OCCUPIED},
        {CMD_STR_RESTART_DEVICE, REMOTE_CMD_RESTART_DEVICE},       {CMD_STR_SET_VACANT, REMOTE_CMD_SET_VACANT},
        {CMD_STR_CONFIG_UPDATE, REMOTE_CMD_CONFIG_UPDATE},        {CMD_STR_RESTART_DEVICE, REMOTE_CMD_RESTART_DEVICE},
        {NULL, REMOTE_CMD_UNKNOWN}     {CMD_STR_CONFIG_UPDATE, REMOTE_CMD_CONFIG_UPDATE},
    };
     };
    for (int i = 0; cmd_map[i].cmd_str != NULL; i++) {
        if (strcmp(command, cmd_map[i].cmd_str) == 0) {   for (int i = 0; cmd_map[i].cmd_str != NULL; i++) {
            return cmd_map[i].cmd_enum;
        }        return cmd_map[i].cmd_enum;
    }
    }
    return REMOTE_CMD_UNKNOWN;
}

/**
 * @brief Process MQTT command messages
 */
static void process_mqtt_command(const char *command_str, const char *payload, size_t payload_len)
{id process_mqtt_command(const char *command_str, const char *payload, size_t payload_len)
    ESP_LOGI(TAG, "Processing command: %s", command_str);
    ESP_LOGI(TAG, "Processing command: %s", command_str);
    remote_command_t cmd = parse_mqtt_command(command_str);
    and(command_str);
    // Handle request_status internally
    if (cmd == REMOTE_CMD_REQUEST_STATUS) {/ Handle request_status internally
        ESP_LOGI(TAG, "Command: Request status");   if (cmd == REMOTE_CMD_REQUEST_STATUS) {
        if (remote_control.status_callback) {        ESP_LOGI(TAG, "Command: Request status");
            room_status_t status = remote_control.status_callback(     if (remote_control.status_callback) {
                remote_control.status_callback_ctx);us_callback(
            publish_status_internal(&status);             remote_control.status_callback_ctx);
        };
    }       }
    
    // Forward all commands to callback (including request_status)
    if (remote_control.command_callback) {including request_status)
        remote_control.command_callback(cmd, payload, remote_control.command_callback_ctx);
    }load, remote_control.command_callback_ctx);
}

/**
 * @brief Periodic timer callback for status reporting/**
 */@brief Periodic timer callback for status reporting
static void report_timer_callback(void *arg)
{tic void report_timer_callback(void *arg)
    ESP_LOGI(TAG, "Periodic status report timer triggered");
       ESP_LOGI(TAG, "Periodic status report timer triggered");
    if (remote_control.status_callback) {
        room_status_t status = remote_control.status_callback(k) {
            remote_control.status_callback_ctx);   room_status_t status = remote_control.status_callback(
        publish_status_internal(&status);        remote_control.status_callback_ctx);
    }
}

/**
 * @brief Internal function to publish room status
 */brief Internal function to publish room status
static esp_err_t publish_status_internal(const room_status_t *status)
{tatus_internal(const room_status_t *status)
    if (!mqtt_manager_is_connected() || status == NULL) {
        return ESP_ERR_INVALID_STATE;ted() || status == NULL) {
    }ATE;
    
    // Take mutex to ensure thread safety
    if (xSemaphoreTake(remote_control.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {safety
        ESP_LOGW(TAG, "Failed to take mutex for publishing");Take(remote_control.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;(TAG, "Failed to take mutex for publishing");
    }
    
    // Convert room state to string (using constant strings to avoid allocations)
    const char *state_str;// Convert room state to string (using constant strings to avoid allocations)
    switch (status->state) {
        case ROOM_STATE_VACANT:
            state_str = "vacant";    case ROOM_STATE_VACANT:
            break;
        case ROOM_STATE_OCCUPIED:
            state_str = "occupied";
            break;e_str = "occupied";
        default:
            state_str = "unknown";
            break;
    }
    
    // Get source name or default
    const char *source = (status->source_name != NULL) ? status->source_name : "unknown"; name or default
    = (status->source_name != NULL) ? status->source_name : "unknown";
    // Format status as JSON - use a reasonably sized buffer on stack
    char json_buffer[256];uffer on stack
    snprintf(json_buffer, sizeof(json_buffer), [256];
             "{"ffer), 
             "\"state\":\"%s\","
             "\"count\":%d,"       "\"state\":\"%s\","
             "\"count_reliable\":%s,"         "\"count\":%d,"
             "\"source\":\"%s\","
             "\"reliability\":%d,"
             "\"timestamp\":%lld"
             "}",
             state_str,         "}",
             status->occupant_count,
             status->count_reliable ? "true" : "false",
             source,"true" : "false",
             status->source_reliability,
             esp_timer_get_time() / 1000 // microseconds to millisecondssource_reliability,
    ); 1000 // microseconds to milliseconds
    
    // Get appropriate status topic based on allocation type
    const char *status_topic = remote_control.using_static_topics ? e
                             remote_control.topics.status_topic : rol.using_static_topics ? 
                             remote_control.status_topic;                       remote_control.topics.status_topic : 
                             remote_control.status_topic;
    // Publish the status using MQTT manager
    ESP_LOGI(TAG, "Publishing status: %s", json_buffer);// Publish the status using MQTT manager
    int msg_id = mqtt_manager_publish(blishing status: %s", json_buffer);
        status_topic,
        json_buffer, 
        -1,  // use null-terminator   json_buffer, 
        1,   // QoS 1    -1,  // use null-terminator
        1,   // retain
        remote_control.user_properties,  // user propertiestain
        remote_control.user_property_count       remote_control.user_properties,  // user properties
    );        remote_control.user_property_count
     );
    xSemaphoreGive(remote_control.mutex);
     xSemaphoreGive(remote_control.mutex);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish status");   if (msg_id < 0) {
        return ESP_FAIL;atus");
    }       return ESP_FAIL;
        }
    ESP_LOGI(TAG, "Status published successfully, msg_id=%d", msg_id); 
    return ESP_OK;ssfully, msg_id=%d", msg_id);
} return ESP_OK;

/**
 * @brief Publish room status update
 */
esp_err_t remote_control_publish_status(const room_status_t *status)
{err_t remote_control_publish_status(const room_status_t *status)
    return publish_status_internal(status);
}

/**
 * @brief Request immediate status update
 */
esp_err_t remote_control_request_update(void)
{)
    if (!remote_control.status_callback) {
        return ESP_ERR_INVALID_STATE;
    }       return ESP_ERR_INVALID_STATE;
        }
    room_status_t status = remote_control.status_callback(remote_control.status_callback_ctx); 
    return publish_status_internal(&status);llback(remote_control.status_callback_ctx);
} return publish_status_internal(&status);

/**
 * @brief Get memory usage of the remote control component
 */ontrol component
size_t remote_control_get_memory_usage(void)
{_memory_usage(void)
    return s_total_topics_memory + s_total_user_props_memory;
}_user_props_memory;

/**
 * @brief Deinitialize the remote control component
 */component
esp_err_t remote_control_deinit(void)
{
    // Stop the component first
    esp_err_t ret = remote_control_stop();
    
    // Clean up resources
    if (!remote_control.using_static_topics) {ean up resources
        if (remote_control.topic_prefix) {f (!remote_control.using_static_topics) {
            free(remote_control.topic_prefix);    if (remote_control.topic_prefix) {
            remote_control.topic_prefix = NULL;fix);
        }L;
        if (remote_control.status_topic) {
            free(remote_control.status_topic);   if (remote_control.status_topic) {
            remote_control.status_topic = NULL;        free(remote_control.status_topic);
        }us_topic = NULL;
        if (remote_control.command_topic) {
            free(remote_control.command_topic);opic) {
            remote_control.command_topic = NULL;       free(remote_control.command_topic);
        }        remote_control.command_topic = NULL;
    }
    
    if (remote_control.user_properties && remote_control.owns_user_properties) {
        free(remote_control.user_properties);f (remote_control.user_properties) {
        s_total_user_props_memory = 0;    free(remote_control.user_properties);
    }properties = NULL;
    remote_control.user_properties = NULL;
    
    if (remote_control.mutex) {if (remote_control.mutex) {
        vSemaphoreDelete(remote_control.mutex);te_control.mutex);
        remote_control.mutex = NULL;
    }}
    
    if (remote_control.report_timer) {   if (remote_control.report_timer) {
        esp_timer_delete(remote_control.report_timer);        esp_timer_delete(remote_control.report_timer);













}    return ret;        memset(&remote_control, 0, sizeof(remote_control_t));    // Zero out the structure        s_total_user_props_memory = 0;    s_total_topics_memory = 0;    // Reset memory tracking        }        remote_control.report_timer = NULL;        remote_control.report_timer = NULL;
    }
    
    // Reset memory tracking
    s_total_topics_memory = 0;
    s_total_user_props_memory = 0;
    
    // Zero out the structure
    memset(&remote_control, 0, sizeof(remote_control_t));
    
    return ret;
}
