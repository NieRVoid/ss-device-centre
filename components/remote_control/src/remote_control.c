/**
 * @file remote_control.c
 * @brief MQTT-based remote control for room management system
 * 
 * Implementation of the remote control component using the mqtt_manager component
 * 
 * Updated to use the mqtt_manager component for MQTT operations.
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

typedef struct {
    SemaphoreHandle_t mutex;
    esp_timer_handle_t report_timer;
    char *topic_prefix;
    char *status_topic;
    char *command_topic;
    bool running;
    int command_handler_id;
    
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
    
    // Prepare topics
    size_t prefix_len = strlen(config->topic_prefix);
    
    remote_control.topic_prefix = malloc(prefix_len + 1);
    if (!remote_control.topic_prefix) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    strcpy(remote_control.topic_prefix, config->topic_prefix);
    
    // Create full topic paths
    size_t status_topic_len = prefix_len + strlen(TOPIC_STATUS) + 1; // +1 for '/'
    remote_control.status_topic = malloc(status_topic_len + 1);      // +1 for null terminator
    if (!remote_control.status_topic) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    snprintf(remote_control.status_topic, status_topic_len + 1, "%s%s", 
             config->topic_prefix, TOPIC_STATUS);
    
    size_t command_topic_len = prefix_len + strlen(TOPIC_COMMAND) + 1;
    remote_control.command_topic = malloc(command_topic_len + 1);
    if (!remote_control.command_topic) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    snprintf(remote_control.command_topic, command_topic_len + 1, "%s%s", 
             config->topic_prefix, TOPIC_COMMAND);
    
    // Log topic information
    ESP_LOGI(TAG, "Status topic: %s", remote_control.status_topic);
    ESP_LOGI(TAG, "Command topic: %s", remote_control.command_topic);
    
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
    if (remote_control.topic_prefix) free(remote_control.topic_prefix);
    if (remote_control.status_topic) free(remote_control.status_topic);
    if (remote_control.command_topic) free(remote_control.command_topic);
    if (remote_control.mutex) vSemaphoreDelete(remote_control.mutex);
    if (remote_control.report_timer) esp_timer_delete(remote_control.report_timer);
    
    return ret;
}

/**
 * @brief Start the remote control background task
 */
esp_err_t remote_control_start(void)
{
    ESP_LOGI(TAG, "Starting remote control component");
    
    // Register for the command topic with MQTT manager
    int handler_id;
    esp_err_t ret = mqtt_manager_register_handler(
        remote_control.command_topic,
        1, // QoS 1
        mqtt_message_handler,
        NULL,
        NULL,
        0,
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
    
    // Copy data to ensure null-termination
    char *command = malloc(data_len + 1);
    if (command) {
        memcpy(command, data, data_len);
        command[data_len] = '\0';
        
        // Process the command
        process_mqtt_command(command, data, data_len);
        
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
    if (strcmp(command, CMD_STR_REQUEST_STATUS) == 0) {
        return REMOTE_CMD_REQUEST_STATUS;
    } else if (strcmp(command, CMD_STR_SET_OCCUPIED) == 0) {
        return REMOTE_CMD_SET_OCCUPIED;
    } else if (strcmp(command, CMD_STR_SET_VACANT) == 0) {
        return REMOTE_CMD_SET_VACANT;
    } else if (strcmp(command, CMD_STR_RESTART_DEVICE) == 0) {
        return REMOTE_CMD_RESTART_DEVICE;
    } else if (strcmp(command, CMD_STR_CONFIG_UPDATE) == 0) {
        return REMOTE_CMD_CONFIG_UPDATE;
    } else {
        return REMOTE_CMD_UNKNOWN;
    }
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
     
     // Convert room state to string
     const char *state_str = "unknown";
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
     
     // Format status as JSON
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
              status->source_name ? status->source_name : "unknown",
              status->source_reliability,
              esp_timer_get_time() / 1000 // microseconds to milliseconds
     );
     
     // Publish the status using MQTT manager
     ESP_LOGI(TAG, "Publishing status: %s", json_buffer);
     int msg_id = mqtt_manager_publish(
         remote_control.status_topic,
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
 * @brief Deinitialize the remote control component
 */
esp_err_t remote_control_deinit(void)
{
    // Stop the component first
    esp_err_t ret = remote_control_stop();
    
    // Clean up resources
    if (remote_control.topic_prefix) free(remote_control.topic_prefix);
    if (remote_control.status_topic) free(remote_control.status_topic);
    if (remote_control.command_topic) free(remote_control.command_topic);
    if (remote_control.mutex) vSemaphoreDelete(remote_control.mutex);
    if (remote_control.report_timer) esp_timer_delete(remote_control.report_timer);
    
    // Zero out the structure
    memset(&remote_control, 0, sizeof(remote_control_t));
    
    return ret;
}
