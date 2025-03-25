/**
 * @file mqtt_manager.c
 * @brief Generic MQTT v5 client manager for ESP-IDF
 * 
 * Implementation of the MQTT manager component with topic registration
 * and message routing capabilities.
 * 
 * Updated to support MQTT v5 properties and user properties.
 * 
 * @date 2025-03-20
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "mqtt5_client.h"
#include "mqtt_manager.h"

static const char *TAG = "mqtt-manager";

// Define initial handler array capacity
#define INITIAL_HANDLER_CAPACITY 8
#define REALLOC_MULTIPLIER 1.5

// Add memory tracking variables
static size_t s_total_handlers_memory = 0;
static size_t s_total_topics_memory = 0;
static size_t s_total_properties_memory = 0;

// Define topic optimization threshold
#define SMALL_TOPIC_MAX_LEN 32

// Add a static buffer pool for small topics
typedef struct {
    char data[SMALL_TOPIC_MAX_LEN + 1];
    bool used;
} small_topic_buffer_t;

#define SMALL_TOPIC_POOL_SIZE 8
static small_topic_buffer_t s_small_topic_pool[SMALL_TOPIC_POOL_SIZE] = {0};

// Function to get small topic buffer
static char* get_small_topic_buffer(const char* topic) {
    size_t len = strlen(topic);
    if (len > SMALL_TOPIC_MAX_LEN) {
        return NULL;
    }
    
    for (int i = 0; i < SMALL_TOPIC_POOL_SIZE; i++) {
        if (!s_small_topic_pool[i].used) {
            s_small_topic_pool[i].used = true;
            strcpy(s_small_topic_pool[i].data, topic);
            return s_small_topic_pool[i].data;
        }
    }
    return NULL; // No buffer available
}

// Function to release small topic buffer
static void release_small_topic_buffer(char* buffer) {
    for (int i = 0; i < SMALL_TOPIC_POOL_SIZE; i++) {
        if (s_small_topic_pool[i].data == buffer) {
            s_small_topic_pool[i].used = false;
            return;
        }
    }
}

// Topic handler structure
typedef struct {
    int id;                      // Unique handler ID
    char *topic;                 // Topic filter
    int qos;                     // QoS level
    mqtt_message_handler_t fn;   // Callback function
    void *user_data;             // User data pointer
    bool active;                 // Whether this handler is active
    bool subscribed;             // Whether we've successfully subscribed to this topic
    mqtt_user_property_t *user_properties; // User properties
    int user_property_count;     // Number of user properties
} mqtt_topic_handler_t;

// MQTT manager structure
typedef struct {
    esp_mqtt_client_handle_t client;          // MQTT client handle
    mqtt_topic_handler_t *handlers;           // Array of topic handlers
    int handler_count;                        // Number of handlers in use
    int handler_capacity;                     // Allocated capacity for handlers
    int next_handler_id;                      // Next handler ID to assign
    SemaphoreHandle_t handlers_mutex;         // Mutex for handler operations
    SemaphoreHandle_t client_mutex;           // Mutex for client operations
    bool connected;                           // Connection state
    mqtt_connect_handler_t connect_handler;   // Connect event handler
    void *connect_handler_ctx;                // Connect handler context
    mqtt_disconnect_handler_t disconnect_handler; // Disconnect event handler
    void *disconnect_handler_ctx;             // Disconnect handler context
} mqtt_manager_t;

static mqtt_manager_t mqtt_manager = {0};

// Function prototypes
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static esp_err_t resubscribe_to_all_topics(void);
static void route_message(const char *topic, int topic_len, const char *data, int data_len);
static bool topic_matches(const char *pattern, const char *topic, int topic_len);
static esp_err_t expand_handler_array(void);
static esp_err_t subscribe_handler(mqtt_topic_handler_t *handler);
static esp_err_t set_user_properties(mqtt5_user_property_handle_t *user_property_handle, const mqtt_user_property_t *user_properties, int user_property_count);

/**
 * @brief Initialize the MQTT manager
 */
esp_err_t mqtt_manager_init(const mqtt_manager_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing MQTT manager with MQTT v5");
    
    // Create mutexes
    mqtt_manager.handlers_mutex = xSemaphoreCreateMutex();
    if (mqtt_manager.handlers_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create handlers mutex");
        return ESP_ERR_NO_MEM;
    }
    
    mqtt_manager.client_mutex = xSemaphoreCreateMutex();
    if (mqtt_manager.client_mutex == NULL) {
        vSemaphoreDelete(mqtt_manager.handlers_mutex);
        ESP_LOGE(TAG, "Failed to create client mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize handlers array
    mqtt_manager.handler_capacity = INITIAL_HANDLER_CAPACITY;
    mqtt_manager.handlers = calloc(mqtt_manager.handler_capacity, sizeof(mqtt_topic_handler_t));
    if (mqtt_manager.handlers == NULL) {
        vSemaphoreDelete(mqtt_manager.handlers_mutex);
        vSemaphoreDelete(mqtt_manager.client_mutex);
        ESP_LOGE(TAG, "Failed to allocate handlers array");
        return ESP_ERR_NO_MEM;
    }
    
    mqtt_manager.handler_count = 0;
    mqtt_manager.next_handler_id = 1;
    
    // Save connection/disconnection handlers
    mqtt_manager.connect_handler = config->connect_handler;
    mqtt_manager.connect_handler_ctx = config->connect_handler_ctx;
    mqtt_manager.disconnect_handler = config->disconnect_handler;
    mqtt_manager.disconnect_handler_ctx = config->disconnect_handler_ctx;
    
    ESP_LOGI(TAG, "MQTT broker: %s", config->broker_uri);
    
    // Initialize MQTT client with v5 properties
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = config->broker_uri,
        .credentials.client_id = config->client_id,
        .credentials.username = config->username,
        .credentials.authentication.password = config->password,
        .broker.verification.certificate = config->cert_pem,
        .session.keepalive = config->keepalive,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .session.last_will.topic = config->lwt_topic,
        .session.last_will.msg = config->lwt_msg,
        .session.last_will.qos = config->lwt_qos,
        .session.last_will.retain = config->lwt_retain,
        .session.disable_clean_session = !config->clean_session,
    };
    
    mqtt_manager.client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_manager.client == NULL) {
        free(mqtt_manager.handlers);
        vSemaphoreDelete(mqtt_manager.handlers_mutex);
        vSemaphoreDelete(mqtt_manager.client_mutex);
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    // Set MQTT v5 connection properties
    esp_mqtt5_client_set_connect_property(mqtt_manager.client, &config->connection_property);
    
    // Register MQTT event handler
    esp_err_t ret = esp_mqtt_client_register_event(mqtt_manager.client, 
                                                  ESP_EVENT_ANY_ID,
                                                  mqtt_event_handler, 
                                                  NULL);
    if (ret != ESP_OK) {
        esp_mqtt_client_destroy(mqtt_manager.client);
        free(mqtt_manager.handlers);
        vSemaphoreDelete(mqtt_manager.handlers_mutex);
        vSemaphoreDelete(mqtt_manager.client_mutex);
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Reset memory tracking
    s_total_handlers_memory = INITIAL_HANDLER_CAPACITY * sizeof(mqtt_topic_handler_t);
    s_total_topics_memory = 0;
    s_total_properties_memory = 0;
    
    // Clear small topic pool
    memset(s_small_topic_pool, 0, sizeof(s_small_topic_pool));
    
    return ESP_OK;
}

/**
 * @brief Start the MQTT manager
 */
esp_err_t mqtt_manager_start(void)
{
    if (mqtt_manager.client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting MQTT manager");
    
    return esp_mqtt_client_start(mqtt_manager.client);
}

/**
 * @brief Stop the MQTT manager
 */
esp_err_t mqtt_manager_stop(void)
{
    if (mqtt_manager.client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Stopping MQTT manager");
    
    return esp_mqtt_client_stop(mqtt_manager.client);
}

/**
 * @brief Check if MQTT manager is connected
 */
bool mqtt_manager_is_connected(void)
{
    return mqtt_manager.connected;
}

/**
 * @brief Register a message handler for a specific topic
 */
esp_err_t mqtt_manager_register_handler(const char *topic, int qos,
                                      mqtt_message_handler_t handler,
                                      void *user_data,
                                      const mqtt_user_property_t *user_properties,
                                      int user_property_count,
                                      int *handler_id)
{
    if (topic == NULL || handler == NULL || qos < 0 || qos > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(mqtt_manager.handlers_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take handlers mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if we need to expand the handlers array
    if (mqtt_manager.handler_count >= mqtt_manager.handler_capacity) {
        esp_err_t ret = expand_handler_array();
        if (ret != ESP_OK) {
            xSemaphoreGive(mqtt_manager.handlers_mutex);
            return ret;
        }
    }
    
    // Find an empty slot or use the next available one
    int index = -1;
    for (int i = 0; i < mqtt_manager.handler_count; i++) {
        if (!mqtt_manager.handlers[i].active) {
            index = i;
            break;
        }
    }
    
    if (index == -1) {
        index = mqtt_manager.handler_count++;
    }
    
    // First try to use a small topic buffer for common topics
    mqtt_manager.handlers[index].topic = get_small_topic_buffer(topic);
    
    // If small buffer not available, fall back to dynamic allocation
    if (mqtt_manager.handlers[index].topic == NULL) {
        mqtt_manager.handlers[index].topic = strdup(topic);
        if (mqtt_manager.handlers[index].topic == NULL) {
            xSemaphoreGive(mqtt_manager.handlers_mutex);
            return ESP_ERR_NO_MEM;
        }
        // Track topic memory
        s_total_topics_memory += strlen(topic) + 1;
    }
    
    // Initialize handler
    mqtt_manager.handlers[index].id = mqtt_manager.next_handler_id++;
    mqtt_manager.handlers[index].qos = qos;
    mqtt_manager.handlers[index].fn = handler;
    mqtt_manager.handlers[index].user_data = user_data;
    mqtt_manager.handlers[index].active = true;
    mqtt_manager.handlers[index].subscribed = false;

    // Optimize user property allocation
    if (user_properties != NULL && user_property_count > 0) {
        size_t props_size = user_property_count * sizeof(mqtt_user_property_t);
        mqtt_manager.handlers[index].user_properties = malloc(props_size);
        if (mqtt_manager.handlers[index].user_properties == NULL) {
            if (get_small_topic_buffer(topic) == NULL) {
                free(mqtt_manager.handlers[index].topic);
                s_total_topics_memory -= strlen(topic) + 1;
            } else {
                release_small_topic_buffer(mqtt_manager.handlers[index].topic);
            }
            xSemaphoreGive(mqtt_manager.handlers_mutex);
            return ESP_ERR_NO_MEM;
        }
        memcpy(mqtt_manager.handlers[index].user_properties, user_properties, props_size);
        mqtt_manager.handlers[index].user_property_count = user_property_count;
        
        // Track properties memory
        s_total_properties_memory += props_size;
    } else {
        mqtt_manager.handlers[index].user_properties = NULL;
        mqtt_manager.handlers[index].user_property_count = 0;
    }
    
    // Return handler ID
    if (handler_id != NULL) {
        *handler_id = mqtt_manager.handlers[index].id;
    }
    
    ESP_LOGI(TAG, "Registered handler %d for topic '%s' with QoS %d", 
             mqtt_manager.handlers[index].id, topic, qos);
    
    // Subscribe to topic if we're connected
    if (mqtt_manager.connected) {
        subscribe_handler(&mqtt_manager.handlers[index]);
    }
    
    xSemaphoreGive(mqtt_manager.handlers_mutex);
    
    return ESP_OK;
}

/**
 * @brief Unregister a message handler
 */
esp_err_t mqtt_manager_unregister_handler(int handler_id)
{
    if (handler_id <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(mqtt_manager.handlers_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take handlers mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Find the handler with the given ID
    for (int i = 0; i < mqtt_manager.handler_count; i++) {
        if (mqtt_manager.handlers[i].active && mqtt_manager.handlers[i].id == handler_id) {
            ESP_LOGI(TAG, "Unregistering handler %d for topic '%s'", 
                     handler_id, mqtt_manager.handlers[i].topic);
            
            // Check if using small topic buffer
            bool is_small_buffer = false;
            for (int j = 0; j < SMALL_TOPIC_POOL_SIZE; j++) {
                if (s_small_topic_pool[j].data == mqtt_manager.handlers[i].topic) {
                    is_small_buffer = true;
                    release_small_topic_buffer(mqtt_manager.handlers[i].topic);
                    break;
                }
            }
            
            // Free topic string if dynamically allocated
            if (!is_small_buffer) {
                s_total_topics_memory -= strlen(mqtt_manager.handlers[i].topic) + 1;
                free(mqtt_manager.handlers[i].topic);
            }
            mqtt_manager.handlers[i].topic = NULL;
            mqtt_manager.handlers[i].active = false;

            // Free user properties
            if (mqtt_manager.handlers[i].user_properties != NULL) {
                s_total_properties_memory -= mqtt_manager.handlers[i].user_property_count * sizeof(mqtt_user_property_t);
                free(mqtt_manager.handlers[i].user_properties);
                mqtt_manager.handlers[i].user_properties = NULL;
            }
            
            // No need to unsubscribe, as other handlers might still be interested in this topic
            // We'll clean up unused subscriptions only when disconnecting and reconnecting
            
            xSemaphoreGive(mqtt_manager.handlers_mutex);
            return ESP_OK;
        }
    }
    
    xSemaphoreGive(mqtt_manager.handlers_mutex);
    
    ESP_LOGW(TAG, "Handler %d not found", handler_id);
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Publish a message to an MQTT topic
 */
int mqtt_manager_publish(const char *topic, const void *data, int len, 
                        int qos, int retain,
                        const mqtt_user_property_t *user_properties,
                        int user_property_count)
{
    if (topic == NULL || (data == NULL && len > 0) || qos < 0 || qos > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!mqtt_manager.connected) {
        ESP_LOGW(TAG, "Cannot publish: not connected to broker");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(mqtt_manager.client_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take client mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // If len is -1, calculate the length for null-terminated strings
    if (len == -1 && data != NULL) {
        len = strlen((const char *)data);
    }
    
    ESP_LOGD(TAG, "Publishing to '%s' (QoS %d, retain %d): %.*s", 
             topic, qos, retain, len, (const char *)data);
    
    // Set MQTT v5 publish properties
    esp_mqtt5_publish_property_config_t publish_property = {0};
    publish_property.user_property = NULL;
    if (user_properties != NULL && user_property_count > 0) {
        set_user_properties(&publish_property.user_property, user_properties, user_property_count);
    }
    esp_mqtt5_client_set_publish_property(mqtt_manager.client, &publish_property);
    
    int msg_id = esp_mqtt_client_publish(mqtt_manager.client, topic, 
                                        data, len, qos, retain);
    
    xSemaphoreGive(mqtt_manager.client_mutex);
    
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to '%s'", topic);
    }
    
    return msg_id;
}

/**
 * @brief Deinitialize the MQTT manager
 */
esp_err_t mqtt_manager_deinit(void)
{
    if (mqtt_manager.client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Deinitializing MQTT manager");
    
    // Stop MQTT client
    esp_err_t ret = mqtt_manager_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(ret));
    }
    
    // Clean up resources
    if (mqtt_manager.client) {
        esp_mqtt_client_destroy(mqtt_manager.client);
        mqtt_manager.client = NULL;
    }
    
    if (mqtt_manager.handlers_mutex) {
        xSemaphoreTake(mqtt_manager.handlers_mutex, portMAX_DELAY);
        
        // Free all handler topics and user properties
        if (mqtt_manager.handlers) {
            for (int i = 0; i < mqtt_manager.handler_count; i++) {
                if (mqtt_manager.handlers[i].active && mqtt_manager.handlers[i].topic) {
                    free(mqtt_manager.handlers[i].topic);
                }
                if (mqtt_manager.handlers[i].user_properties) {
                    free(mqtt_manager.handlers[i].user_properties);
                }
            }
            free(mqtt_manager.handlers);
            mqtt_manager.handlers = NULL;
        }
        
        xSemaphoreGive(mqtt_manager.handlers_mutex);
        vSemaphoreDelete(mqtt_manager.handlers_mutex);
        mqtt_manager.handlers_mutex = NULL;
    }
    
    if (mqtt_manager.client_mutex) {
        vSemaphoreDelete(mqtt_manager.client_mutex);
        mqtt_manager.client_mutex = NULL;
    }
    
    // Reset state
    mqtt_manager.handler_count = 0;
    mqtt_manager.handler_capacity = 0;
    mqtt_manager.next_handler_id = 1;
    mqtt_manager.connected = false;
    mqtt_manager.connect_handler = NULL;
    mqtt_manager.connect_handler_ctx = NULL;
    mqtt_manager.disconnect_handler = NULL;
    mqtt_manager.disconnect_handler_ctx = NULL;
    
    // Reset memory tracking
    s_total_handlers_memory = 0;
    s_total_topics_memory = 0;
    s_total_properties_memory = 0;
    
    // Clear small topic pool
    memset(s_small_topic_pool, 0, sizeof(s_small_topic_pool));
    
    return ESP_OK;
}

/**
 * @brief MQTT event handler function
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                              int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT client connected to broker");
            mqtt_manager.connected = true;
            
            // Resubscribe to all topics
            resubscribe_to_all_topics();
            
            // Call connect handler if registered
            if (mqtt_manager.connect_handler) {
                mqtt_manager.connect_handler(mqtt_manager.connect_handler_ctx);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT client disconnected from broker");
            mqtt_manager.connected = false;
            
            // Mark all topics as unsubscribed
            if (xSemaphoreTake(mqtt_manager.handlers_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                for (int i = 0; i < mqtt_manager.handler_count; i++) {
                    if (mqtt_manager.handlers[i].active) {
                        mqtt_manager.handlers[i].subscribed = false;
                    }
                }
                xSemaphoreGive(mqtt_manager.handlers_mutex);
            }
            
            // Call disconnect handler if registered
            if (mqtt_manager.disconnect_handler) {
                mqtt_manager.disconnect_handler(mqtt_manager.disconnect_handler_ctx);
            }
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT_EVENT_DATA");
            ESP_LOGD(TAG, "Topic: %.*s", event->topic_len, event->topic);
            ESP_LOGD(TAG, "Data: %.*s", event->data_len, event->data);
            
            // Route message to appropriate handlers
            route_message(event->topic, event->topic_len, 
                         event->data, event->data_len);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last error code reported from ESP-TLS: 0x%x", 
                         event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Last tls stack error number: 0x%x", 
                         event->error_handle->esp_tls_stack_err);
                ESP_LOGE(TAG, "Last captured errno : %d (%s)",  
                         event->error_handle->esp_transport_sock_errno,
                         strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
            
        default:
            ESP_LOGD(TAG, "Other MQTT event id:%d", event->event_id);
            break;
    }
}

/**
 * @brief Resubscribe to all active topics
 */
static esp_err_t resubscribe_to_all_topics(void)
{
    if (!mqtt_manager.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(mqtt_manager.handlers_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take handlers mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    for (int i = 0; i < mqtt_manager.handler_count; i++) {
        if (mqtt_manager.handlers[i].active && !mqtt_manager.handlers[i].subscribed) {
            subscribe_handler(&mqtt_manager.handlers[i]);
        }
    }
    
    xSemaphoreGive(mqtt_manager.handlers_mutex);
    
    return ESP_OK;
}

/**
 * @brief Subscribe to a topic for a specific handler
 */
static esp_err_t subscribe_handler(mqtt_topic_handler_t *handler)
{
    if (!mqtt_manager.connected || handler == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(mqtt_manager.client_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take client mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "Subscribing to topic '%s' with QoS %d", 
             handler->topic, handler->qos);
    
    // Set MQTT v5 subscribe properties
    esp_mqtt5_subscribe_property_config_t subscribe_property = {0};
    subscribe_property.user_property = NULL;
    if (handler->user_properties != NULL && handler->user_property_count > 0) {
        set_user_properties(&subscribe_property.user_property, handler->user_properties, handler->user_property_count);
    }
    esp_mqtt5_client_set_subscribe_property(mqtt_manager.client, &subscribe_property);
    
    int msg_id = esp_mqtt_client_subscribe(mqtt_manager.client, 
                                         handler->topic, handler->qos);
    
    xSemaphoreGive(mqtt_manager.client_mutex);
    
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to topic '%s'", handler->topic);
        return ESP_FAIL;
    }
    
    handler->subscribed = true;
    return ESP_OK;
}

/**
 * @brief Route message to appropriate handlers
 */
static void route_message(const char *topic, int topic_len, 
                         const char *data, int data_len)
{
    if (xSemaphoreTake(mqtt_manager.handlers_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take handlers mutex for routing");
        return;
    }
    
    // Use stack allocation for topic instead of heap
    char topic_buf[128]; // Use a reasonably sized stack buffer
    char *topic_str;
    bool heap_allocated = false;
    
    if (topic_len < sizeof(topic_buf)) {
        // Use stack buffer if topic fits
        topic_str = topic_buf;
    } else {
        // Fall back to heap for very long topics
        topic_str = malloc(topic_len + 1);
        if (topic_str == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for topic copy");
            xSemaphoreGive(mqtt_manager.handlers_mutex);
            return;
        }
        heap_allocated = true;
    }
    
    memcpy(topic_str, topic, topic_len);
    topic_str[topic_len] = '\0';
    
    // Find handlers matching this topic
    for (int i = 0; i < mqtt_manager.handler_count; i++) {
        if (mqtt_manager.handlers[i].active && 
            topic_matches(mqtt_manager.handlers[i].topic, topic_str, topic_len)) {
            
            // Store handler details to avoid using them while mutex is released
            mqtt_message_handler_t fn = mqtt_manager.handlers[i].fn;
            void *user_data = mqtt_manager.handlers[i].user_data;
            
            // Release mutex before callback to prevent deadlocks
            xSemaphoreGive(mqtt_manager.handlers_mutex);
            
            // Call the handler
            fn(topic, topic_len, data, data_len, user_data);
            
            // Reacquire mutex
            if (xSemaphoreTake(mqtt_manager.handlers_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to retake handlers mutex after callback");
                if (heap_allocated) {
                    free(topic_str);
                }
                return;
            }
        }
    }
    
    // Clean up if heap allocated
    if (heap_allocated) {
        free(topic_str);
    }
    
    xSemaphoreGive(mqtt_manager.handlers_mutex);
}

/**
 * @brief Check if a topic matches a subscription pattern
 * 
 * MQTT wildcards supported:
 * + (plus) matches a single level
 * # (hash) matches multiple levels (must be the last character)
 * 
 * @param pattern Subscription pattern
 * @param topic Actual topic
 * @param topic_len Topic length
 * @return bool true if match, false otherwise
 */
static bool topic_matches(const char *pattern, const char *topic, int topic_len)
{
    // If topic is already null-terminated, use it directly
    if (topic[topic_len] == '\0') {
        // Direct matching algorithm without allocation
        // ...implement direct matching logic...
        
        // This is the matching algorithm from the original function
        const char *p = pattern;
        const char *t = topic;
        const char *p_end = pattern + strlen(pattern);
        const char *t_end = topic + topic_len;
        
        while (p < p_end && t < t_end) {
            if (*p == '+') {
                p++;
                while (t < t_end && *t != '/') {
                    t++;
                }
            } else if (*p == '#') {
                if (p + 1 == p_end) {
                    return true;
                } else {
                    ESP_LOGW(TAG, "Invalid MQTT pattern: # must be the last character");
                    return false;
                }
            } else if (*p == *t) {
                p++;
                t++;
            } else {
                return false;
            }
            
            if (p < p_end && t < t_end && *p == '/' && *t == '/') {
                p++;
                t++;
            }
        }
        
        return (p == p_end && t == t_end);
    } else {
        // Use stack allocation for small topics
        char stack_topic[128];
        char *topic_str;
        bool heap_allocated = false;
        
        if (topic_len < sizeof(stack_topic)) {
            topic_str = stack_topic;
        } else {
            topic_str = malloc(topic_len + 1);
            if (topic_str == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for topic in matching");
                return false;
            }
            heap_allocated = true;
        }
        
        memcpy(topic_str, topic, topic_len);
        topic_str[topic_len] = '\0';
        
        // Use the same matching algorithm as above
        bool result = topic_matches(pattern, topic_str, topic_len);
        
        if (heap_allocated) {
            free(topic_str);
        }
        
        return result;
    }
}

/**
 * @brief Expand the handlers array capacity
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
static esp_err_t expand_handler_array(void)
{
   int new_capacity = (int)(mqtt_manager.handler_capacity * REALLOC_MULTIPLIER);
   if (new_capacity <= mqtt_manager.handler_capacity) {
       new_capacity = mqtt_manager.handler_capacity + INITIAL_HANDLER_CAPACITY;
   }
   
   ESP_LOGI(TAG, "Expanding handlers capacity from %d to %d", 
            mqtt_manager.handler_capacity, new_capacity);
   
   size_t old_size = mqtt_manager.handler_capacity * sizeof(mqtt_topic_handler_t);
   size_t new_size = new_capacity * sizeof(mqtt_topic_handler_t);
   
   mqtt_topic_handler_t *new_handlers = realloc(
       mqtt_manager.handlers, 
       new_size
   );
   
   if (new_handlers == NULL) {
       ESP_LOGE(TAG, "Failed to expand handlers array");
       return ESP_ERR_NO_MEM;
   }
   
   // Track handlers memory
   s_total_handlers_memory += (new_size - old_size);
   
   // Initialize new slots
   for (int i = mqtt_manager.handler_capacity; i < new_capacity; i++) {
       memset(&new_handlers[i], 0, sizeof(mqtt_topic_handler_t));
   }
   
   mqtt_manager.handlers = new_handlers;
   mqtt_manager.handler_capacity = new_capacity;
   
   return ESP_OK;
}

/**
 * @brief Set user properties for MQTT v5 messages
 * 
 * @param user_property_handle Pointer to user property handle
 * @param user_properties Array of user properties
 * @param user_property_count Number of user properties
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
static esp_err_t set_user_properties(mqtt5_user_property_handle_t *user_property_handle, const mqtt_user_property_t *user_properties, int user_property_count)
{
    if (user_property_handle == NULL || user_properties == NULL || user_property_count <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_mqtt5_user_property_item_t *items = malloc(user_property_count * sizeof(esp_mqtt5_user_property_item_t));
    if (items == NULL) {
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < user_property_count; i++) {
        items[i].key = user_properties[i].key;
        items[i].value = user_properties[i].value;
    }

    esp_err_t ret = esp_mqtt5_client_set_user_property(user_property_handle, items, user_property_count);
    free(items);
    return ret;
}

/**
 * @brief Get memory usage of the MQTT manager
 * 
 * @param handlers_memory Pointer to store handlers memory usage
 * @param topics_memory Pointer to store topics memory usage
 * @param properties_memory Pointer to store properties memory usage
 * @return size_t Total memory usage
 */
size_t mqtt_manager_get_memory_usage(size_t *handlers_memory, size_t *topics_memory, size_t *properties_memory)
{
    if (handlers_memory) {
        *handlers_memory = s_total_handlers_memory;
    }
    
    if (topics_memory) {
        *topics_memory = s_total_topics_memory;
    }
    
    if (properties_memory) {
        *properties_memory = s_total_properties_memory;
    }
    
    return s_total_handlers_memory + s_total_topics_memory + s_total_properties_memory;
}