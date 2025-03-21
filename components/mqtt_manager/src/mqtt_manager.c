/**
 * @file mqtt_manager.c
 * @brief Generic MQTT v5 client manager for ESP-IDF
 * 
 * Implementation of the MQTT manager component with topic registration
 * and message routing capabilities.
 * 
 * @author NieRVoid
 * @date 2025-03-20
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "mqtt_manager.h"

static const char *TAG = "mqtt-manager";

// Define initial handler array capacity
#define INITIAL_HANDLER_CAPACITY 8
#define REALLOC_MULTIPLIER 1.5

// Topic handler structure
typedef struct {
    int id;                      // Unique handler ID
    char *topic;                 // Topic filter
    int qos;                     // QoS level
    mqtt_message_handler_t fn;   // Callback function
    void *user_data;             // User data pointer
    bool active;                 // Whether this handler is active
    bool subscribed;             // Whether we've successfully subscribed to this topic
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
    
    // Initialize MQTT client
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
                                      void *user_data, int *handler_id)
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
    
    // Allocate topic string
    mqtt_manager.handlers[index].topic = strdup(topic);
    if (mqtt_manager.handlers[index].topic == NULL) {
        xSemaphoreGive(mqtt_manager.handlers_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize handler
    mqtt_manager.handlers[index].id = mqtt_manager.next_handler_id++;
    mqtt_manager.handlers[index].qos = qos;
    mqtt_manager.handlers[index].fn = handler;
    mqtt_manager.handlers[index].user_data = user_data;
    mqtt_manager.handlers[index].active = true;
    mqtt_manager.handlers[index].subscribed = false;
    
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
            
            // Free topic string
            free(mqtt_manager.handlers[i].topic);
            mqtt_manager.handlers[i].topic = NULL;
            mqtt_manager.handlers[i].active = false;
            
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
                        int qos, int retain)
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
        
        // Free all handler topics
        if (mqtt_manager.handlers) {
            for (int i = 0; i < mqtt_manager.handler_count; i++) {
                if (mqtt_manager.handlers[i].active && mqtt_manager.handlers[i].topic) {
                    free(mqtt_manager.handlers[i].topic);
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
    
    // Create a null-terminated copy of the topic for matching
    char *topic_str = malloc(topic_len + 1);
    if (topic_str == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for topic copy");
        xSemaphoreGive(mqtt_manager.handlers_mutex);
        return;
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
                free(topic_str);
                return;
            }
        }
    }
    
    free(topic_str);
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
   // Create a null-terminated copy of the topic if it isn't already
   bool allocated = false;
   char *topic_str = (char *)topic;
   
   if (topic[topic_len] != '\0') {
       topic_str = malloc(topic_len + 1);
       if (topic_str == NULL) {
           ESP_LOGE(TAG, "Failed to allocate memory for topic in matching");
           return false;
       }
       memcpy(topic_str, topic, topic_len);
       topic_str[topic_len] = '\0';
       allocated = true;
   }
   
   // Algorithm to match MQTT topics with wildcards
   const char *p = pattern;
   const char *t = topic_str;
   const char *p_end = pattern + strlen(pattern);
   const char *t_end = topic_str + strlen(topic_str);
   
   while (p < p_end && t < t_end) {
       if (*p == '+') {
           // '+' matches exactly one level, so find the next '/'
           p++;
           // Skip to the next level in the topic
           while (t < t_end && *t != '/') {
               t++;
           }
       } else if (*p == '#') {
           // '#' must be the last character and matches all remaining levels
           if (p + 1 == p_end) {
               // Match successful
               if (allocated) {
                   free(topic_str);
               }
               return true;
           } else {
               // Invalid pattern - # must be the last character
               ESP_LOGW(TAG, "Invalid MQTT pattern: # must be the last character");
               if (allocated) {
                   free(topic_str);
               }
               return false;
           }
       } else if (*p == *t) {
           // Exact character match
           p++;
           t++;
       } else {
           // No match
           if (allocated) {
               free(topic_str);
           }
           return false;
       }
       
       // Handle level transitions (both pattern and topic at '/')
       if (p < p_end && t < t_end && *p == '/' && *t == '/') {
           p++;
           t++;
       }
   }
   
   // Match if we've consumed both strings completely
   bool match = (p == p_end && t == t_end);
   
   if (allocated) {
       free(topic_str);
   }
   
   return match;
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
   
   mqtt_topic_handler_t *new_handlers = realloc(
       mqtt_manager.handlers, 
       new_capacity * sizeof(mqtt_topic_handler_t)
   );
   
   if (new_handlers == NULL) {
       ESP_LOGE(TAG, "Failed to expand handlers array");
       return ESP_ERR_NO_MEM;
   }
   
   // Initialize new slots
   for (int i = mqtt_manager.handler_capacity; i < new_capacity; i++) {
       memset(&new_handlers[i], 0, sizeof(mqtt_topic_handler_t));
   }
   
   mqtt_manager.handlers = new_handlers;
   mqtt_manager.handler_capacity = new_capacity;
   
   return ESP_OK;
}