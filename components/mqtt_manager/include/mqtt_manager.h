/**
 * @file mqtt_manager.h
 * @brief Generic MQTT v5 client manager for ESP-IDF
 * 
 * Provides a reusable MQTT v5 client component with topic registration
 * and message routing capabilities.
 * 
 * Updated to support MQTT v5 properties and user properties.
 * 
 * @date 2025-03-20
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "mqtt_client.h"
#include "mqtt5_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MQTT message handler callback function type
 * 
 * @param topic Topic string
 * @param topic_len Topic string length
 * @param data Message payload
 * @param data_len Message payload length
 * @param user_data User context pointer
 */
typedef void (*mqtt_message_handler_t)(const char *topic, int topic_len, 
                                      const char *data, int data_len, 
                                      void *user_data);

/**
 * @brief MQTT disconnect event handler callback function type
 * 
 * @param user_data User context pointer
 */
typedef void (*mqtt_disconnect_handler_t)(void *user_data);

/**
 * @brief MQTT connect event handler callback function type
 * 
 * @param user_data User context pointer
 */
typedef void (*mqtt_connect_handler_t)(void *user_data);

/**
 * @brief MQTT client configuration
 */
typedef struct {
    const char *broker_uri;       /*!< MQTT broker URI */
    const char *client_id;        /*!< MQTT client ID */
    const char *username;         /*!< MQTT username or NULL */
    const char *password;         /*!< MQTT password or NULL */
    bool use_ssl;                 /*!< Enable SSL/TLS */
    const char *cert_pem;         /*!< SSL Certificate in PEM format or NULL */
    int keepalive;                /*!< Keepalive time in seconds */
    bool clean_session;           /*!< Start a clean session */
    
    // Last Will and Testament
    const char *lwt_topic;        /*!< LWT topic or NULL */
    const char *lwt_msg;          /*!< LWT message or NULL */
    int lwt_qos;                  /*!< LWT QoS level */
    int lwt_retain;               /*!< LWT retain flag */
    
    // Event handlers
    mqtt_connect_handler_t connect_handler;     /*!< Connect event handler or NULL */
    void *connect_handler_ctx;                 /*!< Connect handler context */
    mqtt_disconnect_handler_t disconnect_handler; /*!< Disconnect event handler or NULL */
    void *disconnect_handler_ctx;              /*!< Disconnect handler context */

    // MQTT v5 properties
    esp_mqtt5_connection_property_config_t connection_property; /*!< MQTT v5 connection properties */
} mqtt_manager_config_t;

/**
 * @brief Default MQTT manager configuration
 */
#define MQTT_MANAGER_DEFAULT_CONFIG() {             \
    .broker_uri = "mqtt://mqtt.example.com:1883",   \
    .client_id = "esp32-device",                    \
    .username = NULL,                               \
    .password = NULL,                               \
    .use_ssl = false,                               \
    .cert_pem = NULL,                               \
    .keepalive = 120,                               \
    .clean_session = true,                          \
    .lwt_topic = NULL,                              \
    .lwt_msg = NULL,                                \
    .lwt_qos = 0,                                   \
    .lwt_retain = 0,                                \
    .connect_handler = NULL,                        \
    .connect_handler_ctx = NULL,                    \
    .disconnect_handler = NULL,                     \
    .disconnect_handler_ctx = NULL,                 \
    .connection_property = {0},                     \
}

/**
 * @brief MQTT v5 user property item
 */
typedef struct {
    const char *key;   /*!< User property key */
    const char *value; /*!< User property value */
} mqtt_user_property_t;

/**
 * @brief Initialize the MQTT manager
 * 
 * @param config Configuration parameters
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_manager_init(const mqtt_manager_config_t *config);

/**
 * @brief Register a message handler for a specific topic
 * 
 * @param topic Topic filter to subscribe to (supports wildcards +, #)
 * @param qos QoS level (0, 1, or 2)
 * @param handler Callback function for incoming messages
 * @param user_data User context passed to the callback
 * @param user_properties Array of user properties (optional)
 * @param user_property_count Number of user properties
 * @param handler_id Pointer to store the generated handler ID
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_manager_register_handler(const char *topic, int qos,
                                      mqtt_message_handler_t handler,
                                      void *user_data,
                                      const mqtt_user_property_t *user_properties,
                                      int user_property_count,
                                      int *handler_id);

/**
 * @brief Unregister a message handler
 * 
 * @param handler_id Handler ID returned from mqtt_manager_register_handler
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_manager_unregister_handler(int handler_id);

/**
 * @brief Publish a message to an MQTT topic
 * 
 * @param topic Topic to publish to
 * @param data Message payload
 * @param len Message payload length (use -1 for null-terminated strings)
 * @param qos QoS level (0, 1, or 2)
 * @param retain Retain flag
 * @param user_properties Array of user properties (optional)
 * @param user_property_count Number of user properties
 * @return int Message ID on success, negative error code otherwise
 */
int mqtt_manager_publish(const char *topic, const void *data, int len, 
                        int qos, int retain,
                        const mqtt_user_property_t *user_properties,
                        int user_property_count);

/**
 * @brief Start the MQTT manager
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_manager_start(void);

/**
 * @brief Stop the MQTT manager
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_manager_stop(void);

/**
 * @brief Check if MQTT manager is connected to broker
 * 
 * @return true if connected, false otherwise
 */
bool mqtt_manager_is_connected(void);

/**
 * @brief Check if MQTT manager is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool mqtt_manager_is_initialized(void);

/**
 * @brief Deinitialize the MQTT manager
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_manager_deinit(void);

/**
 * @brief Get the current memory usage of the MQTT manager
 * 
 * @param handlers_memory Pointer to store handlers memory usage (bytes)
 * @param topics_memory Pointer to store topics memory usage (bytes)
 * @param properties_memory Pointer to store properties memory usage (bytes)
 * @return Total memory usage in bytes
 */
size_t mqtt_manager_get_memory_usage(size_t *handlers_memory, size_t *topics_memory, size_t *properties_memory);

#ifdef __cplusplus
}
#endif
