/**
 * @file remote_control.h
 * @brief MQTT-based remote control for room management
 * 
 * Provides MQTT communication for remote monitoring and control of room status.
 * This component is designed to be independent of the occupancy management system,
 * communicating through well-defined callback interfaces.
 * 
 * @author NieRVoid
 * @date 2025-03-19
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Commands that can be received from remote control
 */
typedef enum {
    REMOTE_CMD_UNKNOWN = 0,      /*!< Unknown command */
    REMOTE_CMD_REQUEST_STATUS,   /*!< Request immediate status report */
    REMOTE_CMD_SET_OCCUPIED,     /*!< Set room as occupied */
    REMOTE_CMD_SET_VACANT,       /*!< Set room as vacant */
    REMOTE_CMD_RESTART_DEVICE,   /*!< Restart the device */
    REMOTE_CMD_CONFIG_UPDATE     /*!< Update configuration */
} remote_command_t;

/**
 * @brief Room occupancy state
 */
typedef enum {
    ROOM_STATE_UNKNOWN = 0,   /*!< State unknown */
    ROOM_STATE_VACANT,        /*!< Room is vacant */
    ROOM_STATE_OCCUPIED       /*!< Room is occupied */
} room_state_t;

/**
 * @brief Room status information structure
 */
typedef struct {
    room_state_t state;            /*!< Current room state */
    int occupant_count;            /*!< Number of occupants (-1 if unknown) */
    bool count_reliable;           /*!< Whether the count is reliable */
    const char *source_name;       /*!< Name of determining source */
    int source_reliability;        /*!< Reliability of determining source (0-3) */
} room_status_t;

/**
 * @brief Command callback function type
 * 
 * Called when a command is received from remote control
 */
typedef void (*remote_command_callback_t)(remote_command_t command, const char *payload, void *user_ctx);

/**
 * @brief Status request callback function type
 * 
 * Called when a status report is requested or due
 */
typedef room_status_t (*status_request_callback_t)(void *user_ctx);

/**
 * @brief MQTT remote control configuration
 */
typedef struct {
    const char *broker_uri;       /*!< MQTT broker URI */
    const char *client_id;        /*!< MQTT client ID */
    const char *username;         /*!< MQTT username or NULL */
    const char *password;         /*!< MQTT password or NULL */
    const char *topic_prefix;     /*!< Topic prefix for all pub/sub operations */
    bool use_ssl;                 /*!< Enable SSL/TLS */
    uint32_t status_interval_sec; /*!< Status reporting interval in seconds */
    remote_command_callback_t command_callback; /*!< Command callback function */
    void *command_callback_ctx;   /*!< Command callback context */
    status_request_callback_t status_callback; /*!< Status callback function */
    void *status_callback_ctx;    /*!< Status callback context */
} remote_control_config_t;

/**
 * @brief Default remote control configuration
 */
#define REMOTE_CONTROL_DEFAULT_CONFIG() {             \
    .broker_uri = "mqtt://mqtt.example.com:1883",     \
    .client_id = "esp32-room-control",                \
    .username = NULL,                                 \
    .password = NULL,                                 \
    .topic_prefix = "homestay/room/",                 \
    .use_ssl = false,                                 \
    .status_interval_sec = 300,                       \
    .command_callback = NULL,                         \
    .command_callback_ctx = NULL,                     \
    .status_callback = NULL,                          \
    .status_callback_ctx = NULL,                      \
}

/**
 * @brief Initialize the remote control component
 * 
 * @param config Configuration parameters
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t remote_control_init(const remote_control_config_t *config);

/**
 * @brief Start the remote control background task
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t remote_control_start(void);

/**
 * @brief Stop the remote control background task
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t remote_control_stop(void);

/**
 * @brief Publish room status update
 * 
 * @param status Current room status
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t remote_control_publish_status(const room_status_t *status);

/**
 * @brief Request immediate status update
 * 
 * Forces a status update outside the regular reporting interval
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t remote_control_request_update(void);

/**
 * @brief Deinitialize the remote control component
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t remote_control_deinit(void);

#ifdef __cplusplus
}
#endif