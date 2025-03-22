/**
 * @file main.c
 * @brief Multi-source occupancy detection system using LD2450 radar
 *
 * This application integrates four components:
 * - ld2450_driver: Radar sensor driver
 * - people_counter: People counting logic
 * - mqtt_manager: Generic MQTT communication
 * - remote_control: MQTT-based remote control interface
 *
 * Three occupancy sources are implemented:
 * 1. Radar-based people counting (high accuracy, continuous)
 * 2. BOOT button on GPIO0 (high reliability, triggered)
 * 3. MQTT remote control (medium reliability, triggered)
 * 
 * @date 2025-03-20
 * @license MIT
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "protocol_examples_common.h"

// Include components' headers
#include "ld2450.h"
#include "people_counter.h"
#include "mqtt_manager.h"
#include "remote_control.h"

// Include secrets header
#include "secrets.h"

static const char *TAG = "ss-device-cent";

// Simple occupancy source tracking
typedef struct {
    const char* name;
    int count;
    int64_t last_update_time;
    int timeout_ms;
    int reliability; // 1=low, 2=medium, 3=high
} occupancy_source_t;

// Define source IDs
#define SOURCE_RADAR 0
#define SOURCE_BUTTON 1
#define SOURCE_REMOTE 2
#define MAX_SOURCES 3

// Array of occupancy sources
static occupancy_source_t occupancy_sources[MAX_SOURCES] = {
    { "Radar", 0, 0, 5000, 1 },    // Radar updates every 5 seconds, low reliability
    { "Button", 0, 0, 30000, 3 },  // Button presence times out after 30 seconds, high reliability
    { "Remote", 0, 0, 60000, 2 }   // Remote presence times out after 60 seconds, medium reliability
};

// Queue for button events
static QueueHandle_t button_event_queue;

// Function prototypes
static void button_task(void *arg);
static void people_counter_callback(int count, int entries, int exits, void *context);
static void radar_target_callback(const ld2450_frame_t *frame, void *user_ctx);
static void button_isr_handler(void *arg);
static void print_radar_targets(const ld2450_frame_t *frame);
static const char* get_reliability_name(int reliability);

// Source management functions
static void update_source_count(int source_id, int count);
static int get_best_source(void);
static void get_occupancy_status(bool *is_occupied, int *count, int *source_id);
static void print_occupancy_status(bool is_occupied, int count, int source_id);

// LED control functions
static void send_led_control_message(bool power_on, int red, int green, int blue);
static void determine_led_color(int source_id, int count, int *r, int *g, int *b);
static void update_status_and_led(void);

// Remote control function prototypes
static void remote_command_handler(remote_command_t command, const char *payload, void *user_ctx);
static room_status_t get_room_status(void *user_ctx);

// MQTT manager function prototypes
static void mqtt_connect_handler(void *user_data);
static void mqtt_disconnect_handler(void *user_data);

// Stack sizes
#define BUTTON_TASK_STACK_SIZE 4096

/**
 * @brief Update source count and timestamp
 */
static void update_source_count(int source_id, int count) {
    if (source_id >= 0 && source_id < MAX_SOURCES) {
        occupancy_sources[source_id].count = count;
        occupancy_sources[source_id].last_update_time = esp_timer_get_time() / 1000; // Convert to ms
        
        // Update status when any source changes
        update_status_and_led();
    }
}

/**
 * @brief Get the best source (highest count, considering timeouts)
 * @return Source ID with highest count, or -1 if all sources have count 0
 */
static int get_best_source(void) {
    int best_source = -1;
    int highest_count = 0;
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    
    for (int i = 0; i < MAX_SOURCES; i++) {
        // Check if source has timed out
        if (current_time - occupancy_sources[i].last_update_time > occupancy_sources[i].timeout_ms) {
            // Source timed out, count is 0
            occupancy_sources[i].count = 0;
        }
        
        // Check if this source has a higher count
        if (occupancy_sources[i].count > highest_count) {
            highest_count = occupancy_sources[i].count;
            best_source = i;
        }
    }
    
    return best_source;
}

/**
 * @brief Get current occupancy status
 */
static void get_occupancy_status(bool *is_occupied, int *count, int *source_id) {
    int best_source = get_best_source();
    
    if (best_source >= 0) {
        *is_occupied = (occupancy_sources[best_source].count > 0);
        *count = occupancy_sources[best_source].count;
        *source_id = best_source;
    } else {
        *is_occupied = false;
        *count = 0;
        *source_id = -1;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());
    ESP_LOGI(TAG, "Connected to AP");

    esp_err_t ret;

    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║ Starting Occupancy Detection System    ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    
    // Initialize queue for button events
    button_event_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!button_event_queue) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return;
    }
    
    // ------------------------------------------------------------------------
    // 1. Initialize LD2450 radar driver
    // ------------------------------------------------------------------------
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    // radar_config.uart_port = UART_NUM_2;
    radar_config.uart_rx_pin = 16;
    radar_config.uart_tx_pin = 17;
    radar_config.uart_baud_rate = 256000;
    // radar_config.auto_processing = true;
    
    ESP_LOGI(TAG, "⚙️  Initializing LD2450 radar driver...");
    ret = ld2450_init(&radar_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize LD2450 radar: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register radar data callback
    ret = ld2450_register_target_callback(radar_target_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to register radar callback: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "✅ LD2450 radar initialized successfully");
    
    // ------------------------------------------------------------------------
    // 2. Initialize people counter
    // ------------------------------------------------------------------------
    people_counter_config_t counter_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    counter_config.vector_threshold = 1000;          // 100cm movement to count as entry/exit
    counter_config.empty_target_threshold = 5;       // 5 empty frames to consider target gone
    counter_config.detection_min_x = -2000;          // Detection area: 4m wide, 2m deep
    counter_config.detection_max_x = 2000;
    counter_config.detection_min_y = 0;
    counter_config.detection_max_y = 2000;

    counter_config.count_changed_cb = people_counter_callback;
    
    ESP_LOGI(TAG, "⚙️  Initializing people counter component...");
    ret = people_counter_init(&counter_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize people counter: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure detection region
    ret = people_counter_configure_region();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to configure detection region: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "✅ People counter initialized successfully");
    
    // ------------------------------------------------------------------------
    // 3. Configure BOOT button (GPIO0)
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "⚙️  Configuring BOOT button on GPIO0...");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .pull_up_en = 1,
        .pull_down_en = 0
    };
    
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_0, button_isr_handler, NULL);
    ESP_LOGI(TAG, "✅ BOOT button configured on GPIO0");
    
    // ------------------------------------------------------------------------
    // 4. Initialize MQTT manager
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "⚙️  Initializing MQTT manager...");
    
    mqtt_manager_config_t mqtt_config = MQTT_MANAGER_DEFAULT_CONFIG();
    mqtt_config.broker_uri = SECRET_MQTT_BROKER_URI;
    mqtt_config.client_id = SECRET_MQTT_CLIENT_ID;
    mqtt_config.username = SECRET_MQTT_USERNAME;
    mqtt_config.password = SECRET_MQTT_PASSWORD;
    mqtt_config.use_ssl = false;
    mqtt_config.keepalive = 120;
    mqtt_config.connect_handler = mqtt_connect_handler;
    mqtt_config.disconnect_handler = mqtt_disconnect_handler;
    
    // Configure LWT (Last Will and Testament)
    char lwt_topic[128];
    snprintf(lwt_topic, sizeof(lwt_topic), "%s%s", SECRET_MQTT_TOPIC_PREFIX, "status");
    mqtt_config.lwt_topic = lwt_topic;
    mqtt_config.lwt_msg = "{\"state\":\"disconnected\"}";
    mqtt_config.lwt_qos = 1;
    mqtt_config.lwt_retain = 1;
    
    ret = mqtt_manager_init(&mqtt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize MQTT manager: %s", esp_err_to_name(ret));
        // Continue even if MQTT fails, as other components might still work
    } else {
        ESP_LOGI(TAG, "✅ MQTT manager initialized successfully");
        
        // Start MQTT manager
        ret = mqtt_manager_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to start MQTT manager: %s", esp_err_to_name(ret));
            // Continue even if MQTT fails
        } else {
            ESP_LOGI(TAG, "✅ MQTT manager started successfully");
        }
    }

    // Wait for MQTT Manager to connect
    int wait_time_ms = 0;
    while (!mqtt_manager_is_connected() && wait_time_ms < 10000) { // Wait up to 10 seconds
        ESP_LOGI(TAG, "Waiting for MQTT Manager to connect...");
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100ms
        wait_time_ms += 100;
    }

    if (!mqtt_manager_is_connected()) {
        ESP_LOGE(TAG, "MQTT Manager failed to connect within the timeout period");
        return;
    }

    // ------------------------------------------------------------------------
    // 5. Initialize MQTT remote control
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "⚙️  Initializing MQTT remote control...");

    remote_control_config_t remote_config = REMOTE_CONTROL_DEFAULT_CONFIG();
    remote_config.topic_prefix = SECRET_MQTT_TOPIC_PREFIX;
    remote_config.status_interval_sec = 300; // 5 minute updates
    remote_config.command_callback = remote_command_handler;
    remote_config.status_callback = get_room_status;

    // Define user properties
    mqtt_user_property_t user_properties[] = {
        {"deviceType", SECRET_DEVICE_TYPE},
        {"roomId", SECRET_ROOM_ID},
        {"deviceId", SECRET_DEVICE_ID},
        {"firmwareVersion", FIRMWARE_VERSION}
    };
    int user_property_count = sizeof(user_properties) / sizeof(user_properties[0]);

    // Add the user properties to the remote control configuration
    remote_config.user_properties = user_properties;
    remote_config.user_property_count = user_property_count;

    ret = remote_control_init(&remote_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize remote control: %s", esp_err_to_name(ret));
        return;
    }

    // Start the remote control component
    ret = remote_control_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start remote control: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "✅ MQTT remote control initialized successfully");
    
    // ------------------------------------------------------------------------
    // 6. Start button task
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "⚙️  Starting system tasks...");
    xTaskCreate(button_task, "button_task", BUTTON_TASK_STACK_SIZE, NULL, 10, NULL);
    
    // Show initial occupancy status
    bool is_occupied;
    int count, source_id;
    get_occupancy_status(&is_occupied, &count, &source_id);
    ESP_LOGI(TAG, "Initial occupancy status:");
    print_occupancy_status(is_occupied, count, source_id);
    
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║ Occupancy Detection System Running     ║");
    ESP_LOGI(TAG, "║ Press BOOT button to trigger presence  ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
}

/**
 * @brief MQTT connection event handler
 */
static void mqtt_connect_handler(void *user_data)
{
    ESP_LOGI(TAG, "🔌 MQTT Connected to broker");
    
    // Optionally publish a connection message
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"event\":\"connected\",\"device\":\"%s\",\"timestamp\":%lld}",
             SECRET_MQTT_CLIENT_ID, (long long)(esp_timer_get_time() / 1000));
    
    mqtt_manager_publish(SECRET_MQTT_TOPIC_PREFIX "system", msg, -1, 0, 0, NULL, 0);
}

/**
 * @brief MQTT disconnection event handler
 */
static void mqtt_disconnect_handler(void *user_data)
{
    ESP_LOGW(TAG, "🔌 MQTT Disconnected from broker");
    // Handle disconnection if needed
}

/**
 * @brief Process button press events
 */
static void button_task(void *arg)
{
    uint32_t gpio_num;
    
    while (1) {
        if (xQueueReceive(button_event_queue, &gpio_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "🔘 BOOT BUTTON PRESSED (GPIO %"PRIu32")", gpio_num);
            
            // When button is pressed, set button source count to 1
            update_source_count(SOURCE_BUTTON, 1);
            ESP_LOGI(TAG, "✅ Button source triggered presence (count=1)");
            
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }
}

/**
 * @brief Button interrupt service routine
 */
static void button_isr_handler(void *arg)
{
    uint32_t gpio_num = GPIO_NUM_0;
    xQueueSendFromISR(button_event_queue, &gpio_num, NULL);
}

/**
 * @brief Print radar targets in a human-friendly format
 */
static void print_radar_targets(const ld2450_frame_t *frame)
{
    if (frame->count == 0) {
        return;
    }
    
    ESP_LOGI(TAG, "┌─────────────────────────────────────────────┐");
    ESP_LOGI(TAG, "│             RADAR TARGET DATA               │");
    ESP_LOGI(TAG, "├─────────┬──────────┬──────────┬─────────────┤");
    ESP_LOGI(TAG, "│ Target  │   X (mm) │   Y (mm) │ Speed (cm/s)│");
    ESP_LOGI(TAG, "├─────────┼──────────┼──────────┼─────────────┤");
    
    for (int i = 0; i < frame->count; i++) {
        ESP_LOGI(TAG, "│ %7d │ %8d │ %8d │ %11d │", 
            i + 1, frame->targets[i].x, frame->targets[i].y, frame->targets[i].speed);
    }
    
    ESP_LOGI(TAG, "└─────────┴──────────┴──────────┴─────────────┘");
}

/**
 * @brief Callback for radar target data
 */
static void radar_target_callback(const ld2450_frame_t *frame, void *user_ctx)
{
    // Only log when we have targets
    if (frame->count > 0) {
        print_radar_targets(frame);
    }
}

/**
 * @brief Callback for people counter events
 */
static void people_counter_callback(int count, int entries, int exits, void *context)
{
    // Get previous count
    static int prev_count = 0;
    const char* trend = "";
    
    // Determine trend indicators
    if (count > prev_count) {
        trend = "⬆️ ";
    } else if (count < prev_count) {
        trend = "⬇️ ";
    } else {
        trend = "◼️ ";
    }
    prev_count = count;
    
    ESP_LOGI(TAG, "┌────────────────────────────────────────────────────┐");
    ESP_LOGI(TAG, "│                   PEOPLE COUNTER                   │");
    ESP_LOGI(TAG, "├───────────────┬──────────────────┬─────────────────┤");
    ESP_LOGI(TAG, "│ Current: %s %d │ Total Entry: %2d │ Total Exit: %2d │",
        trend, count, entries, exits);
    ESP_LOGI(TAG, "└───────────────┴──────────────────┴─────────────────┘");
    
    // Update the radar source count with the new value
    update_source_count(SOURCE_RADAR, count);
    ESP_LOGI(TAG, "✅ Radar source updated (count=%d)", count);
}

/**
 * @brief Get reliability level name
 */
static const char* get_reliability_name(int reliability)
{
    switch (reliability) {
        case 1: return "Low";
        case 2: return "Medium";
        case 3: return "High";
        default: return "Unknown";
    }
}

/**
 * @brief Print occupancy status details
 */
static void print_occupancy_status(bool is_occupied, int count, int source_id)
{
    const char* source_name = (source_id >= 0 && source_id < MAX_SOURCES) ? 
                              occupancy_sources[source_id].name : "Unknown";
    int reliability = (source_id >= 0 && source_id < MAX_SOURCES) ? 
                      occupancy_sources[source_id].reliability : 0;
    const char* reliability_name = get_reliability_name(reliability);
    const char* occupancy_icon = is_occupied ? "🟢" : "⚪";
    
    ESP_LOGI(TAG, "╭───────────────────────────────────────────╮");
    ESP_LOGI(TAG, "│           OCCUPANCY STATUS                │");
    ESP_LOGI(TAG, "├───────────────────────────────────────────┤");
    ESP_LOGI(TAG, "│ %s State: %-8s      Count: %d            │", 
        occupancy_icon,
        is_occupied ? "OCCUPIED" : "VACANT",
        count);
    ESP_LOGI(TAG, "│                                           │");
    ESP_LOGI(TAG, "│ Source: %-10s  Reliability: %-8s │", 
        source_name, reliability_name);
    ESP_LOGI(TAG, "╰───────────────────────────────────────────╯");
}

/**
 * @brief Determine RGB LED color based on occupancy source and count
 */
static void determine_led_color(int source_id, int count, int *r, int *g, int *b)
{
    // Default color (warm white)
    *r = 255;
    *g = 220;
    *b = 180;
    
    // Adjust based on source
    if (source_id == SOURCE_RADAR) {
        // Radar: blue-green gradient based on count (1-3)
        *r = 0;
        *g = 150 + (count > 0 ? (count <= 3 ? count * 35 : 105) : 0);
        *b = 200 + (count > 0 ? (count <= 3 ? count * 18 : 55) : 0);
    } 
    else if (source_id == SOURCE_REMOTE) {
        // Remote: purple gradient based on count (1-3)
        *r = 180 + (count > 0 ? (count <= 3 ? count * 25 : 75) : 0);
        *g = 0;
        *b = 220 + (count > 0 ? (count <= 3 ? count * 11 : 35) : 0);
    } 
    else if (source_id == SOURCE_BUTTON) {
        // Button: fixed amber color
        *r = 255;
        *g = 170;
        *b = 0;
    }
    
    // Ensure values are in valid range
    *r = (*r > 255) ? 255 : (*r < 0 ? 0 : *r);
    *g = (*g > 255) ? 255 : (*g < 0 ? 0 : *g);
    *b = (*b > 255) ? 255 : (*b < 0 ? 0 : *b);
}

/**
 * @brief Send LED control message via MQTT
 */
static void send_led_control_message(bool power_on, int red, int green, int blue)
{
    // Skip if MQTT manager is not connected
    if (!mqtt_manager_is_connected()) {
        ESP_LOGW(TAG, "Cannot send LED control - MQTT not connected");
        return;
    }

    // Create message payload
    char payload[128];
    snprintf(payload, sizeof(payload), 
             "{\"power\":\"%s\",\"color\":{\"r\":%d,\"g\":%d,\"b\":%d}}",
             power_on ? "on" : "off", red, green, blue);
    
    // Topic to control the LED device
    char topic[128];
    strcpy(topic, SECRET_MQTT_TOPIC_CONTROL);
    
    ESP_LOGI(TAG, "🌈 Sending LED control message: %s", payload);
    
    // Publish the message with QoS 1 to ensure delivery
    int msg_id = mqtt_manager_publish(topic, payload, -1, 1, 0, NULL, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "❌ Failed to publish LED control message: %d", msg_id);
    } else {
        ESP_LOGI(TAG, "✅ LED control message sent successfully (msg_id=%d)", msg_id);
    }
}

/**
 * @brief Update status reporting and LED control
 */
static void update_status_and_led(void)
{
    bool is_occupied;
    int count, source_id;
    
    // Get current occupancy status (highest count source)
    get_occupancy_status(&is_occupied, &count, &source_id);
    
    ESP_LOGI(TAG, "⚠️ Occupancy data changed:");
    print_occupancy_status(is_occupied, count, source_id);
    
    // Report status change via MQTT
    room_status_t room_status = {
        .state = is_occupied ? ROOM_STATE_OCCUPIED : ROOM_STATE_VACANT,
        .occupant_count = count,
        .count_reliable = true, // We're using highest count, so it's reliable by design
        .source_name = (source_id >= 0) ? occupancy_sources[source_id].name : "unknown",
        .source_reliability = (source_id >= 0) ? occupancy_sources[source_id].reliability : 0
    };
    
    remote_control_publish_status(&room_status);
    
    // Control RGB LED based on occupancy
    int r = 0, g = 0, b = 0;
    
    if (is_occupied) {
        // Room is occupied - set LED color based on source and count
        determine_led_color(source_id, count, &r, &g, &b);
        send_led_control_message(true, r, g, b);
        
        ESP_LOGI(TAG, "💡 LED updated (R:%d, G:%d, B:%d) based on %s source with count %d", 
                r, g, b, occupancy_sources[source_id].name, count);
    } else {
        // Room is vacant - turn LED off
        send_led_control_message(false, 0, 0, 0);
        ESP_LOGI(TAG, "💡 LED turned OFF - room is vacant");
    }
}

/**
 * @brief Handle commands received from remote control
 */
static void remote_command_handler(remote_command_t command, const char *payload, void *user_ctx)
{
    ESP_LOGI(TAG, "Received remote command: %d", command);
    
    switch (command) {
        case REMOTE_CMD_SET_OCCUPIED:
            // When remote says occupied, set remote source count to 1
            ESP_LOGI(TAG, "Remote command: Set occupied");
            update_source_count(SOURCE_REMOTE, 1);
            break;
            
        case REMOTE_CMD_SET_VACANT:
            // When remote says vacant, set remote source count to 0
            ESP_LOGI(TAG, "Remote command: Set vacant");
            update_source_count(SOURCE_REMOTE, 0);
            break;
            
        case REMOTE_CMD_REQUEST_STATUS:
            // Status request is handled internally by remote_control component
            ESP_LOGI(TAG, "Remote command: Request status");
            break;
            
        case REMOTE_CMD_RESTART_DEVICE:
            ESP_LOGI(TAG, "Remote command: Restart device");
            esp_restart();
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown or unhandled remote command: %d", command);
            break;
    }
}

/**
 * @brief Get current room status for remote reporting
 */
static room_status_t get_room_status(void *user_ctx)
{
    bool is_occupied;
    int count, source_id;
    
    // Get current occupancy status (highest count source)
    get_occupancy_status(&is_occupied, &count, &source_id);
    
    // Convert to room_status_t
    room_status_t room_status = {
        .state = is_occupied ? ROOM_STATE_OCCUPIED : ROOM_STATE_VACANT,
        .occupant_count = count,
        .count_reliable = true, // Using highest count is reliable by design
        .source_name = (source_id >= 0) ? occupancy_sources[source_id].name : "unknown",
        .source_reliability = (source_id >= 0) ? occupancy_sources[source_id].reliability : 0
    };
    
    return room_status;
}