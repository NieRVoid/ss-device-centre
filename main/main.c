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

// Pre-allocated topic buffers to reduce heap fragmentation
#define MAX_TOPIC_LEN 128
static char lwt_topic_buf[MAX_TOPIC_LEN];
static char led_control_topic_buf[MAX_TOPIC_LEN];

// Pre-allocated user properties to avoid dynamic allocation
static mqtt_user_property_t user_properties[4] = {
    {"deviceType", SECRET_DEVICE_TYPE},
    {"roomId", SECRET_ROOM_ID},
    {"deviceId", SECRET_DEVICE_ID},
    {"firmwareVersion", FIRMWARE_VERSION}
};
static const int user_property_count = sizeof(user_properties) / sizeof(user_properties[0]);

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

// Stack sizes and initialization flags
#define BUTTON_TASK_STACK_SIZE 4096
static bool mqtt_initialized = false;
static bool remote_control_initialized = false;

static inline esp_err_t require_component_init(const char *name, bool condition) {
    if (!condition) {
        ESP_LOGE(TAG, "%s component must be initialized first", name);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/**
 * @brief Initialize networking stack and connect to WiFi
 */
static esp_err_t init_networking(void)
{
    ESP_LOGI(TAG, "Initializing networking stack");
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = example_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Connected to WiFi network");
    return ESP_OK;
}

/**
 * @brief Initialize MQTT manager with optimized settings
 */
static esp_err_t init_mqtt_manager(void)
{
    ESP_LOGI(TAG, "⚙️  Initializing MQTT manager...");
    
    // Configure and format LWT topic using pre-allocated buffer
    snprintf(lwt_topic_buf, sizeof(lwt_topic_buf), "%s%s", SECRET_MQTT_TOPIC_PREFIX, "status");
    
    mqtt_manager_config_t mqtt_config = MQTT_MANAGER_DEFAULT_CONFIG();
    mqtt_config.broker_uri = SECRET_MQTT_BROKER_URI;
    mqtt_config.client_id = SECRET_MQTT_CLIENT_ID;
    mqtt_config.username = SECRET_MQTT_USERNAME;
    mqtt_config.password = SECRET_MQTT_PASSWORD;
    mqtt_config.use_ssl = false;
    mqtt_config.keepalive = 120;
    mqtt_config.connect_handler = mqtt_connect_handler;
    mqtt_config.disconnect_handler = mqtt_disconnect_handler;
    
    // Configure LWT using pre-allocated buffer
    mqtt_config.lwt_topic = lwt_topic_buf;
    mqtt_config.lwt_msg = "{\"state\":\"disconnected\"}";
    mqtt_config.lwt_qos = 1;
    mqtt_config.lwt_retain = 1;
    
    esp_err_t ret = mqtt_manager_init(&mqtt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize MQTT manager: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ MQTT manager initialized successfully");
    
    // Start MQTT manager
    ret = mqtt_manager_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start MQTT manager: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ MQTT manager started successfully");
    
    // Set initialization flag
    mqtt_initialized = true;
    
    // Format LED control topic
    snprintf(led_control_topic_buf, sizeof(led_control_topic_buf), "%s", SECRET_MQTT_TOPIC_CONTROL);
    
    return ESP_OK;
}

/**
 * @brief Initialize remote control component
 */
static esp_err_t init_remote_control(void)
{
    // Ensure MQTT manager is initialized before remote control
    require_component_init("MQTT Manager", mqtt_manager_is_initialized());
    
    ESP_LOGI(TAG, "⚙️  Initializing MQTT remote control...");
    
    remote_control_config_t remote_config = REMOTE_CONTROL_DEFAULT_CONFIG();
    remote_config.topic_prefix = SECRET_MQTT_TOPIC_PREFIX;
    remote_config.status_interval_sec = 300; // 5 minute updates
    remote_config.command_callback = remote_command_handler;
    remote_config.status_callback = get_room_status;

    // Use pre-allocated user properties
    remote_config.user_properties = user_properties;
    remote_config.user_property_count = user_property_count;

    esp_err_t ret = remote_control_init(&remote_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize remote control: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start the remote control component
    ret = remote_control_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start remote control: %s", esp_err_to_name(ret));
        remote_control_deinit();
        return ret;
    }

    ESP_LOGI(TAG, "✅ MQTT remote control initialized and started successfully");
    remote_control_initialized = true;
    return ESP_OK;
}

/**
 * @brief Wait for MQTT Manager to connect with timeout
 */
static esp_err_t wait_for_mqtt_connection(int timeout_ms)
{
    int wait_time_ms = 0;
    const int check_interval_ms = 100;
    
    while (!mqtt_manager_is_connected() && wait_time_ms < timeout_ms) {
        ESP_LOGI(TAG, "Waiting for MQTT Manager to connect... (%d/%d ms)", 
                wait_time_ms, timeout_ms);
        vTaskDelay(pdMS_TO_TICKS(check_interval_ms));
        wait_time_ms += check_interval_ms;
    }

    if (!mqtt_manager_is_connected()) {
        ESP_LOGE(TAG, "MQTT Manager failed to connect within %d ms timeout", timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "MQTT Manager connected successfully");
    return ESP_OK;
}

/**
 * @brief Initialize button for occupancy detection
 */
static esp_err_t init_button(void)
{
    ESP_LOGI(TAG, "⚙️  Configuring BOOT button on GPIO0...");
    
    // Create queue for button events
    button_event_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!button_event_queue) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return ESP_ERR_NO_MEM;
    }
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .pull_up_en = 1,
        .pull_down_en = 0
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // OK if already installed
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_NUM_0, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ BOOT button configured on GPIO0");
    return ESP_OK;
}

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

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║ Starting Occupancy Detection System    ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    
    // Initialize components with proper error handling
    esp_err_t ret;
    
    // 1. Initialize networking
    ret = init_networking();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize networking, stopping initialization");
        return;
    }
    
    // 2. Initialize LD2450 radar driver
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    radar_config.uart_rx_pin = 16;
    radar_config.uart_tx_pin = 17;
    radar_config.uart_baud_rate = 256000;
    radar_config.log_level = LD2450_LOG_VERBOSE;
    radar_config.data_log_interval_ms = 3000;
    
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
    
    // 3. Initialize people counter
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
    
    // 4. Initialize BOOT button
    ret = init_button();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize button, but continuing...");
        // Continue anyway as this is not critical
    }
    
    // 5. Initialize MQTT manager
    ret = init_mqtt_manager();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT manager, stopping initialization");
        return;
    }
    
    // Wait for MQTT to connect
    ret = wait_for_mqtt_connection(10000); // 10 second timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MQTT connection timeout, stopping initialization");
        return;
    }
    
    // 6. Initialize Remote Control
    ret = init_remote_control();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize remote control, stopping initialization");
        return;
    }
    
    // 7. Start button task
    ESP_LOGI(TAG, "⚙️  Starting system tasks...");
    xTaskCreate(button_task, "button_task", BUTTON_TASK_STACK_SIZE, NULL, 10, NULL);
    
    // Show initial occupancy status
    bool is_occupied;
    int count, source_id;
    get_occupancy_status(&is_occupied, &count, &source_id);
    ESP_LOGI(TAG, "Initial occupancy status:");
    print_occupancy_status(is_occupied, count, source_id);
    
    // 8. Print memory usage statistics
    #ifdef CONFIG_MQTT_MANAGER_DEBUG
    mqtt_manager_print_memory_usage();
    #endif
    
    #ifdef CONFIG_REMOTE_CONTROL_DEBUG
    remote_control_print_memory_usage();
    #endif
    
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
 * @brief Callback for radar target data
 */
static void radar_target_callback(const ld2450_frame_t *frame, void *user_ctx)
{
    // Empty callback - detailed logging now handled by LD2450 driver
    // We're keeping the function as it's still used in the ld2450_register_target_callback call
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

    // Limit count for color gradient (max 5)
    int adjusted_count = (count > 5) ? 5 : (count < 0) ? 0 : count;

    if (source_id == SOURCE_RADAR) {
        // Radar: blue color scheme
        *r = 0;
        *g = 0;
        if (adjusted_count == 0) {
            *b = 0; // No people
        } else {
            // Linear gradient: count 1-5 maps from 100 to 255
            *b = 100 + ((adjusted_count - 1) * (155 / 4)); // Values: 100,138,177,216,255
        }
    } 
    else if (source_id == SOURCE_REMOTE) {
        // Remote: green color scheme
        *r = 0;
        *b = 0;
        if (adjusted_count == 0) {
            *g = 0; // No people
        } else {
            // Linear gradient: count 1-5 maps from 100 to 255
            *g = 100 + ((adjusted_count - 1) * (155 / 4)); // Values: 100,138,177,216,255
        }
    }
    else if (source_id == SOURCE_BUTTON) {
        // Button: fixed yellow color (single state as it can only detect presence)
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

    // Pre-allocated buffer for message payload to avoid heap allocation
    char payload[128];
    snprintf(payload, sizeof(payload), 
             "{\"power\":\"%s\",\"color\":{\"r\":%d,\"g\":%d,\"b\":%d}}",
             power_on ? "on" : "off", red, green, blue);
    
    ESP_LOGI(TAG, "🌈 Sending LED control message: %s", payload);
    
    // Publish the message with QoS 1 to ensure delivery
    int msg_id = mqtt_manager_publish(led_control_topic_buf, payload, -1, 1, 0, user_properties, user_property_count);
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
    
    // Use stack allocation for the room status
    room_status_t room_status = {
        .state = is_occupied ? ROOM_STATE_OCCUPIED : ROOM_STATE_VACANT,
        .occupant_count = count,
        .count_reliable = true, // Using highest count is reliable by design
        .source_name = (source_id >= 0) ? occupancy_sources[source_id].name : "unknown",
        .source_reliability = (source_id >= 0) ? occupancy_sources[source_id].reliability : 0
    };
    
    return room_status;
}