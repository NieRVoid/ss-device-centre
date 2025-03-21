# components/mqtt_manager/CMakeLists.txt

idf_component_register(
    SRCS
        "src/mqtt_manager.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES
        log
        freertos
        esp_event
        esp_system
        esp_timer
        mqtt
)