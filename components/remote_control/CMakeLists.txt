# components/remote_control/CMakeLists.txt

idf_component_register(
    SRCS
        "src/remote_control.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES
        log
        freertos
        esp_event
        esp_timer
        mqtt_manager
)