# components/remote_control/CMakeLists.txt

idf_component_register(
    SRCS 
        "src/remote_control.c"
        "src/remote_memory_debug.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES
        mqtt_manager
)