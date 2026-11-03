set(srcs main.c hal.c control.c screw.c kinematics.c config.c motion.c uros_cmd.c)

# Este archivo se evalúa DOS veces: primero ESP-IDF lo lee para averiguar dependencias, cuando todavía
# NO existe el sdkconfig (CONFIG_* vacío); después, ya con la configuración. Por eso el modo se decide por
# algo que existe en ambas fases: el archivo app-colcon.meta que crea tools/uros_select.py
# (existe en serial/wifi y se borra en text). Las comprobaciones con CONFIG_* solo van en la 2.ª fase.
get_filename_component(SCARA_PROJ "${CMAKE_CURRENT_LIST_DIR}" DIRECTORY)
set(SCARA_UROS_MODE OFF)
if(EXISTS "${SCARA_PROJ}/app-colcon.meta")
    set(SCARA_UROS_MODE ON)
endif()

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    # Protecciones contra una configuración a medias (si no, compilaría en silencio en otro modo):
    if(SCARA_UROS_MODE AND NOT CONFIG_SCARA_COMMS_UROS)
        message(FATAL_ERROR
            "Existe app-colcon.meta (modo micro-ROS) pero CONFIG_SCARA_COMMS_UROS no esta activo. "
            "Ejecuta 'python tools/uros_select.py serial' (o 'text' para borrarlo) y luego 'pio run -t fullclean'. "
            "Si ya lo hiciste, PlatformIO no esta leyendo src/Kconfig.projbuild: ver UBUNTU_MICROROS.md, seccion 8.")
    endif()
    if(NOT SCARA_UROS_MODE AND CONFIG_SCARA_COMMS_UROS)
        message(FATAL_ERROR
            "CONFIG_SCARA_COMMS_UROS esta activo pero falta app-colcon.meta. Ejecuta 'python tools/uros_select.py serial' "
            "y luego 'pio run -t fullclean'.")
    endif()
endif()

if(SCARA_UROS_MODE)
    if(NOT EXISTS "${SCARA_PROJ}/components/micro_ros_espidf_component")
        message(FATAL_ERROR
            "Falta components/micro_ros_espidf_component. Ejecuta: "
            "git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component")
    endif()
    # ROS 2 / micro-ROS (ver tools/uros_select.py y UBUNTU_MICROROS.md)
    list(APPEND srcs uros_comms.c uros_serial_transport.c)
    set(extra_requires micro_ros_espidf_component)
else()
    # protocolo de texto por serial
    list(APPEND srcs comms.c)
    set(extra_requires "")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer esp_system ${extra_requires})
