#!/usr/bin/env python3
"""
Selecciona el modo de comunicación del firmware y deja la configuración COHERENTE.

    python tools/uros_select.py text                       protocolo de texto (monitor de PlatformIO / teleop.py)
    python tools/uros_select.py serial                     micro-ROS por el cable USB (UART0)  <- primera prueba
    python tools/uros_select.py wifi --ssid RED --password CLAVE --agent-ip 192.168.1.50 [--agent-port 8888]

Escribe en la raíz del proyecto:
    sdkconfig.defaults     ajustes de ESP-IDF del modo elegido
    app-colcon.meta        (solo micro-ROS) transporte y límites de entidades de la librería micro-ROS
y borra los sdkconfig generados (sdkconfig, sdkconfig.<env>) para que se vuelvan a crear.

DESPUÉS de cambiar de modo:   pio run -t fullclean   y   pio run -t upload
(la primera compilación con micro-ROS tarda varios minutos: descarga y compila la librería).
"""
import argparse
import glob
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

BASE = """\
CONFIG_IDF_TARGET="esp32"
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200
CONFIG_ESP_BROWNOUT_DET=y
CONFIG_FREERTOS_HZ=1000
# menos ruido en el puerto serie (el protocolo usa el mismo puerto)
CONFIG_LOG_DEFAULT_LEVEL_WARN=y
# watchdog de tareas: si el lazo de control se cuelga, se reinicia (y STBY cae a 0 por R10)
CONFIG_ESP_TASK_WDT_EN=y
CONFIG_ESP_TASK_WDT_INIT=y
CONFIG_ESP_TASK_WDT_PANIC=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=2
"""

# Límites de entidades de micro-ROS. La base (colcon.meta del componente) permite 2 publicadores y
# 2 suscriptores; este nodo usa 3 publicadores y 3 suscriptores.
COLCON_COMMON = """\
        "-DRMW_UXRCE_MAX_NODES=1",
        "-DRMW_UXRCE_MAX_PUBLISHERS=4",
        "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=4",
        "-DRMW_UXRCE_MAX_SERVICES=1",
        "-DRMW_UXRCE_MAX_CLIENTS=1",
        "-DRMW_UXRCE_MAX_HISTORY=4",
        "-DRMW_UXRCE_XML_BUFFER_LENGTH=400\""""


def colcon_meta(transport):
    return ('{\n  "names": {\n    "rmw_microxrcedds": {\n      "cmake-args": [\n        "-DRMW_UXRCE_TRANSPORT=%s",\n%s\n      ]\n    }\n  }\n}\n'
            % (transport, COLCON_COMMON))


def base_without(prefixes):
    return "".join(l + "\n" for l in BASE.splitlines() if not any(l.startswith(p) for p in prefixes))


def write(name, text):
    with open(os.path.join(ROOT, name), "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("  escrito", name)


def main():
    ap = argparse.ArgumentParser(description="Selecciona texto / micro-ROS serial / micro-ROS WiFi")
    ap.add_argument("modo", choices=["text", "serial", "wifi"])
    ap.add_argument("--ssid")
    ap.add_argument("--password")
    ap.add_argument("--agent-ip")
    ap.add_argument("--agent-port", default="8888")
    ap.add_argument("--baud", type=int, default=460800, help="solo serial; debe coincidir con '-b' del agente")
    a = ap.parse_args()

    if a.modo == "wifi" and not (a.ssid and a.password is not None and a.agent_ip):
        ap.error("wifi necesita --ssid, --password y --agent-ip (la IP del PC con Ubuntu)")

    if a.modo == "text":
        cfg = BASE + "CONFIG_SCARA_COMMS_UROS=n\n"
        meta = None
    elif a.modo == "serial":
        cfg = base_without(["CONFIG_ESP_CONSOLE_UART", "CONFIG_LOG_DEFAULT_LEVEL"]) + (
            "# --- micro-ROS por serial: UART0 es del transporte, sin consola ni logs ---\n"
            "CONFIG_SCARA_COMMS_UROS=y\n"
            "CONFIG_ESP_CONSOLE_NONE=y\n"
            "CONFIG_LOG_DEFAULT_LEVEL_NONE=y\n"
            "CONFIG_BOOTLOADER_LOG_LEVEL_NONE=y\n"
            "CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE=y\n"
            "CONFIG_MICRO_ROS_ESP_UART_TRANSPORT=y\n"
            "CONFIG_SCARA_UROS_BAUD=%d\n" % a.baud)
        meta = colcon_meta("custom")
    else:
        cfg = BASE + (
            "# --- micro-ROS por WiFi (UDP); la consola por USB sigue disponible para depurar ---\n"
            "CONFIG_SCARA_COMMS_UROS=y\n"
            "CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE=y\n"
            "CONFIG_MICRO_ROS_ESP_NETIF_WLAN=y\n"
            'CONFIG_ESP_WIFI_SSID="%s"\n'
            'CONFIG_ESP_WIFI_PASSWORD="%s"\n'
            'CONFIG_MICRO_ROS_AGENT_IP="%s"\n'
            'CONFIG_MICRO_ROS_AGENT_PORT="%s"\n' % (a.ssid, a.password, a.agent_ip, a.agent_port))
        meta = colcon_meta("udp")

    print("Modo:", a.modo)
    write("sdkconfig.defaults", cfg)
    meta_path = os.path.join(ROOT, "app-colcon.meta")
    if meta is None:
        if os.path.exists(meta_path):
            os.remove(meta_path)
            print("  borrado app-colcon.meta")
    else:
        write("app-colcon.meta", meta)

    for p in glob.glob(os.path.join(ROOT, "sdkconfig*")):
        if not p.endswith(".defaults"):
            os.remove(p)
            print("  borrado", os.path.basename(p))

    print("\nSiguiente:  pio run -t fullclean   y luego   pio run -t upload")
    if a.modo != "text":
        print("IMPORTANTE: compila en una terminal SIN 'source /opt/ros/jazzy/setup.bash' (ver UBUNTU_MICROROS.md).")
    if a.modo == "wifi":
        print("Nota: la clave WiFi queda escrita en sdkconfig.defaults; no lo subas a un repositorio publico.")


if __name__ == "__main__":
    main()
