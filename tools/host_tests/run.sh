#!/bin/sh
# Pruebas en el PC (Linux / WSL / MSYS2 con gcc). Ejecutar desde la raíz del proyecto:  sh tools/host_tests/run.sh
# Termina con código distinto de 0 si cualquier prueba falla.
set -e
mkdir -p /tmp/scara_host
OUT=/tmp/scara_host/salida.txt

# probar FILTRO comando...: corre, muestra las líneas que coinciden con FILTRO y aborta si el programa falló.
probar() {
    filtro="$1"; shift
    rc=0
    "$@" > "$OUT" 2>&1 || rc=$?
    grep -E "$filtro" "$OUT" || true
    if [ "$rc" -ne 0 ]; then echo "*** FALLO: $*"; exit 1; fi
}

gcc -Wall -Wextra -O2 -Isrc tools/host_tests/test_kin_traj.c src/kinematics.c -lm -o /tmp/scara_host/t_kin
probar '.' /tmp/scara_host/t_kin
# la simulación de todo el firmware necesita las cabeceras "falsas" de ESP-IDF de tools/host_tests/stubs
gcc -std=gnu11 -O2 -Wall -Wextra -Itools/host_tests/stubs -Isrc -Itools/host_tests tools/host_tests/test_control.c \
    src/config.c src/kinematics.c src/motion.c src/screw.c src/uros_cmd.c -lm -o /tmp/scara_host/t_ctl
probar '^[^ ]|^  (>>>|\(tiempo)' /tmp/scara_host/t_ctl
gcc -std=gnu11 -O1 -Wall -Wextra -Itools/host_tests/stubs -Isrc tools/host_tests/test_comms.c src/config.c src/kinematics.c src/motion.c src/uros_cmd.c src/screw.c -lm -o /tmp/scara_host/t_comms
probar '\[ok\]|FALLO|TODO OK|FALLOS' /tmp/scara_host/t_comms
# intérprete de comandos de ROS 2 + módulo compartido de movimiento
gcc -std=gnu11 -O1 -Wall -Wextra -Werror=format -Itools/host_tests/stubs -Isrc tools/host_tests/test_uros_cmd.c src/uros_cmd.c src/motion.c src/config.c src/kinematics.c src/screw.c -lm -o /tmp/scara_host/t_uros
probar '\[ok\]|FALLO|TODO OK|FALLOS' /tmp/scara_host/t_uros
# la capa micro-ROS no se puede ejecutar en el PC; se comprueba que COMPILA contra cabeceras falsas con las firmas de rclc/rmw (jazzy)
gcc -fsyntax-only -std=gnu11 -Wall -Wextra -DFAKE_SERIAL -Itools/host_tests/stubs_uros -Itools/host_tests/stubs -Isrc src/uros_comms.c src/uros_serial_transport.c && echo "  [ok] uros_comms.c + transporte serial compilan (sintaxis)"
gcc -fsyntax-only -std=gnu11 -Wall -Wextra -Itools/host_tests/stubs_uros -Itools/host_tests/stubs -Isrc src/uros_comms.c && echo "  [ok] uros_comms.c compila con transporte UDP/WiFi (sintaxis)"
# secuencia, enlace y panel (sin ROS: el panel corre sin pantalla con un enlace falso)
probar 'OK|FAILED|^Ran' python3 -m unittest tools/host_tests/test_sequence.py tools/host_tests/test_link.py
probar 'OK|FAILED|^Ran' python3 -m unittest tools/host_tests/test_panel.py
# simulador de extremo a extremo, si hay ROS 2 cargado
if [ -n "$ROS_DISTRO" ]; then probar 'OK|FAILED|^Ran' python3 tools/sim/test_e2e.py; else echo "  (sin ROS 2 cargado: se omite tools/sim/test_e2e.py)"; fi
echo "=== TODAS LAS PRUEBAS PASARON ==="
