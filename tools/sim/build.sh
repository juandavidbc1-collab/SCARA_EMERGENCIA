#!/bin/sh
# Compila el firmware con la planta simulada:  sh tools/sim/build.sh  ->  tools/sim/build/libscara_sim.so
set -e
cd "$(dirname "$0")/../.."
mkdir -p tools/sim/build
# el simulador se comporta como un robot ya calibrado con "ajuste": los valores salen de tools/sim/precision.py
gcc -DJ1_AJUSTE=1.0107f -DJ2_AJUSTE=1.0060f -std=gnu11 -O2 -w -fPIC -shared -DCONFIG_SCARA_COMMS_UROS=1 -Itools/host_tests/stubs -Isrc \
    tools/sim/scara_sim.c src/config.c src/kinematics.c src/motion.c src/screw.c src/uros_cmd.c \
    -lm -o tools/sim/build/libscara_sim.so
