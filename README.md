# SCARA, firmware de emergencia sin encoders

Para usar si la parte de encoders de la PCB no sirve. Hace lo mismo que `scara_fw` con el mismo panel, RViz, micro-ROS, comandos y cableado. La diferencia es que cada eje se enciende un tiempo calculado y se frena: los ángulos son **estimados**.

**Nada de esto se ha probado en el robot real.** Todo está verificado contra un modelo de los motores, y los tiempos y números de abajo salen de ahí.

## Qué se necesita sí o sí

- Los 4 finales de carrera funcionando (J9 a J12). Sin encoders son la única medición que hay.
- M3 arriba antes de Home y antes de `ref`, porque esos dos recorren los ejes de tope a tope.

## Qué es igual y qué cambia

Igual: panel, gemelo en RViz, tópicos de ROS 2, hombre muerto, paro de emergencia, activar y desactivar motores, ir a x,y, ir a ángulos, manual, servo, M3, subida de M3 tras cada tornillo, secuencia de 16 huecos, opción Ensayo y simulador.

| | Firmware principal | Este firmware |
|---|---|---|
| Control | PID o RST | Por tiempo. Pestaña «Controlador» pasa a «Calibración» |
| Home | Busca topes y mide con encoder | Mide tiempos de tope a tope, unos 74 s, y vuelve cerca de donde estaba |
| Quieto (HOLD) | Mantiene la posición | Motores frenados |
| Eje trabado | Falla de inmediato | Solo se nota al pisar un final de carrera o en el siguiente `ref` |
| Desactivar un eje o `off` | Conserva el Home | Pierde el Home, porque nadie mediría si lo mueven |
| Tornillo | Mide vueltas, estima corriente y presión | PWM y tiempo. El resultado siempre es `sin_medir`: **revísalos a mano** |
| Secuencia | Sin correcciones | `ref` al inicio y cada 4 huecos, unos 16 s cada uno |

No se copiaron `teleop.py`, `characterize.py`, `scara_teleop_ros.py`, ni los reportes de RST y M3. Los comandos `gain`, `ctrl`, `rst` y `rstform` responden con un error que lo explica.

## Cargar y abrir

```sh
sudo apt install python3-pyqt5 python3-pyqtgraph python3-yaml python3-numpy   # una vez, para el panel
source /opt/ros/jazzy/setup.bash
pio run -t upload                                                   # ya queda en modo micro-ROS
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 460800 -v6
python3 ros2/scara_panel.py                                         # o --sim para probar sin robot
```

## Medir el robot la primera vez

Home mide tiempos, no grados. Estos valores son los que los convierten, y hoy tienen los del simulador:

1. **Finales de carrera.** En la pestaña Motores, pisa cada uno con la mano. Lo correcto, por los topes cruzados de `src/config.c`, es: J9 → J1 «tope MAX», J10 → J1 «tope MIN», J11 → J2 «tope MAX», J12 → J2 «tope MIN». Si ves esto, no recablees.
2. **Sentidos.**
   - **J1 y J2:** con el brazo lejos de los topes, envía `pulse 1` y `pulse 2`. `#PULSO` siempre dice positivo, así que mira el brazo: q aumenta girando en sentido antihorario visto desde arriba. Si va al revés, cambia `motor_sign` en `src/config.c`.
   - **M3:** envía `motor 3 on`, y con `tool 30` y `tool 0` comprueba que apretar gira hacia abajo. Si no, cambia `TOOL_MOTOR_SIGN` en `src/config.h`.
   - **Servo:** prueba `servo abrir` y `servo cerrar`, y ajusta sus ángulos en `src/config.h` si hace falta.
3. **Home y `ref`.** Sube M3 con `tool -30` y luego `tool 0`, pulsa Home y luego «Recorrer ejes». q1 = 0 es el eslabón 1 sobre +x, y q2 = 0 es el eslabón 2 alineado con el 1. Mide con un transportador el ángulo de cada eje y escríbelo en `q_home` de `src/config.c`.
4. **Rango.** Con `off`, lleva cada eje a mano hasta que suene su final de carrera MAX. `rango` es ese ángulo menos `q_home`. Carga el firmware de nuevo.
5. **Velocidad.** Home, «Recorrer ejes», pide `ang 60 40` y mide dónde quedó. Escribe esos valores en la pestaña Calibración y pulsa Ajustar. El panel muestra el valor para `J1_AJUSTE` y `J2_AJUSTE` en `src/config.h`: cópialo o se pierde al reiniciar.

Si cambias los límites, actualiza también `Q1_LIM_DEG` y `Q2_LIM_DEG` en `ros2/scara_sequence.py`.

**Después del paro físico o de un corte de 12 V, usa `off` y repite Home.** El firmware no ve ese corte: puede decir `#LLEGO` con el brazo en otro sitio.

## Precisión esperada

Error del brazo al llegar a cada hueco, según el simulador. Se calibra con el robot nominal y luego cambian las condiciones. Regenerar con `python3 tools/sim/precision.py`.

| Condición | Mediana mm | Peor mm | Corridas abortadas |
|---|---|---|---|
| Nominal | 0,93 | 1,91 | 0 de 1 |
| Juego del reductor 0,5 grados | 1,36 | 2,60 | 0 de 1 |
| Rango mal medido 1 grado | 0,90 | 1,31 | 0 de 1 |
| Voltaje 3 % menor | 1,86 | 2,86 | 0 de 1 |
| Fricción 10 % mayor | 3,26 | 5,48 | 0 de 1 |
| Fricción 10 % mayor desde el hueco 7 | 1,75 | 4,62 | 0 de 1 |
| Fricción varía 1 % en cada arranque | 1,92 | 7,90 | 0 de 20 |
| Fricción varía 2 % en cada arranque | 3,38 | 15,04 | 2 de 20 |
| Todo junto | 5,63 | 19,42 | 3 de 20 |

Corregir en cada hueco, con fricción que varía 2 %, baja la mediana de 3,38 a 2,20 mm.

No alcanza la holgura de 0,1 mm del hueco, así que la guía mecánica tiene que absorber el resto. Si `ref` falla con «se perdió», usa `clear` y repite Home. El primer `ref` tras Home mide la distancia fija hasta el final de carrera y admite hasta `DESVIO_PRIMER_REF_MAX_DEG`, así que no detecta un empujón menor que eso. Los siguientes admiten `DESVIO_MAX_DEG`. Ambos están en `src/config.h`. Con fricción variable, esa alarma puede saltar sin que el brazo esté tan perdido: en el simulador reportó 8,1 grados con 1 grado de error real.

En el simulador, el proceso completo tarda unos 14 minutos y el ensayo unos 4.

## Sin confirmar

- **M3 sube girando al revés.** Con el mismo PWM y las mismas vueltas. Se supone que la punta se suelta sola del tornillo, así que la subida no lo afloja, y el simulador lo modela así. No está medido: compruébalo con el primer tornillo. Si no se suelta, según el simulador M3 queda calado 18,5 s sin subir y el siguiente `ref` arrastraría la punta: cambia esa subida en `ros2/secuencia.yaml`.
- **La bajada y la subida no son iguales si M3 no se frena.** Aprieta 23 s, unas 15 vueltas si gira libre, y sube 18,5 s, 12 vueltas. Si M3 baja sin que el tornillo lo frene, por ejemplo porque no cayó, quedan 1,5 mm de más en cada hueco.
- **La señal del servo** pasa por R40 de 10 kΩ en serie. Con el servo conectado, mide en J13 pin 2 que llegue al nivel alto; si no, puentea R40.
- **El torque de 0,4 N·m** sale de la hoja de datos del motor, no de una medición.
- **El modo texto** (`tools/uros_select.py text`) se compiló en una copia temporal, pero no se ha ejecutado en la placa.

## Pruebas

```sh
sh tools/host_tests/run.sh      # firmware, panel y secuencia. Con ROS 2 cargado también corre tools/sim/test_e2e.py
```

`components/micro_ros_espidf_component` pesa 497 MB y no está en git. Si clonas este repositorio, bájalo una vez antes de compilar:

```sh
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component
git -C components/micro_ros_espidf_component checkout 5cd71f525eedc1e59a48e3a12184ba75b60b05b7
sed -i '/use_gitignore: true/d' components/micro_ros_espidf_component/idf_component.yml
```

La primera compilación construye micro-ROS y falla si ROS 2 está cargado. Hazla en una terminal nueva, que luego no uses para el panel:

```sh
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
export PATH=$(echo "$PATH" | tr ':' '
' | grep -v '/opt/ros' | paste -sd:)
pio run
```
