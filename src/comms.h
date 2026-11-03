/*
 * COMUNICACION POR SERIAL (UART0 / USB de la DevKit, 115200 8N1).
 *
 * MODO TEXTO (el de arranque, para el monitor serie de PlatformIO o cualquier terminal):
 *   - Lo que escribes se ve en pantalla (eco) y se borra con Backspace.
 *   - Menu numerico de un toque (1..9, 0) y comandos escritos (help). Respuestas en lenguaje claro.
 *   - Esc o CUALQUIER TECLA detiene los monitores en vivo (2, 3, tele, sens) y los movimientos
 *     (Home, goto, ang, pulso). En una entrada con argumentos, Esc o Enter vacio cancela.
 *
 * MODO MAQUINA (lo activa tools/teleop.py con "fmt 1"; vuelve solo al modo texto tras 3 s sin trafico):
 *   - Sin eco ni atajos. Lineas de texto, una orden por linea.
 *   PC -> ESP32:  d <dm> <a> <b>   latido manual (dm = dead-man; a, b = -1/0/1 para J1 y J2)
 *                 home | goto x y z | ang q1 q2 | stop | off | clear | elbow +-1 | gain ... | pulse j | tele 0/1
 *   ESP32 -> PC:  ST,modo,home1,home2,q1,q2,x,y,falla,fc,duty1,duty2,qref1,qref2   (telemetria)
 *                 #INFO/#ERROR/#FALLA/#HOME/#LLEGO/#PULSO ...                        (eventos en texto)
 *
 * Latido manual: el PC envia "d 1 <a> <b>" ~25 veces por segundo MIENTRAS el espacio (dead-man) este
 * presionado. Sin latido durante DEADMAN_TIMEOUT_MS, o con dm = 0, los motores se detienen.
 */
#pragma once
void comms_start(void);
