# esp-joy

Lectura de los eventos de un Pro Controller Gamepad o compatible.

Probado con un EasySMX 9124.

Utiliza el modo Switch pulsando X + Home en el mando.  En ese momento resetea el ESP32, se vinculará automáticamente.

Usa BT Classic con Bluedroid porque no parece que soporte BLE.

En principio no sería difícil que funcionara cualquier dispositivo BT HID.

Funciona con ESP32, pero no con ESP32C3.  El C3 no soporte BT Classic.
