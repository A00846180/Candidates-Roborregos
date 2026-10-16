//Microcontrolador ESP32

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
// Declaraciones utilizadas cuando el framework de Arduino no está disponible para el editor.
struct SerialPort {
  void begin(unsigned long);
  void println(const char*);
};
extern SerialPort Serial;
void delay(unsigned long);
#endif

void setup() {
  Serial.begin(115200); // Inicia la comunicación serial a 115200 baudios
}

void loop() {
  Serial.println("Hola desde el ESP32");
  delay(1000); // Espera 1 segundo (1000 milisegundos)
}
