// =====================================================================
//  Prueba: detectar la pelota con el infrarrojo y cerrar la garra
//
//  - Al prender, la garra queda abierta (90 grados).
//  - Cuando el TCRT5000 del centro ve la pelota, cierra la garra.
//  - Se queda cerrada. Para volver a abrirla, reiniciar el ESP32
//    o escribir "a" en el monitor serie (115200).
// =====================================================================

#include <ESP32Servo.h>

#define PIN_PELOTA  39   // TCRT5000 del centro
#define PIN_SERVO   32

int servoAbierto = 20;
int servoCerrado = 60;      // si cierra al reves, probar con 120
int nivelConPelota = LOW;   // lo que da el TCRT cuando ve la pelota

Servo garra;
bool cerrada = false;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PELOTA, INPUT);   // el 39 no tiene pull-up interno

  garra.setPeriodHertz(50);
  garra.attach(PIN_SERVO, 1000, 2000);   // mismo rango que en nuestras pruebas
  garra.write(servoAbierto);

  Serial.println("garra abierta, esperando la pelota...");
}

void loop() {
  // abrir otra vez desde el monitor serie
  if (Serial.available()) {
    char letra = Serial.read();
    if (letra == 'a') {
      garra.write(servoAbierto);
      cerrada = false;
      Serial.println("garra abierta, esperando la pelota...");
      delay(500);   // tiempo para quitar la mano
    }
  }

  if (!cerrada && digitalRead(PIN_PELOTA) == nivelConPelota) {
    delay(50);   // confirmar que si es la pelota y no un brinco del sensor
    if (digitalRead(PIN_PELOTA) == nivelConPelota) {
      garra.write(servoCerrado);
      cerrada = true;
      Serial.println("pelota detectada, garra cerrada");
    }
  }

  delay(10);
}
