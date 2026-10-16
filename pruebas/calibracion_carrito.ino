/*
  calibración del carrito (avance, giro 90° y strafe)
  Sube este código, pon el carrito en la pista y ve apuntando lo que mide.
  No necesita monitor serial, todo se ve en la OLED, así que puedes
  desconectar el USB y usar solo la batería.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== qué quieres calibrar (deja solo UNA en 1) =====
#define PRUEBA_AVANCE  0
#define PRUEBA_GIRO    0
#define PRUEBA_STRAFE  1

// ===== tiempos de prueba (los cambias entre subidas) =====
#define VELOCIDAD_BASE   100
#define TIEMPO_PRUEBA_MS 1080   // avance: se mide cuántos cm recorre en este tiempo
                                // giro: ajústalo hasta que gire justo 90°
                                // strafe: igual que avance, pero de lado
#define REPETICIONES     3
#define ESPERA_MEDIR_MS  6000   // tiempo para que midas y regreses el carrito

// pines (los mismos del código final)
#define FL_IN1  16
#define FL_IN2  17
#define FR_IN1  18
#define FR_IN2  19
#define RL_IN1  23
#define RL_IN2  25
#define RR_IN1  26
#define RR_IN2  27

#define PWM_FREQ        1000
#define PWM_RESOLUTION  8

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  ledcAttach(FL_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(FR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RL_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RR_IN1, PWM_FREQ, PWM_RESOLUTION);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  detenerMotores();
}

void loop() {
  for (int i = 1; i <= REPETICIONES; i++) {
    cuentaRegresiva();

#if PRUEBA_AVANCE
    mostrar("AVANCE\nPrueba " + String(i) + "/" + String(REPETICIONES) +
            "\nT=" + String(TIEMPO_PRUEBA_MS) + " ms");
    setMotores(VELOCIDAD_BASE, VELOCIDAD_BASE, VELOCIDAD_BASE, VELOCIDAD_BASE);
#elif PRUEBA_GIRO
    mostrar("GIRO DERECHA\nPrueba " + String(i) + "/" + String(REPETICIONES) +
            "\nT=" + String(TIEMPO_PRUEBA_MS) + " ms");
    setMotores(VELOCIDAD_BASE, -VELOCIDAD_BASE, VELOCIDAD_BASE, -VELOCIDAD_BASE);
#elif PRUEBA_STRAFE
    mostrar("STRAFE DERECHA\nPrueba " + String(i) + "/" + String(REPETICIONES) +
            "\nT=" + String(TIEMPO_PRUEBA_MS) + " ms");
    setMotores(VELOCIDAD_BASE, -VELOCIDAD_BASE, -VELOCIDAD_BASE, VELOCIDAD_BASE);
#endif

    delay(TIEMPO_PRUEBA_MS);
    detenerMotores();

    mostrar("MIDE AHORA\ny regresa el\ncarrito al inicio");
    delay(ESPERA_MEDIR_MS);
  }

  mostrar("Terminado\n\nPromedia tus\nmediciones");
  while (true) { delay(1000); }
}

void cuentaRegresiva() {
  for (int s = 3; s > 0; s--) {
    mostrar("Saliendo en\n" + String(s));
    delay(1000);
  }
}

void setMotores(int fl, int fr, int rl, int rr) {
  setMotor(FL_IN1, FL_IN2, fl);
  setMotor(FR_IN1, FR_IN2, fr);
  setMotor(RL_IN1, RL_IN2, rl);
  setMotor(RR_IN1, RR_IN2, rr);
}

void setMotor(int pinIN1, int pinIN2, int velocidad) {
  if (velocidad >= 0) {
    digitalWrite(pinIN2, LOW);
    ledcWrite(pinIN1, velocidad);
  } else {
    digitalWrite(pinIN2, HIGH);
    ledcWrite(pinIN1, 255 + velocidad);
  }
}

void detenerMotores() {
  setMotores(0, 0, 0, 0);
}

void mostrar(String texto) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(texto);
  display.display();
}
