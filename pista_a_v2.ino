#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>

// pines (por definir todavía)

// hc-sr04 
#define TRIG_FRENTE     4
#define ECHO_FRENTE     34
#define TRIG_IZQ        5
#define ECHO_IZQ        35
#define TRIG_DER        33
#define ECHO_DER        36 

// driver de los motores delanteros (U5)
#define FL_IN1  16   // AIN1
#define FL_IN2  17   // AIN2
#define FR_IN1  19   // BIN1
#define FR_IN2  18   // BIN2

// driver de los motores traseros (U6)
#define RL_IN1  23   // AIN1
#define RL_IN2  25   // AIN2
#define RR_IN1  26   // BIN1
#define RR_IN2  27   // BIN2

// pulse width modulation
#define PWM_FREQ      1000
#define PWM_RESOLUTION 8

// pantalla oled
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ssd1306
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// constantes de calibración

#define UMBRAL_PARED_CM     20.0   // si detecta valor menor a este le indica al carrito que hay pared
#define VELOCIDAD_BASE      180    
#define TIEMPO_AVANCE_MS    900    // tiempo en el que el carrito recorre 1 unidad (se calibrará en pista)
#define TIEMPO_GIRO90_MS    550    // tiempo que le toma girar 90° (se calibrará en pista)
#define TIEMPO_STRAFE_MS    900    // tiempo para el strafe lateral de 1 unidad (se calibrará en pista)

// laberinto

#define GRID_SIZE 5

enum Direccion { NORTE = 0, ESTE = 1, SUR = 2, OESTE = 3 };

bool visitado[GRID_SIZE][GRID_SIZE];

// pila de camino array-based
int pathFila[GRID_SIZE * GRID_SIZE];
int pathCol[GRID_SIZE * GRID_SIZE];
int pathTop = -1;

int filaActual = 0;
int colActual = 0;
Direccion orientacionActual = NORTE;

// setup del carrito

void setup() {
  Serial.begin(115200);

  // ultrasónicos
  pinMode(TRIG_FRENTE, OUTPUT); pinMode(ECHO_FRENTE, INPUT);
  pinMode(TRIG_IZQ, OUTPUT);    pinMode(ECHO_IZQ, INPUT);
  pinMode(TRIG_DER, OUTPUT);    pinMode(ECHO_DER, INPUT);

  // motores
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  // canales pwm
  ledcAttach(FL_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(FR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RL_IN1, PWM_FREQ, PWM_RESOLUTION); 
  ledcAttach(RR_IN1, PWM_FREQ, PWM_RESOLUTION);

  // oled
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error al iniciar OLED");
  }
  display.clearDisplay();
  display.display();

  // sensor de color
  if (!tcs.begin()) {
    Serial.println("Error al iniciar TCS34725");
  }

  // para inicializar mapa
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      visitado[i][j] = false;

  mostrarMensaje("Listo para Candidates 2026");
  delay(2000);
}

//main loop
void loop() {
  explorarLaberinto();
  detenerMotores();
  mostrarMensaje("Fin de recorrido");
  while (true) { delay(1000); } // fin de ronda, se detiene
}

// dfs con backtracking (exploración del carrito)
void explorarLaberinto() {
  pushCamino(filaActual, colActual);
  visitado[filaActual][colActual] = true;

  while (true) {
    detectarColorEnCelda();

    // para que escanee las 3 direcciones posibles
    bool libreFrente = medirDistancia(TRIG_FRENTE, ECHO_FRENTE) > UMBRAL_PARED_CM;
    bool libreIzq    = medirDistancia(TRIG_IZQ, ECHO_IZQ) > UMBRAL_PARED_CM;
    bool libreDer     = medirDistancia(TRIG_DER, ECHO_DER) > UMBRAL_PARED_CM;

    Direccion dirFrenteAbs = orientacionActual;
    Direccion dirIzqAbs    = (Direccion)((orientacionActual + 3) % 4); // -90°
    Direccion dirDerAbs    = (Direccion)((orientacionActual + 1) % 4); // +90°

    int sigFila, sigCol;
    bool encontroNoVisitada = false;
    Direccion dirElegida;

    // aquí priorizo el frente > derecha > izquierda
    if (libreFrente && celdaValidaNoVisitada(dirFrenteAbs, sigFila, sigCol)) {
      dirElegida = dirFrenteAbs;
      encontroNoVisitada = true;
    } else if (libreDer && celdaValidaNoVisitada(dirDerAbs, sigFila, sigCol)) {
      dirElegida = dirDerAbs;
      encontroNoVisitada = true;
    } else if (libreIzq && celdaValidaNoVisitada(dirIzqAbs, sigFila, sigCol)) {
      dirElegida = dirIzqAbs;
      encontroNoVisitada = true;
    }

    if (encontroNoVisitada) {
      moverHaciaDireccion(dirElegida);
      filaActual = sigFila;
      colActual = sigCol;
      visitado[filaActual][colActual] = true;
      pushCamino(filaActual, colActual);
    } else {
      // backtrack
      if (pathTop <= 0) {
        break; // significa que finalizó la exploración
      }
      popCamino();
      int filaAnterior = pathFila[pathTop];
      int colAnterior = pathCol[pathTop];
      Direccion dirRetroceso = calcularDireccionEntreCeldas(filaActual, colActual, filaAnterior, colAnterior);
      moverHaciaDireccion(dirRetroceso);
      filaActual = filaAnterior;
      colActual = colAnterior;
    }
  }
}

bool celdaValidaNoVisitada(Direccion dir, int &filaOut, int &colOut) {
  filaOut = filaActual;
  colOut = colActual;
  if (dir == NORTE) filaOut -= 1;
  else if (dir == SUR) filaOut += 1;
  else if (dir == ESTE) colOut += 1;
  else if (dir == OESTE) colOut -= 1;

  if (filaOut < 0 || filaOut >= GRID_SIZE || colOut < 0 || colOut >= GRID_SIZE) return false;
  return !visitado[filaOut][colOut];
}

Direccion calcularDireccionEntreCeldas(int filaDesde, int colDesde, int filaHacia, int colHacia) {
  if (filaHacia == filaDesde - 1) return NORTE;
  if (filaHacia == filaDesde + 1) return SUR;
  if (colHacia == colDesde + 1) return ESTE;
  return OESTE;
}

void pushCamino(int f, int c) {
  pathTop++;
  pathFila[pathTop] = f;
  pathCol[pathTop] = c;
}

void popCamino() {
  pathTop--;
}

// movimiento del carrito

void moverHaciaDireccion(Direccion dirAbsoluta) {
  if (dirAbsoluta == orientacionActual) {
    avanzar();
  } else if (dirAbsoluta == (Direccion)((orientacionActual + 2) % 4)) {
    // si es dirección opuesta en vez de regresar retrocede a la derecha
    retroceder();
  } else if (dirAbsoluta == (Direccion)((orientacionActual + 1) % 4)) {
    girar90(true); // derecha
    orientacionActual = dirAbsoluta;
    avanzar();
  } else {
    girar90(false); // izquierda
    orientacionActual = dirAbsoluta;
    avanzar();
  }
}

void avanzar() {
  setMotores(VELOCIDAD_BASE, VELOCIDAD_BASE, VELOCIDAD_BASE, VELOCIDAD_BASE);
  delay(TIEMPO_AVANCE_MS);
  detenerMotores();
}

void retroceder() {
  setMotores(-VELOCIDAD_BASE, -VELOCIDAD_BASE, -VELOCIDAD_BASE, -VELOCIDAD_BASE);
  delay(TIEMPO_AVANCE_MS);
  detenerMotores();
}

void girar90(bool haciaDerecha) {
  int signo = haciaDerecha ? 1 : -1;
  // giro sobre su propio eje: ruedas de un lado adelante, del otro atrás
  setMotores(signo * VELOCIDAD_BASE, -signo * VELOCIDAD_BASE,
             signo * VELOCIDAD_BASE, -signo * VELOCIDAD_BASE);
  delay(TIEMPO_GIRO90_MS);
  detenerMotores();
}

// Placeholder para cuando decidan usar strafe lateral en vez de girar+avanzar+girar
void strafeLateral(bool haciaDerecha) {
  int signo = haciaDerecha ? 1 : -1;
  // Cinemática mecanum básica para desplazamiento lateral puro:
  setMotores(signo * VELOCIDAD_BASE, -signo * VELOCIDAD_BASE,
             -signo * VELOCIDAD_BASE, signo * VELOCIDAD_BASE);
  delay(TIEMPO_STRAFE_MS);
  detenerMotores();
}

// Escribe velocidad con signo (+ adelante, - atrás) a cada motor
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
    ledcWrite(pinIN1, 255 + velocidad); // velocidad es negativa aquí
  }
}

void detenerMotores() {
  setMotores(0, 0, 0, 0);
}

// sensores

float medirDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  long duracion = pulseIn(pinEcho, HIGH, 30000); // timeout 30ms
  if (duracion == 0) return 999.0; // sin eco = asumir libre o revisar cableado

  float distanciaCm = duracion * 0.0343 / 2.0;
  return distanciaCm;
}

// Promedia 3 lecturas para reducir ruido
float medirDistanciaPromediada(int pinTrig, int pinEcho) {
  float suma = 0;
  for (int i = 0; i < 3; i++) {
    suma += medirDistancia(pinTrig, pinEcho);
    delay(10);
  }
  return suma / 3.0;
}

// detección de colores

void detectarColorEnCelda() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // esto se calibra en la pista ya observando los colores reales
  String colorDetectado = clasificarColor(r, g, b);

  if (colorDetectado != "") {
    mostrarMensaje("Color: " + colorDetectado);
    delay(500); // para que se pueda leer a tiempo
  }
}

String clasificarColor(uint16_t r, uint16_t g, uint16_t b) {
  // colores de la pista (se calibrará en la pista)
  if (r > 150 && g < 200 && b < 100) return "naranja";
  if (r > 150 && g > 150 && b < 100) return "amarillo";
  if (r < 100 && g > 150 && b > 150) return "cian";
  if (r > 150 && g < 210 && b > 100) return "rosa";
  return ""; // no hay color (el piso es color blanco)
}

// oled

void mostrarMensaje(String texto) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(texto);
  display.display();
}
