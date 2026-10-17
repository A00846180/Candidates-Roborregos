/*
Notas: Esta es una modificación del código luego de haberlo probado físicamente.
Nos dimos cuenta de la limitación física que supone que el carrito sea de casi 27cm de ancho.
*/

//secciones que fueron corregidas:
///constantes de calibración
///agregamos centrado en los ultrasonicos laterales
///laberinto y "mapeo" de este (para backtracking)
///dfs con backtracking
///movimiento del carrito

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>

// pines

// hc-sr04
#define TRIG_FRENTE     4
#define ECHO_FRENTE     34
#define TRIG_IZQ        5
#define ECHO_IZQ        35
#define TRIG_DER        33
#define ECHO_DER        36

// driver de los motores delanteros
#define FL_IN1  16   // AIN1
#define FL_IN2  17   // AIN2
#define FR_IN1  18   // BIN1
#define FR_IN2  19   // BIN2

// driver de los motores traseros
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

// sensor de color
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// constantes de calibración

// umbrales para decidir si hay pared (distinto por sensor)
#define UMBRAL_PARED_FRENTE_CM   10.0   // frente: menor a esto = pared
#define UMBRAL_PARED_LATERAL_CM  5.0    // izquierda y derecha: menor a esto = pared

#define DIST_LATERAL_IDEAL_CM 5.0    // (30 - 20) / 2: distancia a cada pared lateral si va centrado

// velocidades (pwm 0-255). dependen de ellas: TIEMPO_AVANCE_MS, TIEMPO_STRAFE_MS,
// KP_CENTRADO y CORR_MAX
#define VELOCIDAD_BASE        80     // avance / reversa
#define VELOCIDAD_STRAFE      80     // strafe lateral (si no se mueve o se atora, súbele solo a esta)

// tiempos por celda: puntos de partida, hay que recalibrarlos en pista
// (a 150 eran 1080 ms; a 80 el carrito es bastante más lento, y no es lineal)
#define TIEMPO_AVANCE_MS      2000   // 1 celda hacia adelante/atrás
#define TIEMPO_STRAFE_MS      2000   // 1 celda lateral

// impulso de arranque: a pwm 80 los motores pueden no vencer la inercia desde el reposo,
// así que se da un golpe corto más fuerte y luego baja a la velocidad normal
#define VELOCIDAD_IMPULSO  140
#define IMPULSO_MS         60

// centrado con los ultrasónicos laterales mientras avanza/retrocede
#define UMBRAL_CENTRADO_CM 15.0  // hasta qué distancia usa una pared lateral para centrarse
#define PASO_CONTROL_MS   40     // cada cuánto corrige
#define KP_CENTRADO       3.0    // pwm extra por cada cm de error
#define CORR_MAX          20     // límite de corrección (pwm); con base 80 las ruedas quedan entre 60 y 100

// laberinto
// el mapa es RELATIVO al punto de inicio: el inicio queda en el centro de una
// cuadrícula de 9x9, así no importa en qué esquina empiece ni hacia dónde mire.
// NORTE = hacia donde mira el carrito al empezar (nunca cambia, no gira)
#define GRID_SIZE   9
#define INICIO_FILA 4
#define INICIO_COL  4

enum Direccion { NORTE = 0, ESTE = 1, SUR = 2, OESTE = 3 };

bool visitado[GRID_SIZE][GRID_SIZE];

// pila de camino array-based
int pathFila[GRID_SIZE * GRID_SIZE];
int pathCol[GRID_SIZE * GRID_SIZE];
int pathTop = -1;

int filaActual = INICIO_FILA;
int colActual = INICIO_COL;

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

// main loop
void loop() {
  esperarVerde();
  bool encontroRojo = explorarLaberinto();
  detenerMotores();
  if (encontroRojo) mostrarMensaje("Rojo: de vuelta en el inicio");
  else              mostrarMensaje("Sin rojo: ya no hay rutas");
  while (true) { delay(1000); } // fin de ronda, se detiene
}

// el carrito no se mueve hasta ver verde
void esperarVerde() {
  mostrarMensaje("Esperando verde...");
  while (detectarColorEnCelda() != "verde") {
    delay(100);
  }
  mostrarMensaje("Verde: iniciando");
  delay(500);
}

// dfs con backtracking (exploración del carrito)
// regresa true si vio rojo (y ya regresó al inicio), false si no quedan rutas
bool explorarLaberinto() {
  pushCamino(filaActual, colActual);
  visitado[filaActual][colActual] = true;

  bool revisarColor = false; // el verde del inicio ya se leyó en esperarVerde()

  while (true) {
    String color = "";
    if (revisarColor) color = detectarColorEnCelda();
    revisarColor = false;

    // al ver rojo deja de explorar y regresa por el camino recorrido
    if (color == "rojo") {
      detenerMotores();
      mostrarMensaje("Rojo: backtrack");
      delay(500);
      regresarAlInicio();
      return true;
    }

    // escanea las 3 direcciones que alcanza a ver (atrás no hay sensor)
    float dFrente = medirDistanciaPromediada(TRIG_FRENTE, ECHO_FRENTE);
    float dIzq    = medirDistanciaPromediada(TRIG_IZQ, ECHO_IZQ);
    float dDer    = medirDistanciaPromediada(TRIG_DER, ECHO_DER);

    bool libreFrente = !esPared(dFrente, UMBRAL_PARED_FRENTE_CM);
    bool libreIzq    = !esPared(dIzq,    UMBRAL_PARED_LATERAL_CM);
    bool libreDer    = !esPared(dDer,    UMBRAL_PARED_LATERAL_CM);

    // para depurar: distancias en serial (-1 = sin eco)
    Serial.printf("celda (%d,%d) F:%.1f I:%.1f D:%.1f\n",
                  filaActual, colActual, dFrente, dIzq, dDer);

    int sigFila, sigCol;
    bool encontroNoVisitada = false;
    Direccion dirElegida = NORTE;

    // prioridad: frente > derecha > izquierda
    if (libreFrente && celdaValidaNoVisitada(NORTE, sigFila, sigCol)) {
      dirElegida = NORTE;
      encontroNoVisitada = true;
    } else if (libreDer && celdaValidaNoVisitada(ESTE, sigFila, sigCol)) {
      dirElegida = ESTE;
      encontroNoVisitada = true;
    } else if (libreIzq && celdaValidaNoVisitada(OESTE, sigFila, sigCol)) {
      dirElegida = OESTE;
      encontroNoVisitada = true;
    }

    if (encontroNoVisitada) {
      moverHaciaDireccion(dirElegida);
      filaActual = sigFila;
      colActual = sigCol;
      visitado[filaActual][colActual] = true;
      pushCamino(filaActual, colActual);
      revisarColor = true; // celda nueva: leer su color
    } else {
      // backtrack
      if (pathTop <= 0) return false; // exploró todo lo que pudo sin ver rojo
      retrocederUnaCelda();
    }
  }
}

// da un paso de backtrack hacia la celda anterior de la pila
void retrocederUnaCelda() {
  popCamino();
  int filaAnterior = pathFila[pathTop];
  int colAnterior  = pathCol[pathTop];
  Direccion dirRetroceso = calcularDireccionEntreCeldas(filaActual, colActual, filaAnterior, colAnterior);
  moverHaciaDireccion(dirRetroceso);
  filaActual = filaAnterior;
  colActual  = colAnterior;
}

// deshace todo el camino hasta la celda de inicio
void regresarAlInicio() {
  while (pathTop > 0) {
    retrocederUnaCelda();
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

// movimiento del carrito (sin giros: la orientación nunca cambia)
//   NORTE = avanzar | SUR = reversa | ESTE = strafe derecha | OESTE = strafe izquierda

void moverHaciaDireccion(Direccion dir) {
  switch (dir) {
    case NORTE: moverRecto(+1);        break;
    case SUR:   moverRecto(-1);        break;
    case ESTE:  strafeLateral(true);   break;
    case OESTE: strafeLateral(false);  break;
  }
}

// avanza (+1) o retrocede (-1) una celda, centrándose con los ultrasónicos laterales
void moverRecto(int sentido) {
  impulsoArranque(sentido, 0);
  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_AVANCE_MS) {
    int vy = calcularCorreccionLateral();
    setMotoresMecanum(sentido * VELOCIDAD_BASE, vy);
    delay(PASO_CONTROL_MS);
  }
  detenerMotores();
}

// strafe de una celda hacia un lado
void strafeLateral(bool haciaDerecha) {
  int vy = haciaDerecha ? VELOCIDAD_STRAFE : -VELOCIDAD_STRAFE;
  impulsoArranque(0, haciaDerecha ? 1 : -1);
  setMotoresMecanum(0, vy);
  delay(TIEMPO_STRAFE_MS);
  detenerMotores();
}

// golpe corto de potencia al arrancar; sx = +1/-1 adelante/atrás, sy = +1/-1 derecha/izquierda
void impulsoArranque(int sx, int sy) {
  setMotoresMecanum(sx * VELOCIDAD_IMPULSO, sy * VELOCIDAD_IMPULSO);
  delay(IMPULSO_MS);
}

// regresa velocidad lateral extra (+ derecha, - izquierda) para mantenerse centrado
int calcularCorreccionLateral() {
  float izq = medirDistancia(TRIG_IZQ, ECHO_IZQ);
  float der = medirDistancia(TRIG_DER, ECHO_DER);
  bool hayIzq = (izq > 0 && izq < UMBRAL_CENTRADO_CM);
  bool hayDer = (der > 0 && der < UMBRAL_CENTRADO_CM);

  float err = 0;
  if (hayIzq && hayDer)  err = (der - izq) / 2.0;            // más cerca de la izq -> se va a la derecha
  else if (hayIzq)       err = DIST_LATERAL_IDEAL_CM - izq;  // solo pared izq
  else if (hayDer)       err = der - DIST_LATERAL_IDEAL_CM;  // solo pared der
  // sin paredes a los lados: no corrige

  return constrain((int)(KP_CENTRADO * err), -CORR_MAX, CORR_MAX);
}

// cinemática mecanum: vx = adelante(+)/atrás(-), vy = derecha(+)/izquierda(-)
void setMotoresMecanum(int vx, int vy) {
  int fl = constrain(vx + vy, -255, 255);
  int fr = constrain(vx - vy, -255, 255);
  int rl = constrain(vx - vy, -255, 255);
  int rr = constrain(vx + vy, -255, 255);
  setMotores(fl, fr, rl, rr);
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

// regresa distancia en cm, o -1 si no hubo eco
float medirDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  long duracion = pulseIn(pinEcho, HIGH, 30000); // timeout 30ms
  if (duracion == 0) return -1.0;

  return duracion * 0.0343 / 2.0;
}

// promedia hasta 3 lecturas válidas; si ninguna tuvo eco regresa -1
float medirDistanciaPromediada(int pinTrig, int pinEcho) {
  float suma = 0;
  int validas = 0;
  for (int i = 0; i < 3; i++) {
    float d = medirDistancia(pinTrig, pinEcho);
    if (d > 0) { suma += d; validas++; }
    delay(10);
  }
  return (validas == 0) ? -1.0 : suma / validas;
}

// pared solo si hay eco y está más cerca que el umbral; sin eco (-1) se considera libre
bool esPared(float distancia, float umbral) {
  return (distancia >= 0) && (distancia < umbral);
}

// detección de colores

String detectarColorEnCelda() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // descomenta para calibrar los colores en la pista
  // Serial.printf("R:%u G:%u B:%u\n", r, g, b);

  String colorDetectado = clasificarColor(r, g, b);

  if (colorDetectado != "") {
    mostrarMensaje("Color: " + colorDetectado);
    delay(500); // para que se pueda leer a tiempo
  }
  return colorDetectado;
}

// referencias en proporciones (r/suma, g/suma, b/suma), sacadas de lecturas reales en pista:
// rojo 150,59,46 | rosa 265,101,91 | verde 80,65,39 | amarillo 159,109,60 | naranja 79,74,58 | azul 136,66,43
// uso proporciones y no valores absolutos para que no importe tanto la luz o la altura del sensor
struct ColorRef { const char* nombre; float r, g, b; };

const ColorRef REFS[] = {
  {"rojo",     0.588, 0.231, 0.180},
  {"rosa",     0.580, 0.221, 0.199},
  {"verde",    0.435, 0.353, 0.212},
  {"amarillo", 0.485, 0.332, 0.183},
  {"naranja",  0.374, 0.351, 0.275},
  {"azul",     0.555, 0.269, 0.176}
};
const int NUM_REFS = sizeof(REFS) / sizeof(REFS[0]);

#define DIST2_MAX            0.0025  // si ninguna referencia queda cerca (0.05^2) se considera piso
#define SUMA_MIN_VALIDA      50      // lectura muy oscura = no confiable
#define SUMA_ROJO_VS_ROSA    350     // rosa se lee mucho más intenso que rojo/azul (~457 vs ~250)
#define GR_ROJO_VS_AZUL      0.44    // g/r: rojo ~0.39, azul ~0.49 (¡diferencia chica, recalibrar!)

String clasificarColor(uint16_t r, uint16_t g, uint16_t b) {
  float suma = (float)r + g + b;
  if (suma < SUMA_MIN_VALIDA) return "";

  float rn = r / suma, gn = g / suma, bn = b / suma;

  // referencia más cercana en el espacio de proporciones
  int mejor = -1;
  float mejorDist2 = 1e9;
  for (int i = 0; i < NUM_REFS; i++) {
    float dr = rn - REFS[i].r, dg = gn - REFS[i].g, db = bn - REFS[i].b;
    float d2 = dr * dr + dg * dg + db * db;
    if (d2 < mejorDist2) { mejorDist2 = d2; mejor = i; }
  }

  if (mejor < 0 || mejorDist2 > DIST2_MAX) return ""; // no hay color (el piso es blanco)

  String nombre = REFS[mejor].nombre;

  // rojo, rosa y azul quedan casi en el mismo lugar en proporciones, así que se separan aparte:
  // primero por intensidad (rosa) y luego por la relación g/r (rojo vs azul)
  if (nombre == "rojo" || nombre == "rosa" || nombre == "azul") {
    if (suma > SUMA_ROJO_VS_ROSA) return "rosa";
    return ((gn / rn) > GR_ROJO_VS_AZUL) ? "azul" : "rojo";
  }
  return nombre;
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
