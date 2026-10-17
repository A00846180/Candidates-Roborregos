// =====================================================================
//  Candidates 2026 - Pista B, SECCION 1 (encontrar la pelota)
//
//  Que hace:
//   1. Sale del INICIO (abajo a la izquierda) hacia la cuadricula de 3x3.
//   2. Le da la vuelta a la celda de en medio revisando con los
//      ultrasonicos de que lado NO hay pared (abajo, izquierda, derecha;
//      si ninguno esta abierto, es el de arriba).
//   3. Se pone de frente a la entrada con la garra abierta (quieta),
//      avanza despacio y cuando el sensor IR del centro ve la pelota
//      cierra la garra.
//   4. Se sale de reversa por donde entro y se va al checkpoint rojo
//      (a la derecha de la fila de en medio).
//
//
//  El robot se pone en INICIO viendo hacia la cuadricula (hacia "arriba").
//
//  Monitor serie (115200), escribir una letra:
//    1 = correr la seccion 1
//    u = ver ultrasonicos     p = ver sensor de la pelota
//    s = abrir/cerrar garra   g = ver giroscopio
//    c = ver sensor de color  m = menu
//  Si no se escribe nada, a los 3 seg de prenderlo arranca solo.
// =====================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ----------------------------- PINES ---------------------------------
// motores (asi quedaron en la prueba de mecanum)
#define DEL_IZQ_IN1   23
#define DEL_IZQ_IN2   25
#define DEL_DER_IN1   26
#define DEL_DER_IN2   27
#define TRAS_IZQ_IN1  16
#define TRAS_IZQ_IN2  17
#define TRAS_DER_IN1  18
#define TRAS_DER_IN2  19

const uint8_t pinIN1[4] = { DEL_IZQ_IN1, DEL_DER_IN1, TRAS_IZQ_IN1, TRAS_DER_IN1 };
const uint8_t pinIN2[4] = { DEL_IZQ_IN2, DEL_DER_IN2, TRAS_IZQ_IN2, TRAS_DER_IN2 };
bool alReves[4] = {false, true, false, false};   // la delantera der va al reves

// ultrasonicos
#define FRENTE 0
#define IZQ    1
#define DER    2
const uint8_t pinTrig[3] = {4, 5, 33};
const uint8_t pinEcho[3] = {34, 35, 36};

#define PIN_PELOTA  39   // TCRT5000 del centro, avisa cuando entra la pelota
#define PIN_SERVO   32
#define PIN_SDA     21
#define PIN_SCL     22

// --------------------------- AJUSTES ---------------------------------
// velocidades (pwm 0-255)
int velNormal    = 150;
int velMin       = 85;    // menos que esto y los motores casi no se mueven
int velAcercarse = 100;   // para ir por la pelota
int velGiro      = 140;

// lo que marca cada ultrasonico cuando el robot esta centrado en una
// casilla y la pared esta pegada (lo medimos nosotros)
float distFrente = 3.0;
float distLado   = 2.7;
const float CASILLA = 30.0;
float tolerancia = 1.5;      // cm, que tan cerca del objetivo es "ya llegue"
float kDistancia = 8.0;      // que tan rapido se acerca segun el error

// si un ultrasonico marca esto de mas que con pared, ese lado esta abierto
float umbralAbierto = 10.0;

// garra: 90 es como esta quieta (abierta) al buscar
int servoAbierto = 90;
int servoCerrado = 60;      // si cierra al reves probar con 120

// sensor de la pelota
int nivelConPelota = LOW;               // lo que da el TCRT cuando ve la pelota
unsigned long maxTiempoAcercarse = 2500; // ms, por si nunca la ve
unsigned long extraAlAgarrar = 150;      // ms que avanza de mas antes de cerrar

// giroscopio
bool  giroInvertido = false;   // si al girar a la derecha el angulo baja, poner true
float kpRumbo = 4.0;           // correccion mientras avanza
float kpGiro  = 3.0;

// checkpoint
unsigned long tiempoCasilla = 1080;   // ms en recorrer una casilla a velNormal (de la pista A)
unsigned long extraCheckpoint = 300;  // para quedar mas del 50% adentro

#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// ------------------------- OBJETOS -----------------------------------
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
Adafruit_TCS34725 sensorColor(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_MPU6050 mpu;
bool hayOled = false, hayColor = false, hayGiro = false;

float angulo = 0;          // grados, + = a la derecha
float rumbo = 0;           // hacia donde queremos ver
float offsetGiro = 0;
unsigned long ultimaLectura = 0;

// ---------------------------- PWM ------------------------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define CONECTAR_PWM(pin, canal, frec, bits) ledcAttach((pin), (frec), (bits))
  #define ESCRIBIR_PWM(pin, canal, valor)      ledcWrite((pin), (valor))
#else
  #define CONECTAR_PWM(pin, canal, frec, bits) do { ledcSetup((canal), (frec), (bits)); ledcAttachPin((pin), (canal)); } while (0)
  #define ESCRIBIR_PWM(pin, canal, valor)      ledcWrite((canal), (valor))
#endif
#define CANAL_SERVO 4

// ---------------------------- PANTALLA -------------------------------
void mostrar(String titulo, String linea = "") {
  Serial.println(titulo + "  " + linea);
  if (!hayOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(0, 0);
  oled.println(titulo);
  oled.setTextSize(1);
  oled.setCursor(0, 30);
  oled.println(linea);
  oled.display();
}

// ---------------------------- MOTORES --------------------------------
// igual que nuestro setMotor: IN2 bajo + pwm = adelante, IN2 alto = atras
void moverMotor(int i, int vel) {
  if (alReves[i]) vel = -vel;
  vel = constrain(vel, -255, 255);
  if (vel >= 0) {
    digitalWrite(pinIN2[i], LOW);
    ESCRIBIR_PWM(pinIN1[i], i, vel);
  } else {
    digitalWrite(pinIN2[i], HIGH);
    ESCRIBIR_PWM(pinIN1[i], i, 255 + vel);
  }
}

// formula de las mecanum (adelante, lado + = derecha, giro + = derecha)
void mover(int adelante, int lado, int giro) {
  int delIzq  = adelante + lado + giro;
  int delDer  = adelante - lado - giro;
  int trasIzq = adelante - lado + giro;
  int trasDer = adelante + lado - giro;
  int mayor = max(max(abs(delIzq), abs(delDer)), max(abs(trasIzq), abs(trasDer)));
  if (mayor > 255) {
    delIzq  = delIzq  * 255 / mayor;
    delDer  = delDer  * 255 / mayor;
    trasIzq = trasIzq * 255 / mayor;
    trasDer = trasDer * 255 / mayor;
  }
  moverMotor(0, delIzq);
  moverMotor(1, delDer);
  moverMotor(2, trasIzq);
  moverMotor(3, trasDer);
}

void parar() {
  for (int i = 0; i < 4; i++) moverMotor(i, 0);
}

// ----------------------------- GARRA ---------------------------------
void moverServo(int grados) {
  // mismo rango que en nuestras pruebas (1000 a 2000 microsegundos)
  uint32_t pulso = map(constrain(grados, 0, 180), 0, 180, 1000, 2000);
  ESCRIBIR_PWM(PIN_SERVO, CANAL_SERVO, pulso * 65535UL / 20000UL);
}
void abrirGarra()  { moverServo(servoAbierto); }
void cerrarGarra() { moverServo(servoCerrado); }

// -------------------------- GIROSCOPIO -------------------------------
float leerGiroZ() {   // grados por segundo
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  return g.gyro.z * 57.2958;
}

void calibrarGiro() {
  if (!hayGiro) return;
  mostrar("NO MOVER", "calibrando giro");
  float suma = 0;
  for (int i = 0; i < 300; i++) {
    suma += leerGiroZ();
    delay(3);
  }
  offsetGiro = suma / 300;
  ultimaLectura = micros();
}

void actualizarAngulo() {
  if (!hayGiro) return;
  unsigned long ahora = micros();
  float dt = (ahora - ultimaLectura) / 1000000.0;
  ultimaLectura = ahora;
  if (dt > 0.2) return;
  float vel = leerGiroZ() - offsetGiro;
  if (fabs(vel) < 0.3) vel = 0;               // quitar ruidito
  angulo += (giroInvertido ? vel : -vel) * dt;   // z hacia arriba: + es a la izquierda
}

// cuanto tiene que girar para ver hacia "rumbo" (+ = derecha)
float errorRumbo() {
  return fmod(rumbo - angulo + 540.0, 360.0) - 180.0;
}

int correccion() {
  return constrain((int)(kpRumbo * errorRumbo()), -40, 40);
}

void esperar(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    actualizarAngulo();
    delay(2);
  }
}

// gira en su lugar (+ = derecha). ej: girar(90), girar(-90), girar(180)
void girar(float grados) {
  rumbo += grados;
  unsigned long inicio = millis();
  int bien = 0;
  while (millis() - inicio < 4000) {
    actualizarAngulo();
    float error = errorRumbo();
    if (fabs(error) < 2) {
      parar();
      if (++bien > 15) break;   // que se quede quieto un ratito
    } else {
      bien = 0;
      int fuerza = constrain((int)fabs(kpGiro * error), velMin, velGiro);
      mover(0, 0, error > 0 ? fuerza : -fuerza);
    }
    delay(4);
  }
  parar();
}

// --------------------------- SENSORES --------------------------------
float distanciaUnaVez(int s) {
  digitalWrite(pinTrig[s], LOW);
  delayMicroseconds(3);
  digitalWrite(pinTrig[s], HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig[s], LOW);
  unsigned long tiempo = pulseIn(pinEcho[s], HIGH, 25000UL);
  if (tiempo == 0) return 999;   // sin eco = muy lejos
  return tiempo * 0.0343 / 2.0;
}

// 3 lecturas y nos quedamos con la de en medio
float distancia(int s) {
  float m[3];
  for (int i = 0; i < 3; i++) {
    m[i] = distanciaUnaVez(s);
    esperar(6);
  }
  if (m[0] > m[1]) { float t = m[0]; m[0] = m[1]; m[1] = t; }
  if (m[1] > m[2]) { float t = m[1]; m[1] = m[2]; m[2] = t; }
  if (m[0] > m[1]) { float t = m[0]; m[0] = m[1]; m[1] = t; }
  return m[1];
}

bool hayPelota() { return digitalRead(PIN_PELOTA) == nivelConPelota; }

bool pisoRojo() {
  if (!hayColor) return false;
  uint16_t r, g, b, c;
  sensorColor.getRawData(&r, &g, &b, &c);
  return c > 50 && r > g * 1.6 && r > b * 1.6;
}

// ------------------- MOVERSE CON LOS ULTRASONICOS --------------------
// velocidad segun que tan lejos estamos del objetivo (mas lento al llegar)
int velPorError(float error) {
  int v = constrain((int)(fabs(error) * kDistancia), velMin, velNormal);
  return error > 0 ? v : -v;
}

// avanza o retrocede hasta que el ultrasonico de enfrente marque "objetivo"
void irHastaFrente(float objetivo) {
  unsigned long inicio = millis();
  while (millis() - inicio < 6000) {
    float d = distancia(FRENTE);
    float error = d - objetivo;          // + = estamos lejos, hay que avanzar
    if (fabs(error) < tolerancia) break;
    mover(velPorError(error), 0, correccion());
  }
  parar();
  esperar(100);
}

// se va de lado hasta que el ultrasonico izquierdo marque "objetivo"
void irHastaIzq(float objetivo) {
  unsigned long inicio = millis();
  while (millis() - inicio < 6000) {
    float d = distancia(IZQ);
    float error = d - objetivo;          // + = estamos lejos de la pared, ir a la izquierda
    if (fabs(error) < tolerancia) break;
    mover(0, -velPorError(error), correccion());
  }
  parar();
  esperar(100);
}

// lo que deberia marcar cada sensor si hay "n" casillas libres antes de la pared
float frenteCon(int n) { return distFrente + n * CASILLA; }
float ladoCon(int n)   { return distLado + n * CASILLA; }

// ------------------------- LA PELOTA ---------------------------------
// avanza con la garra abierta hasta que el IR del centro ve la pelota,
// cierra, y se sale de reversa lo mismo que avanzo
bool agarrarPelota() {
  mostrar("PELOTA", "acercandome");
  abrirGarra();
  esperar(300);

  unsigned long inicio = millis();
  bool laVi = false;
  while (millis() - inicio < maxTiempoAcercarse) {
    actualizarAngulo();
    if (hayPelota()) { laVi = true; break; }
    mover(velAcercarse, 0, correccion());
    delay(3);
  }
  if (laVi) esperar(extraAlAgarrar);   // sigue avanzando un poquito
  unsigned long tiempoAdentro = millis() - inicio;
  parar();

  cerrarGarra();
  esperar(500);
  mostrar("PELOTA", laVi ? "la tengo!" : "no la vi");

  // de reversa el mismo tiempo para salir de la celda
  inicio = millis();
  while (millis() - inicio < tiempoAdentro) {
    actualizarAngulo();
    mover(-velAcercarse, 0, correccion());
    delay(3);
  }
  parar();
  esperar(200);
  return laVi;
}

// ------------------------- SECCION 1 ---------------------------------
// casillas: columna 0 izquierda a 2 derecha, fila 0 abajo a 2 arriba.
// la pelota esta en (1,1) y el INICIO abajo de (0,0).
// casi todo se hace viendo hacia arriba, asi:
//   el de enfrente mide la pared de arriba
//   el izquierdo mide la pared de la izquierda
//   el derecho apunta hacia la derecha

// revisa si el sensor que ve hacia la celda del centro esta libre
bool estaAbierto(int sensor) {
  float d = distancia(sensor);
  float conPared = (sensor == FRENTE) ? distFrente : distLado;
  Serial.printf("  sensor %d marca %.1f cm\n", sensor, d);
  return d > conPared + umbralAbierto;
}

void llegarAlCheckpoint() {
  mostrar("CHECKPOINT", "buscando el rojo");
  // estamos en (2,1) viendo hacia arriba, el checkpoint esta a la derecha
  unsigned long inicio = millis();
  bool vioRojo = false;
  while (millis() - inicio < tiempoCasilla * 1.4) {
    actualizarAngulo();
    if (pisoRojo()) { vioRojo = true; break; }
    mover(0, velNormal, correccion());
    delay(3);
  }
  esperar(extraCheckpoint);   // meterse bien al checkpoint
  parar();
  mostrar("CHECKPOINT 1", vioRojo ? "llegue!" : "no vi el rojo");
}

void seccion1() {
  rumbo = 0;
  angulo = 0;
  abrirGarra();   // la garra se queda quieta abierta mientras buscamos

  // 1) del inicio a la casilla (0,0): adelante hasta que falten 2 casillas
  mostrar("SECCION 1", "entrando");
  irHastaFrente(frenteCon(2));
  irHastaIzq(ladoCon(0));

  // 2) revisar el lado de ABAJO desde (1,0)
  irHastaIzq(ladoCon(1));
  mostrar("REVISANDO", "lado de abajo");
  if (estaAbierto(FRENTE)) {
    // ya estamos viendo hacia la entrada
    agarrarPelota();
    irHastaIzq(ladoCon(2));        // a (2,0)
    irHastaFrente(frenteCon(1));   // a (2,1)
    llegarAlCheckpoint();
    return;
  }

  // 3) revisar el lado IZQUIERDO desde (0,1)
  irHastaIzq(ladoCon(0));          // a (0,0)
  irHastaFrente(frenteCon(1));     // a (0,1)
  mostrar("REVISANDO", "lado izquierdo");
  if (estaAbierto(DER)) {
    girar(90);                     // la garra hacia el centro
    agarrarPelota();
    girar(-90);                    // otra vez viendo hacia arriba
    irHastaIzq(ladoCon(0));
    irHastaFrente(frenteCon(0));   // a (0,2)
    irHastaIzq(ladoCon(2));        // a (2,2)
    irHastaFrente(frenteCon(1));   // a (2,1) de reversa
    llegarAlCheckpoint();
    return;
  }

  // 4) revisar el lado DERECHO desde (2,1), rodeando por arriba
  irHastaFrente(frenteCon(0));     // a (0,2)
  irHastaIzq(ladoCon(2));          // a (2,2)
  irHastaFrente(frenteCon(1));     // a (2,1) de reversa
  mostrar("REVISANDO", "lado derecho");
  if (estaAbierto(IZQ)) {
    girar(-90);                    // la garra hacia el centro
    agarrarPelota();
    girar(90);
    irHastaFrente(frenteCon(1));   // re-acomodarse en (2,1)
    llegarAlCheckpoint();
    return;
  }

  // 5) si ninguno estaba abierto, la entrada es la de ARRIBA
  mostrar("REVISANDO", "lado de arriba");
  irHastaFrente(frenteCon(0));     // a (2,2)
  irHastaIzq(ladoCon(1));          // a (1,2)
  girar(180);                      // la garra hacia el centro
  if (!estaAbierto(FRENTE)) Serial.println("  ojo: arriba tambien se ve cerrado, lo intento igual");
  agarrarPelota();
  girar(180);                      // otra vez viendo hacia arriba
  irHastaFrente(frenteCon(0));
  irHastaIzq(ladoCon(2));          // a (2,2)
  irHastaFrente(frenteCon(1));     // a (2,1)
  llegarAlCheckpoint();
}

// ---------------------------- MENU -----------------------------------
void menu() {
  Serial.println("");
  Serial.println("1 correr seccion 1 | u ultrasonicos | p sensor pelota");
  Serial.println("s abrir/cerrar garra | g giroscopio | c color | m menu");
}

bool tecleo() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

void pruebas(char letra) {
  if (letra == '1') {
    mostrar("SECCION 1", "en 2 seg...");
    delay(2000);
    seccion1();
  } else if (letra == 'u') {
    while (!tecleo()) {
      Serial.printf("frente %.1f  izq %.1f  der %.1f\n", distancia(FRENTE), distancia(IZQ), distancia(DER));
      delay(150);
    }
  } else if (letra == 'p') {
    while (!tecleo()) {
      Serial.println(hayPelota() ? "PELOTA" : "-");
      delay(150);
    }
  } else if (letra == 's') {
    static bool abierta = true;
    abierta = !abierta;
    if (abierta) abrirGarra(); else cerrarGarra();
    Serial.println(abierta ? "garra abierta" : "garra cerrada");
  } else if (letra == 'g') {
    angulo = 0;
    while (!tecleo()) {
      esperar(200);
      Serial.printf("angulo %.1f\n", angulo);
    }
  } else if (letra == 'c') {
    while (!tecleo()) {
      uint16_t r, g, b, c;
      sensorColor.getRawData(&r, &g, &b, &c);
      Serial.printf("r %u g %u b %u c %u %s\n", r, g, b, c, pisoRojo() ? "<- ROJO" : "");
      delay(200);
    }
  } else {
    menu();
  }
}

// ------------------------- SETUP Y LOOP ------------------------------
bool enMenu = false;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(pinIN2[i], OUTPUT);
    digitalWrite(pinIN2[i], LOW);
    CONECTAR_PWM(pinIN1[i], i, PWM_FREQ, PWM_RESOLUTION);
  }
  parar();
  CONECTAR_PWM(PIN_SERVO, CANAL_SERVO, 50, 16);
  abrirGarra();

  for (int i = 0; i < 3; i++) {
    pinMode(pinTrig[i], OUTPUT);
    digitalWrite(pinTrig[i], LOW);
    pinMode(pinEcho[i], INPUT);
  }
  pinMode(PIN_PELOTA, INPUT);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  hayOled  = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  hayColor = sensorColor.begin();
  hayGiro  = mpu.begin(0x68, &Wire);
  if (hayGiro) {
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  Serial.printf("oled %d  color %d  giro %d\n", hayOled, hayColor, hayGiro);

  calibrarGiro();

  // 3 seg para escribir algo y entrar al menu; si no, arranca solo
  mostrar("SECCION 1", "arranca en 3 seg");
  unsigned long inicio = millis();
  while (millis() - inicio < 3000) {
    if (Serial.available()) { enMenu = true; break; }
    esperar(20);
  }
  if (enMenu) {
    mostrar("MENU", "monitor serie");
    menu();
    return;
  }
  seccion1();
}

void loop() {
  if (Serial.available()) {
    char letra = Serial.read();
    if (letra != '\n' && letra != '\r' && letra != ' ') pruebas(letra);
  }
  actualizarAngulo();
  delay(5);
}
