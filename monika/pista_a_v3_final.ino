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