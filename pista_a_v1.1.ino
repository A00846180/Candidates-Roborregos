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
  setMotor(FL_IN1, FL_IN2, CH_FL, fl);
  setMotor(FR_IN1, FR_IN2, CH_FR, fr);
  setMotor(RL_IN1, RL_IN2, CH_RL, rl);
  setMotor(RR_IN1, RR_IN2, CH_RR, rr);
}

void setMotor(int pinIN1, int pinIN2, int canalPWM, int velocidad) {
  if (velocidad >= 0) {
    digitalWrite(pinIN2, LOW);
    ledcWrite(canalPWM, velocidad);
  } else {
    digitalWrite(pinIN2, HIGH);
    ledcWrite(canalPWM, 255 + velocidad); // velocidad es negativo aquí
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
