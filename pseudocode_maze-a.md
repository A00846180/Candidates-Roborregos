# Pseudocódigo pista A: Maze

// Representación del laberinto 5 x 5

CELDA_VISITADA[5] [5] = todo FALSO
CAMINO = []          // para ir hacia atrás y para el Bonus 1
POS_ACTUAL = (fila_inicio, col_inicio)
ORIENTACION_ACTUAL = NORTE/SUR/ESTE/OESTE
COLORES_DETECTADOS = []

//Algoritmo principal

FUNCION explorar_laberinto():
    CAMINO.push(POS_ACTUAL)
    CELDA_VISITADA[POS_ACTUAL] = VERDADERO

    MIENTRAS tiempo_restante > margen_seguridad:

        detectar_color_en_celda_actual()

        vecinas_disponibles = escanear_paredes()  // aquí con el ultrasónico y el infrarrojo
        vecinas_no_visitadas = filtrar(vecinas_disponibles, CELDA_VISITADA == FALSO)

        SI vecinas_no_visitadas NO está vacío ENTONCES
            siguiente = elegir_prioridad(vecinas_no_visitadas)
            mover_hacia(siguiente)
            POS_ACTUAL = siguiente
            CELDA_VISITADA[POS_ACTUAL] = VERDADERO
            CAMINO.push(POS_ACTUAL)

        SINO SI POS_ACTUAL == CASILLA_FINAL_ROJA ENTONCES
            romper ciclo  // laberinto resuelto

        SINO
            // backtrack: no hay hacia dónde avanzar desde aquí
            SI CAMINO.size() <= 1 ENTONCES
                romper ciclo  // ya no hay nada más que explorar
            FIN SI
            CAMINO.pop()
            anterior = CAMINO.top()
            mover_hacia(anterior)
            POS_ACTUAL = anterior
        FIN SI

    FIN MIENTRAS

    SI llegó_a_casilla_final ENTONCES
        mostrar_en_oled("Meta alcanzada")
    FIN SI

// Para escanear paredes

FUNCION escanear_paredes():
    distancias = {}
    distancias[FRENTE] = leer_HC_SR04()
    distancias[IZQUIERDA] = leer_IR_izquierdo()     //IR es infrarrojo
    distancias[DERECHA] = leer_IR_derecho()

    disponibles = []
    PARA CADA direccion EN [FRENTE, IZQUIERDA, DERECHA]:
        SI distancias[direccion] > UMBRAL_UNIDAD_LIBRE ENTONCES
            direccion_absoluta = convertir_relativa_a_absoluta(direccion, ORIENTACION_ACTUAL)
            SI direccion_absoluta no sale del grid 5x5 ENTONCES
                disponibles.push(direccion_absoluta)
            FIN SI
        FIN SI
    FIN PARA

    RETORNAR disponibles

    // Movimiento del robot

    FUNCION mover_hacia(celda_destino):
    direccion_absoluta = calcular_direccion(POS_ACTUAL, celda_destino)

    SI direccion_absoluta == ORIENTACION_ACTUAL ENTONCES
        avanzar_una_unidad()
    SINO SI direccion_absoluta == opuesta(ORIENTACION_ACTUAL) ENTONCES
        retroceder_una_unidad()
    SINO SI es_lateral(direccion_absoluta, ORIENTACION_ACTUAL) ENTONCES
        strafe_lateral(direccion_absoluta)  //strafe es movimiento lateral, como el de cangrejo
    SINO
        girar_90_grados(direccion_absoluta)
        avanzar_una_unidad()
        ORIENTACION_ACTUAL = direccion_absoluta
    FIN SI

    detectar_y_manejar_obstaculo()   // por si hay speedbump/escalera/rampa en el tramo
    registrar_movimiento(CAMINO)  // para Bonus 1

    // Para detección de color

    FUNCION detectar_color_en_celda_actual():
    lectura = leer_TCS34725()  // promedia varias muestras, sensor mirando al piso

    color = clasificar_color(lectura)  // compara contra los 4 valores RGB de referencia calibrados

    SI color != NINGUNO Y color no está en COLORES_DETECTADOS ENTONCES
        COLORES_DETECTADOS.push(color)
        mostrar_en_oled(color)
        mantener_display(3 segundos)  // para que de tiempo de verlo bien
    FIN SI

    //Para Bonus 1

    FUNCION ejecutar_bonus_regreso():
    camino_inverso = invertir(PILA_CAMINO)
    PARA CADA paso EN camino_inverso (excluyendo el primero):
        mover_hacia(paso)
    FIN PARA

    FIN