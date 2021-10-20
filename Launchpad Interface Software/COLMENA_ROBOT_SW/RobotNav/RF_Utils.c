#include <RobotNav/RF_Utils.h>

enum Ch_type chs_to_search[5] = {CH1, CH2, CH3, CH4, CH5};   // Lista de canales a buscar
int Wd[NumCh] = {1, 2, 3, 4, 5};                        // Pesos para cada canal
float RF_THLD_NORM = 0;                                 // Umbral normalizado

// --------- Rotational Search ---------
float Mx[NumCh] = {};                                   // Guarda las componentes en x de los vectores parciales generados
float My[NumCh] = {};                                   // Guarda las componentes en y de los vectores parciales generados
float Mmag[NumCh] = {};                                 // Arreglo que guarda las magnitudes normalizadas de la mediciones (0 es lo peor, 1 es lo mejor)

// ----------- Static Search -----------
int Mstatic[NumCh] = {};



void ResetRFMeasurements(){
    /*
     * Reinicia las mediciones de RF para iniciar una nueva búsqueda
     */

    RF_THLD_NORM = pow(10, (float)(RF_THLD_dB/10.0));       // Umbral normalizado de RF

    uint8_t ch;
    for(ch = 0; ch < NumCh; ch++){
        // -> Mediciones rotacionales
        Mx[ch] = 0;
        My[ch] = 0;
        Mmag[ch] = 0;

        // -> Mediciones estáticas
        Mstatic[ch] = 0;
    }
}

float MeasureRSSI(enum Ch_type ch){
    /*
     * Esta función realiza NUM_MEDS mediciones de RSSI en el canal especificado
     * con el parámetro @ch. Después obtiene la mediana de dichos valores.
     */

    uint8_t i;
    float M_part[NUM_MEDS];

    for(i=0; i<NUM_MEDS; i++){
        M_part[i] = (float)senseRSSI(ch);            // Medición de RSSI en el canal 'ch' iterado
    }

    return GetMedian(M_part, NUM_MEDS);
}


// -------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ RF_Rx Rotacional -------------------------------------------------

void DoRFMeasureInAngle(float ang_rad){

    /*
     * Esta función realiza una medición de RSSI en todos los canales de búsqueda especificados
     * por el arreglo global @chs_to_search. Calcula las componentes del vector de dirección,
     * conociendo previamente el ángulo @ang_rad en el que se encuentra.
     */

    float m = 0;
    uint8_t i;

    for(i=0; i<NumCh; i++){                         // Por cada canal del 1 al 5
        m = MeasureRSSI(chs_to_search[i]);          // Obtiene una medición de RSSI en el canal iterado
        m = pow(10, (float)(m/10.0));               // Se obtiene el valor de la medición en unidades de intensidad RSSI
        Mx[i] += m*cos(ang_rad);                    // Se obtiene la componente en X del vector
        My[i] += m*sin(ang_rad);                    // Se obtiene la componente en Y del vector
    }
}

bool EvaluateMeasurements(){
    /*
     * Esta función evalúa si al menos uno de los canales produjo un vector de dirección con magnitud
     * superior al umbral RF_THLD_NORM. Si es así, se retorna "true" lo que indica que las mediciones
     * fueron correctas. De otra forma retorna "false" que indica que ninguna medición superó el umbral.
     * El uso de esta función es obligatorio antes de usar la función GetDirVector(), debido a que esta
     * función calcula la magnitud del vector resultante para cada canal.
     */

    uint8_t ch;
    for(ch=0; ch<NumCh; ch++) {                 // Itera sobre los canales 1 a 5
        Mmag[ch] = sqrt(Mx[ch]*Mx[ch] + My[ch]*My[ch]);     // Obtiene la magnitud de los vectores resultantes en cada canal, normalizados
        if(Mmag[ch] > RF_THLD_NORM){            // Si la magnitud normalizada es mayor que el umbral de RF normalizado
            return STATE_SUCCESS;               // Si al menos un canal supera el umbral, la medición fue correcta
        }
    }
    return STATE_FAILED;                        // Si ninguna medición supero el umbral, retorna STATE_FAILED
}

void GetDirVector(float* dv){

    /*
     * Esta función itera sobre los vectores de cada canal, los pondera con el arreglo de pesos
     * y selecciona el que tenga la mayor magnitud. Recibe un apuntador a un arreglo de 2 elementos
     * y los llena con la magnitud [índice 0] y el ángulo [índice 1] del vector obtenido.
     */

    float maxMag = 0;
    float aux_ang = 0;
    uint8_t maxCh = 255;
    uint8_t ch;

    for(ch = 0; ch < NumCh; ch++){
        Mmag[ch] = Mmag[ch]*Wd[ch];
        if(Mmag[ch] > maxMag){
            maxMag = Mmag[ch];
            maxCh = ch;
        }
    }

    dv[0] = maxMag;
    aux_ang = atan(My[maxCh]/Mx[maxCh]);

    if(aux_ang >= pi)                           // Si el ángulo es mayor que 180°
        aux_ang -= 2*pi;                        // Gira hacia el otro lado

    dv[1] = aux_ang;
}

bool IsOtherRobotActivated(){

    /*
     * Esta función mide la señal en todos los canales y verifica
     * si alguna es mayor que el umbral de ruido RF_THLD_dB. Lo que
     * indicaría que algún otro robot está activado y ya puede comenzar
     * la secuencia de navegación.
     */

    uint8_t i = 0;
    uint16_t n = 0, Nt = 200;
    float m = 0;

    for(n=0; n<Nt; n++){                                // Se realizará la búsqueda Nt veces de suerte que Tbúsqueda >= 2*Temisión
        for(i=0; i<NumCh; i++){                         // Por cada canal del 1 al 5
            m = MeasureRSSI(chs_to_search[i]);          // Obtiene una medición de RSSI en el canal iterado
            if(m > RF_THLD_dB){                         // Si la medición en dB es menor que el umbral de ruido de RF en dB
                return true;                            // Si la medición supera el umbral, significa que sí hay otro robot encendido
            }
        }
    }

    return false;                                       // Si ningún canal arroja una medición superior al umbral, no hay otro robot encendido
}


// -------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- RF_Rx Estático --------------------------------------------------

void DoRFMeasureStatic(){
    /*
     * Esta función realiza una medición de RSSI en todos los canales de búsqueda especificados
     * por el arreglo global @chs_to_search. Guarda la magnitud de la medición en el arreglo @Mstatic.
     */

    uint8_t i;

    for(i=0; i<NumCh; i++){                         // Por cada canal del 1 al 5
//        m = MeasureRSSI(chs_to_search[i]);          // Obtiene una medición de RSSI en el canal iterado
//        Mstatic[i] = pow(10, (float)(m/10.0));      // Se obtiene el valor de la medición en unidades de intensidad RSSI

        Mstatic[i] = MeasureRSSI(chs_to_search[i]);         // Obtiene una medición de RSSI en el canal iterado
        usleep(200000);
    }
}

uint8_t EvaluateStaticMeasurements(){
    /*
     * Esta función evalúa si al menos uno de los canales produjo un vector de dirección con magnitud
     * superior al umbral RF_THLD_NORM en una medición rápida. Si es así, se vuelven a realizar más
     * mediciones en el canal correspondiente al mayor cluster detectado para saber si son correctas
     * o falsas detecciones.
     * En caso de que se haya tenido una detección correcta, se retorna el tamaño
     * de cluster correspondiente al canal detectado.
     * Si ninguna de las mediciones reportó detección en los canales buscados, se retorna un 0.
     */

    uint8_t ch, i;
    uint8_t Det_Chs[5] = {0, 0, 0, 0, 0};

    for(ch=0; ch<NumCh; ch++) {                         // Itera sobre los canales 1 a 5
        if(Mstatic[ch] > RF_THLD_dB){                   // Si el RSSI en dB de la medición es mayor que el umbral de RF en dBs
            Det_Chs[ch] = 1;                            // Se indica que se detectó el canal
        }
    }

    uint8_t m_count, cluster_found = 0;
    float pct_measures;

    for(ch=NumCh-1; ch==0; ch--){                       // Se iteran las mediciones de los canales mayor al menor
        if(Det_Chs[ch]){                                // Si el canal iterado se había detectado
            m_count = 0;
            for(i=0; i<NUM_ST_VERIF_MEAS; i++){         // Se vuelven a realizar NUM_ST_VERIF_MEAS mediciones en ese mismo canal
                if(MeasureRSSI(chs_to_search[ch]) > RF_THLD_dB){    // En cada medición se verifica si es mayor que el umbral
                    m_count++;                          // En caso afirmativo, se incrementa la variable m_count
                }
                usleep(50000);                          // Cada medición se realiza con 50ms de diferencia
            }
            pct_measures = ((float)m_count)/NUM_ST_VERIF_MEAS;  // Se calcula el porcentaje de éxito de las mediciones realizadas
            if(pct_measures >= PCT_LVL_FOR_SUCCESS){    // Si dicho porcentaje es mayor que el necesario para declarar un canal activo
                cluster_found = ch+1;                   // Se guarda el tamaño de cluster correspondiente al canal encontrado
                break;                                  // Se termina el ciclo. Como se están iterando en sentido descendente, los canales mayores tienen mayor prioridad que los inferiores.
            }
        }
    }

    return cluster_found;                               // Retorna el cluster econtrado, si no se encontró ninguno, retorna 0
}


// ------------------------------------------------------ RF_Tx ------------------------------------------------------

void SetToneInChannel(enum Ch_type ch){
    // Inicia a transmitir un tono a la frecuencia del canal "ch"
    ChangeRadioMode(TX, ch);
}


// ------------------------------------------------------ Utils ------------------------------------------------------

enum Ch_type CS_to_CHtype(uint8_t cs){

    /*
     * Esta función convierte el tamaño de cluster en el que se
     * encuentra el robot @cs, por el canal en el que se transmitirá
     * la señal de emisión. Se retorna como valor dicho parámetro.
     */

    enum Ch_type ch;

    switch(cs){
        case 1:
            ch = CH1;
        break;

        case 2:
            ch = CH2;
        break;

        case 3:
            ch = CH3;
        break;

        case 4:
            ch = CH4;
        break;

        case 5:
            ch = CH5;
        break;

        case 6:
            ch = CH6;
        break;

        default:
            ch = CH_none;
    }

    return ch;
}
