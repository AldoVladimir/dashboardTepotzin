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
     * Reinicia las mediciones de RF para iniciar una nueva b�squeda
     */

    RF_THLD_NORM = pow(10, (float)(RF_THLD_dB/10.0));       // Umbral normalizado de RF

    uint8_t ch;
    for(ch = 0; ch < NumCh; ch++){
        // -> Mediciones rotacionales
        Mx[ch] = 0;
        My[ch] = 0;
        Mmag[ch] = 0;

        // -> Mediciones est�ticas
        Mstatic[ch] = 0;
    }
}

float MeasureRSSI(enum Ch_type ch){
    /*
     * Esta funci�n realiza NUM_MEDS mediciones de RSSI en el canal especificado
     * con el par�metro @ch. Despu�s obtiene la mediana de dichos valores.
     */

    uint8_t i;
    float M_part[NUM_MEDS];

    for(i=0; i<NUM_MEDS; i++){
        M_part[i] = (float)senseRSSI(ch);            // Medici�n de RSSI en el canal 'ch' iterado
    }

    return GetMedian(M_part, NUM_MEDS);
}


// -------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ RF_Rx Rotacional -------------------------------------------------

void DoRFMeasureInAngle(float ang_rad){

    /*
     * Esta funci�n realiza una medici�n de RSSI en todos los canales de b�squeda especificados
     * por el arreglo global @chs_to_search. Calcula las componentes del vector de direcci�n,
     * conociendo previamente el �ngulo @ang_rad en el que se encuentra.
     */

    float m = 0;
    uint8_t i;

    for(i=0; i<NumCh; i++){                         // Por cada canal del 1 al 5
        m = MeasureRSSI(chs_to_search[i]);          // Obtiene una medici�n de RSSI en el canal iterado
        m = pow(10, (float)(m/10.0));               // Se obtiene el valor de la medici�n en unidades de intensidad RSSI
        Mx[i] += m*cos(ang_rad);                    // Se obtiene la componente en X del vector
        My[i] += m*sin(ang_rad);                    // Se obtiene la componente en Y del vector
    }
}

bool EvaluateMeasurements(){
    /*
     * Esta funci�n eval�a si al menos uno de los canales produjo un vector de direcci�n con magnitud
     * superior al umbral RF_THLD_NORM. Si es as�, se retorna "true" lo que indica que las mediciones
     * fueron correctas. De otra forma retorna "false" que indica que ninguna medici�n super� el umbral.
     * El uso de esta funci�n es obligatorio antes de usar la funci�n GetDirVector(), debido a que esta
     * funci�n calcula la magnitud del vector resultante para cada canal.
     */

    uint8_t ch;
    for(ch=0; ch<NumCh; ch++) {                 // Itera sobre los canales 1 a 5
        Mmag[ch] = sqrt(Mx[ch]*Mx[ch] + My[ch]*My[ch]);     // Obtiene la magnitud de los vectores resultantes en cada canal, normalizados
        if(Mmag[ch] > RF_THLD_NORM){            // Si la magnitud normalizada es mayor que el umbral de RF normalizado
            return STATE_SUCCESS;               // Si al menos un canal supera el umbral, la medici�n fue correcta
        }
    }
    return STATE_FAILED;                        // Si ninguna medici�n supero el umbral, retorna STATE_FAILED
}

void GetDirVector(float* dv){

    /*
     * Esta funci�n itera sobre los vectores de cada canal, los pondera con el arreglo de pesos
     * y selecciona el que tenga la mayor magnitud. Recibe un apuntador a un arreglo de 2 elementos
     * y los llena con la magnitud [�ndice 0] y el �ngulo [�ndice 1] del vector obtenido.
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

    if(aux_ang >= pi)                           // Si el �ngulo es mayor que 180�
        aux_ang -= 2*pi;                        // Gira hacia el otro lado

    dv[1] = aux_ang;
}

bool IsOtherRobotActivated(){

    /*
     * Esta funci�n mide la se�al en todos los canales y verifica
     * si alguna es mayor que el umbral de ruido RF_THLD_dB. Lo que
     * indicar�a que alg�n otro robot est� activado y ya puede comenzar
     * la secuencia de navegaci�n.
     */

    uint8_t i = 0;
    uint16_t n = 0, Nt = 200;
    float m = 0;

    for(n=0; n<Nt; n++){                                // Se realizar� la b�squeda Nt veces de suerte que Tb�squeda >= 2*Temisi�n
        for(i=0; i<NumCh; i++){                         // Por cada canal del 1 al 5
            m = MeasureRSSI(chs_to_search[i]);          // Obtiene una medici�n de RSSI en el canal iterado
            if(m > RF_THLD_dB){                         // Si la medici�n en dB es menor que el umbral de ruido de RF en dB
                return true;                            // Si la medici�n supera el umbral, significa que s� hay otro robot encendido
            }
        }
    }

    return false;                                       // Si ning�n canal arroja una medici�n superior al umbral, no hay otro robot encendido
}


// -------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- RF_Rx Est�tico --------------------------------------------------

void DoRFMeasureStatic(){
    /*
     * Esta funci�n realiza una medici�n de RSSI en todos los canales de b�squeda especificados
     * por el arreglo global @chs_to_search. Guarda la magnitud de la medici�n en el arreglo @Mstatic.
     */

    uint8_t i;

    for(i=0; i<NumCh; i++){                         // Por cada canal del 1 al 5
//        m = MeasureRSSI(chs_to_search[i]);          // Obtiene una medici�n de RSSI en el canal iterado
//        Mstatic[i] = pow(10, (float)(m/10.0));      // Se obtiene el valor de la medici�n en unidades de intensidad RSSI

        Mstatic[i] = MeasureRSSI(chs_to_search[i]);         // Obtiene una medici�n de RSSI en el canal iterado
        usleep(200000);
    }
}

uint8_t EvaluateStaticMeasurements(){
    /*
     * Esta funci�n eval�a si al menos uno de los canales produjo un vector de direcci�n con magnitud
     * superior al umbral RF_THLD_NORM en una medici�n r�pida. Si es as�, se vuelven a realizar m�s
     * mediciones en el canal correspondiente al mayor cluster detectado para saber si son correctas
     * o falsas detecciones.
     * En caso de que se haya tenido una detecci�n correcta, se retorna el tama�o
     * de cluster correspondiente al canal detectado.
     * Si ninguna de las mediciones report� detecci�n en los canales buscados, se retorna un 0.
     */

    uint8_t ch, i;
    uint8_t Det_Chs[5] = {0, 0, 0, 0, 0};

    for(ch=0; ch<NumCh; ch++) {                         // Itera sobre los canales 1 a 5
        if(Mstatic[ch] > RF_THLD_dB){                   // Si el RSSI en dB de la medici�n es mayor que el umbral de RF en dBs
            Det_Chs[ch] = 1;                            // Se indica que se detect� el canal
        }
    }

    uint8_t m_count, cluster_found = 0;
    float pct_measures;

    for(ch=NumCh-1; ch==0; ch--){                       // Se iteran las mediciones de los canales mayor al menor
        if(Det_Chs[ch]){                                // Si el canal iterado se hab�a detectado
            m_count = 0;
            for(i=0; i<NUM_ST_VERIF_MEAS; i++){         // Se vuelven a realizar NUM_ST_VERIF_MEAS mediciones en ese mismo canal
                if(MeasureRSSI(chs_to_search[ch]) > RF_THLD_dB){    // En cada medici�n se verifica si es mayor que el umbral
                    m_count++;                          // En caso afirmativo, se incrementa la variable m_count
                }
                usleep(50000);                          // Cada medici�n se realiza con 50ms de diferencia
            }
            pct_measures = ((float)m_count)/NUM_ST_VERIF_MEAS;  // Se calcula el porcentaje de �xito de las mediciones realizadas
            if(pct_measures >= PCT_LVL_FOR_SUCCESS){    // Si dicho porcentaje es mayor que el necesario para declarar un canal activo
                cluster_found = ch+1;                   // Se guarda el tama�o de cluster correspondiente al canal encontrado
                break;                                  // Se termina el ciclo. Como se est�n iterando en sentido descendente, los canales mayores tienen mayor prioridad que los inferiores.
            }
        }
    }

    return cluster_found;                               // Retorna el cluster econtrado, si no se encontr� ninguno, retorna 0
}


// ------------------------------------------------------ RF_Tx ------------------------------------------------------

void SetToneInChannel(enum Ch_type ch){
    // Inicia a transmitir un tono a la frecuencia del canal "ch"
    ChangeRadioMode(TX, ch);
}


// ------------------------------------------------------ Utils ------------------------------------------------------

enum Ch_type CS_to_CHtype(uint8_t cs){

    /*
     * Esta funci�n convierte el tama�o de cluster en el que se
     * encuentra el robot @cs, por el canal en el que se transmitir�
     * la se�al de emisi�n. Se retorna como valor dicho par�metro.
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
