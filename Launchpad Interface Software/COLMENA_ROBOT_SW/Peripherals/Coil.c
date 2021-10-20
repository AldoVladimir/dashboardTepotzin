#include <Peripherals/Coil.h>

// ------------ Variables globales de cálculo de PSD con DFT Goertzel ------------
// uint16_t k = 0, n = 0;
//float w, cosw;
//float pi_N = pi/N;
//float s[] = {0, 0, 0};
float PSD[N/2] = {};                        // Vector para almacenar la PSD (Densidad de Potencia Espectral) de la señal muestreada. Tamaño N/2 debido al algoritmo Cooley y Tukey
float voltBuffer[N] = {};        // Guarda las muestras realizadas de la bobina Rx convertidas a volts en punto flotante

uint16_t sampleBuffer_Sci[SCI_N];           // Buffer de muestras para la misión científica

float Coil_Noise_Baseline, CoilRx_Thld;
uint8_t CoilRx_Polls_counter;

// ---------- Variables globales de conversión ADC y número de sondeos -----------
bool samples_ready = false;
ADCBuf_Handle coilRx_adc;
ADCBuf_Params adcParams;
ADCBuf_Conversion conv;

bool calib_complete = false;                    // Bandera de calibración completa

uint8_t BandsDetected[13] = {};     // Arreglo que guarda las bandas de frecuencia detectadas con la bobina. 13 bandas en total

uint8_t BinsToDetect_Index[13] = {107,114,121,128,136,144,152,158,165,174,181,189,197};
//uint8_t BinsToDetect_Index[13] = {54,57,61,64,68,72,76,79,83,87,91,95,99};      // Bines centrales


// ------------------------------------------------------ COIL RX ------------------------------------------------------

void Init_CoilRx(){
    /*
     * Esta función inicializa el driver ADCBuf que permite tomar un Buffer de N mediciones
     * en el canal ADC0 que muestrea la señal inducida en la bobina receptora.
     * Se configura una frecuencia de muestreo fija de 52kHz
     */
    ADCBuf_init();                                          // Inicializa biblioteca ADCBuf
    ADCBuf_Params_init(&adcParams);                         // Inicializa los parámetros del ADC

    adcParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_ONE_SHOT; // Modo de un único disparo
    adcParams.returnMode = ADCBuf_RETURN_MODE_BLOCKING;     // Modo de bloqueo
    adcParams.samplingFrequency = Fs;                       // Frecuencia de muestreo 52kHz

    conv.arg = NULL;
    conv.adcChannel = Board_ADCBUF0CHANNEL0;                // Indica el canal de conversión
    conv.samplesRequestedCount = N;                         // Tamaño de cada Buffer
}

void ConvertToVolts(void* sampleBuffer, float *target_vect){

    /* Cálculo de datos ajustados a Volts en punto flotante
     * ----------------------------------------------------------------------
     * adcValue = sampleBuffer[i]
     * refValue = 4300000
     * valueVolts = adcValue*(refValue/4095e6) + 2047/4095e6 = adcValue*A + B
     *  A = (4300000/4095e6) = 0.00105006105
     *  B = (2047/4095e6) = 0.0000004998779
    */

    float MeanVoltBuffer = 0;

    uint16_t i = 0;
    for (i = 0; i < N; i++) {
        target_vect[i] = (float)(((int16_t *)sampleBuffer)[i]*0.00105006105 + 0.0000004998779);
        MeanVoltBuffer += target_vect[i];
    }

    MeanVoltBuffer = MeanVoltBuffer/N;

    for (i = 0; i < N; i++) {
        target_vect[i] -= MeanVoltBuffer;
    }
}

void ResetCoilMeasurements(){
    /*
     * Reinicia todos los valores del arreglo PSD a 0
     */

    uint16_t k = 0;
    for(k=0; k<N; k++){
        PSD[k] = 0;
    }
}

void CalcMeanPSD(uint8_t polls){
    /*
     * Obtiene la media punto a punto de la PSD
     */

    uint16_t k = 0;
    for(k=0; k<N/2; k++){
        PSD[k] = PSD[k]/((float)polls);
    }
}


/*
 * TODO: Modificar el tipo de dato de @sampleBuffer a float
 * para precsindir del buffer @voltBuffer y así ahorrar un poco
 * de memoria en el proceso.
 */
int16_t sampleBuffer[N];                    // Buffer 1 de muestras tomadas con el ADC

void CoilRxMeasurement(struct NavState *NavS){
    /*
     * Esta función manda a realizar las mediciones con la bobina Rx y después realiza
     * el postproceso de los datos (DFT_Goertzel, obtiene mediana, etc.).
     * Una vez terminado el proceso llena el arreglo de las bandas detectadas "BandsDetected"
     * - Retorna: uint8_t con el tamaño de cluster en el que se encuentra el robot [1, 6]
     */

    //int16_t sampleBuffer[N];                    // Buffer 1 de muestras tomadas con el ADC

    // ------------------ Configura la estructura del ADC y la conversión ------------------
    adcParams.samplingFrequency = Fs;                           // Frecuencia de muestreo pasada como parámetro
    conv.sampleBuffer = sampleBuffer;                           // Indica el buffer de conversiones
    conv.samplesRequestedCount = N;                             // Tamaño del buffer de mediciones de misión científica

    uint8_t np = 0;                                             // Contador de sondeos realizados
    uint16_t i = 0;

    ResetCoilMeasurements();                                    // Reinicia el vector del PSD de la bobina
    samples_ready = false;                                      // Baja la bandera de muestras listas
    coilRx_adc = ADCBuf_open(Board_ADCBUF0, &adcParams);        // Abre el puerto ADC0 con los parámetros indicados

    do{
        ADCBuf_convert(coilRx_adc, &conv, 1);                   // Inicia el sondeo, al completar N muestras entra a la función de interrupción
        ADCBuf_adjustRawValues(coilRx_adc, sampleBuffer, N, Board_ADCBUF0);    // Ajusta valores a formato legible
        ConvertToVolts(sampleBuffer, voltBuffer);               // Convierte datos del buffer a Volts en punto flotante

        ApplyWindow(voltBuffer, NUTTAL, N);                     // Se le aplica una ventana Nuttal
        fft_CooleyTukey(voltBuffer, PSD, N, HALF_LEN);          // FFT de la señal muestreada

        np++;                                                   // Se incrementa el contador de sondeos

    } while(np < MAX_DET_ITER);                                 // Mientras no se hayan completado las iteraciones de detección

    ADCBuf_close(coilRx_adc);                                   // Una vez terminado el muestreo, se cierra el canal de ADC

    CalcMeanPSD(MAX_DET_ITER);                                  // Se obtiene la media del PSD

    for(i=0; i<N/2; i++){
        voltBuffer[i] = PSD[i];                                 // Se copian los datos del PSD, se toma como buffer auxiliar a voltBuffer. Esto es para ahorrar memoria
    }
    Coil_Noise_Baseline = GetMedian(voltBuffer, N/2);           // Obtiene la mediana, que será la linea de base del ruido en la bobina
    CoilRx_Thld = 30*Coil_Noise_Baseline;                       // El umbral de detección de la bobina Tx será 30 veces el ruido de fondo

    CheckBandsDetected();                                       // Verifica las bandas detectadas

    uint8_t CS_aux = 0;
    CS_aux = GetClusterSize();                                  // Obtiene el tamaño de cluster en el que se encuentra el robot o CLUST_DESTRUCTION

    if(CS_aux == CLUST_DESTRUCTION && NavS->ACS != 1){          // Si se obtuvo CLUST_DESTRUCTION y el robot no estaba solo
        NavS->DestroyCluster = true;                            // Bandera que indica que se destruye el cluster
        NavS->LCS = NavS->ACS;                                  // Actualiza LCS
        NavS->ACS = 1;                                          // Como se destruye el cluster, entonces ACS = 1
        NavS->ClusterChanged = true;                            // Indica que el cluster cambió
    }
    else{
        NavS->DestroyCluster = false;                           // Se limpia la bandera de destrucción de cluster
        if(CS_aux != NavS->ACS){                                // Verifica si el CS cambió (un robot se conectó o se desconectó)
            NavS->LCS = NavS->ACS;                              // Se guarda el último tamaño de cluster conocido
            NavS->ACS = CS_aux;                                 // Se guarda el nuevo tamaño de cluster detectado
            NavS->ClusterChanged = true;                        // Indica que el cluster cambió de tamaño (el robot ya no está solo)
        }
        else{
            NavS->ClusterChanged = false;                       // El cluster no cambió. La búsqueda no arrojó ningún resultado
        }
    }
}

void CheckBandsDetected(){

    /*
     * Esta función verifica cada una de las bandas dedicadas a la detección de
     * clusters y llena el vector BandsDetected con 1 o 0, dependiendo de si se
     * detectó la banda o no. Llama a la función
     */

    uint8_t i = 0;

    float PSD_of_Band = 0;                                  // Variable que guarda el PSD total obtenido en cada banda
    uint8_t Bin_Index = 0;                                  // Obtiene el índice del bin en el cual buscar

    for(i=0; i<13; i++){                                    // Itera sobre las 13 bandas
        Bin_Index = BinsToDetect_Index[i];                  // Obtiene el índice del bin a buscar (según las bandas seleccionadas)
        PSD_of_Band = PSD[Bin_Index - 1];                   // Cada banda tiene un ancho de 3 bines ...
        PSD_of_Band += PSD[Bin_Index];                      // ... por lo que suma las contribuciones...
        PSD_of_Band += PSD[Bin_Index + 1];                  // ... de cada uno de ellos.

        if((PSD_of_Band - Coil_Noise_Baseline) > CoilRx_Thld){   // Si el PSD de la banda elegida es mayor que la línea de base
            BandsDetected[i] = 1;                           // Se detectó la banda, por lo que se marca como 1 en el arreglo de bandas detectadas
        }
        else{                                               // Si no:
            BandsDetected[i] = 0;                           // No se detectó la banda y se marca como un 0
        }
    }
}

uint8_t GetClusterSize(){

    /*
     * Esta función obtiene el tamaño de cluster en el que se encuentra el robot
     * dependiendo de las bandas que haya detectado en el proceso de adquisición.
     * Si se detectó la banda 13 (Banda de destrucción), devuelve la bandera @CLUST_DESTRUCTION
     */

    uint8_t i = 0;
    uint8_t ClusterSize = 1;                // El cluster siempre inicia en tamaño 1 (él mismo)

    if(BandsDetected[12]){                  // Si se detectó la banda 13
        return CLUST_DESTRUCTION;           // Significa que se va a destruír el cluster
    }

    for(i=0; i<6; i++){                                     // Se itera sobre las bandas públicas
        if(BandsDetected[i] && ((i+1) != ROBOT_ID)){        // Si se detectó la banda y no corresponde al ID de este robot:
            ClusterSize++;                                  // Se incrementa el tamaño de cluster
        }
    }

    return ClusterSize;                                     // Se retorna el tamaño de cluster
}

uint16_t * CoilRxMeasurement_ScienceMission(Sci_Fs fs){

    /*
     * Esta función reconfigura el ADCBuf de la bobina Rx para modificar
     * la frecuencia de muestreo según @fs. Inicia el muestreo y retorna el
     * apuntador al vector con las mediciones terminadas en datos crudos.
     */

    adcParams.callbackFxn = CoilRx_SamplesReady_SciMission; // Rutina de interrupción de Buffer completo para misión científica
    adcParams.samplingFrequency = (uint32_t)fs;             // Frecuencia de muestreo pasada como parámetro
    conv.sampleBuffer = sampleBuffer_Sci;                   // Indica el buffer de conversiones
    conv.samplesRequestedCount = SCI_N;                     // Tamaño del buffer de mediciones de misión científica

    coilRx_adc = ADCBuf_open(Board_ADCBUF0, &adcParams);    // Abre el puerto ADC0 con los parámetros indicados
    ADCBuf_convert(coilRx_adc, &conv, 1);                   // Inicia el sondeo, al completar N muestras entra a la función de interrupción
    while(!samples_ready);                                  // Espera a que la conversión se haya completado

    return sampleBuffer_Sci;
}

void CoilRx_SamplesReady_SciMission(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel){

    samples_ready = true;                   // Indica que el muestreo y conversión está listo
    ADCBuf_close(coilRx_adc);               // Una vez terminado el muestreo, se cierra el canal de ADC
}



// ---------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ COIL TX ------------------------------------------------------

// -------------- COEFICIENTES SEÑALES SENO --------------
uint8_t c1[] =  {20,99,30}, c2[] =  {21,95,31}, c3[] =  {22,91,31};
uint8_t c4[] =  {23,87,31}, c5[] =  {24,82,30}, c6[] =  {25,77,30};
uint8_t c7[] =  {26,72,29}, c8[] =  {27,67,28}, c9[] =  {27,62,26};
uint8_t c10[] = {28,56,24}, c11[] = {29,51,23}, c12[] = {29,45,20};
uint8_t c13[] = {30,39,18};

extern DigOscilator osci_h[13] = {0};       // Vector de estructuras DigOscilator para manejo en función ensamblador

// ---------------- Variables globales para GPTimer ----------------
GPTimerCC26XX_Handle hTimer3A;          // Manejador del Timer0 (Maneja la generación de cada dato nuevo con el oscilador)
GPTimerCC26XX_Handle hTimer1A;          // Manejador del Timer1 (Maneja el cambio de señales en secuencia)
SPI_Handle DAC_AD7303;                  // Manejador del puerto SPI0 con el DAC

uint8_t CoilTx_ActiveSignals = 6;           // Número de señales activas
uint8_t SignalToTransmit = 0;               // Señal a transmitir en la secuencia

#if ROBOT_ID == 1
    CoilFreq CoilTx_freqList[7] = {f1, none, none, none, none, none, none};
    // CoilFreq CoilTx_freqList[7] = {f1, f2, f6, f9, f10, f13, none};
#elif ROBOT_ID == 2
    CoilFreq CoilTx_freqList[7] = {f2, f8, none, none, none, none, none};
#elif ROBOT_ID == 3
    CoilFreq CoilTx_freqList[7] = {f3, f9, none, none, none, none, none};
#elif ROBOT_ID == 4
    CoilFreq CoilTx_freqList[7] = {f4, f10, none, none, none, none, none};
#elif ROBOT_ID == 5
    CoilFreq CoilTx_freqList[7] = {f5, f11, none, none, none, none, none};
#elif ROBOT_ID == 6
    CoilFreq CoilTx_freqList[7] = {f6, f12, none, none, none, none, none};
#endif


void Init_CoilTx(){

    /*
     * Esta función inicializa el GPTimer0 para la emisión de la señal con un valor de 10us.
     * Inicializa el SPI para la comunicación con el DAC_AD7303.
     */

    // ---------------- Configuración de Timer3A para emisión de la señal ----------------
    GPTimerCC26XX_Params paramsGPT3A;                                   // Parámetros para el GPTimer a usar
    GPTimerCC26XX_Params_init(&paramsGPT3A);                            // Inicializa los parámetros por defecto
    paramsGPT3A.width          = GPT_CONFIG_16BIT;                      // Timer de 16bits
    paramsGPT3A.mode           = GPT_MODE_PERIODIC;                     // Modo periódico
    paramsGPT3A.direction      = GPTimerCC26XX_DIRECTION_UP;            // Cuenta hacia arriba
    paramsGPT3A.debugStallMode = GPTimerCC26XX_DEBUG_STALL_ON;          // No se detiene en el modo debug
    hTimer3A = GPTimerCC26XX_open(Board_GPTIMER3A, &paramsGPT3A);       // Instancía un objeto timer con los parámetros dados. Timer3A
    GPTimerCC26XX_Value loadVal_T3A = 480;                              // Establece valor al timer. 10us = 480/48MHz
    GPTimerCC26XX_setLoadValue(hTimer3A, loadVal_T3A);                  // Carga el valor al timer
    GPTimerCC26XX_registerInterrupt(hTimer3A, SetValueDAC_timer3ACallback, GPT_INT_TIMEOUT); // Establece el tipo y rutina de interrupción. Por timeout

    // ------------ Configuración de Timer1A para el cambio en la secuencia  -------------
    GPTimerCC26XX_Params paramsGPT1A;                                   // Parámetros para el GPTimer a usar
    GPTimerCC26XX_Params_init(&paramsGPT1A);                            // Inicializa los parámetros por defecto
    paramsGPT1A.width          = GPT_CONFIG_16BIT;                      // Timer de 16bits
    paramsGPT1A.mode           = GPT_MODE_PERIODIC;                     // Modo periódico
    paramsGPT1A.direction      = GPTimerCC26XX_DIRECTION_UP;            // Cuenta hacia arriba
    paramsGPT1A.debugStallMode = GPTimerCC26XX_DEBUG_STALL_ON;          // No se detiene en el modo debug
    hTimer1A = GPTimerCC26XX_open(Board_GPTIMER1A, &paramsGPT1A);       // Instancía un objeto timer con los parámetros dados. Timer1A
    GPTimerCC26XX_Value loadVal_T1A = 480000;                           // Establece valor al timer. t_Tx = 10ms = 480000/48MHz
    GPTimerCC26XX_setLoadValue(hTimer1A, loadVal_T1A);                  // Carga el valor al timer
    GPTimerCC26XX_registerInterrupt(hTimer1A, ChangeSignal_timer1ACallback, GPT_INT_TIMEOUT); // Establece el tipo y rutina de interrupción. Por timeout

    InitSignals();                                                      // Inicializa los manejadores de señales de los osciladores digitales
}

void Configure_CoilTx_SPI(bool opt){

    /*
     * Esta función configura y habilita el puerto SPI0 para la comunicación
     * con el DAC_AD7303. Esta función debe llamarse siempre antes de habilitar
     * la transferencia de datos por SPI.
     */

    if(opt){                                    // Si la opción es true: HABILITAR
        SPI_Params params;                      // Parámetros de configuración del SPI

        SPI_Params_init(&params);               // Inicializa los parámetros del SPI
        params.bitRate     = 12000000;          // Frecuencia del reloj del SPI: 12MHz
        params.frameFormat = SPI_POL0_PHA0;     // Motorola SPI frame format (swcu117i p1489)
        params.mode        = SPI_MASTER;        // SPI en modo maestro
        params.dataSize = (uint32_t)16;         // Cantidad de bits en cada transacción

        DAC_AD7303 = SPI_open(Board_SPI0, &params); // Abre el puerto SPI con la configuración indicada. Instancía un handle de SPI
        if (DAC_AD7303 == NULL) {
            while (1);
        }
        SSIEnable(SSI0_BASE);                   // Habilita el puerto SSI
    }
    else{
        SSIDisable(SSI0_BASE);                  // Habilita el puerto SSI
        SPI_close(DAC_AD7303);                  // Si la opción es false: DESHABILITAR
    }
}

DigOscilator Init_DigOscilator(uint8_t *c){

    /*
     * Esta función inicializa una estructura de tipo DigOscilator con sus respectivos
     * parámetros almacenados en el vector c que se pasa como parámetro.
     */

    DigOscilator dosc;              // Inicializa una variable de tipo Signal_Handler

    dosc.active = 0;                // Indica si la señal está activa o no
    dosc.yn_2 = c[0];               // Inicialmente y0 = b0
    dosc.a1   = c[1];               // Coeficiente a1
    dosc.yn_1 = c[2];               // Inicialmente y1 = a1*y0

    return dosc;                    // Retorna el DigOscilator inicializado
}

void InitSignals(){
    // ---- Inicializa el vector de DigOscilators. Todas las señales comienzan NO activas ----
    osci_h[0] = Init_DigOscilator(c1);        // 10.775 kHz
    osci_h[1] = Init_DigOscilator(c2);        // 11.537 kHz
    osci_h[2] = Init_DigOscilator(c3);        // 12.310 kHz
    osci_h[3] = Init_DigOscilator(c4);        // 13.070 kHz
    osci_h[4] = Init_DigOscilator(c5);        // 13.848 kHz
    osci_h[5] = Init_DigOscilator(c6);        // 14.618 kHz
    osci_h[6] = Init_DigOscilator(c7);        // 15.383 kHz
    osci_h[7] = Init_DigOscilator(c8);        // 16.167 kHz
    osci_h[8] = Init_DigOscilator(c9);        // 16.925 kHz
    osci_h[9] = Init_DigOscilator(c10);       // 17.707 kHz
    osci_h[10] = Init_DigOscilator(c11);      // 18.462 kHz
    osci_h[11] = Init_DigOscilator(c12);      // 19.230 kHz
    osci_h[12] = Init_DigOscilator(c13);      // 20.019 kHz
}

void SetCoilTxSignal(CoilFreq fs){

    /*
     * Esta función establece una señal a la frecuencia deseada "fs" en la bobina transmisora.
     * El resto de frecuencias son desactivadas.
     * Para asignar en el arreglo osci_h se emplea la frecuencia (i-1)ésima debido a que en el
     * enum CoilFreq f1 = 1, f2 = 2, ..., f13 = 13
     */

    int i = 0;
    for(i=f1; i<f13; i++){                      // Itera sobre todas las frecuencias posibles
        osci_h[i-1].active = 0;                 // Las desactiva todas
    }
    osci_h[fs-1].active = 1;                    // Activa solo la frecuencia seleccionada
}

void EnableCoilTxSignal(bool mode){

    /*
     * Esta función habilita el timer que permite al robot generar
     * y transmitir la señal generada al DAC.
     * El orden de activación y desactivación es importante.
     */

    if(mode == true){
        Configure_CoilTx_SPI(true);
        GPTimerCC26XX_start(hTimer1A);    // Arranca el timer que cambia la secuencia de las señales
        GPTimerCC26XX_start(hTimer3A);    // Arranca el timer de generación y transmisión de datos hacia el DAC
    }
    else{
        GPTimerCC26XX_stop(hTimer3A);     // Desactiva ambos timers
        GPTimerCC26XX_stop(hTimer1A);
        Configure_CoilTx_SPI(false);
    }
}

void UpdateCoilTxSignal(){

    /*
     * Esta función usa el arreglo de bandas detectadas a través de la bobina Rx "BandsDetected[]"
     * y modifica el arreglo de bandas a transmitir a través de la bobina Tx.
     */
    int i;
    CoilFreq f;

    for(i=0, f=f1; i<13; i++, f++){
        if(BandsDetected[i]){
            UpdateCoilTxFreqList(ADD_SIGNAL, f);
        }else{
            UpdateCoilTxFreqList(REMOVE_SIGNAL, f);
        }
    }
}

void SetValueDAC_timer3ACallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask){

    /*
     * Cada que se desborda el Timer (10us) entra a la rutina de interrupción para generar
     * un valor nuevo de la señal sumando todos los valores de las señales activas.
     * Este resultado se transmite a través del SPI al DAC. Se empleó la función de bajo
     * nivel implementada en el Driverlib para minimizar el tiempo de transmisión.
     * Además, la función genNewSignalValue_Sine() está escrita en assembly por ser crítica.
     * El código fuente de esta se puede encontrar en el archivo utils.asm
     */

    SSIDataPutNonBlocking(SSI0_BASE, genNewSignalValue_Sine());
}

int Contador_Secuencias = 0;            // TODO: Eliminar esta variable

void ChangeSignal_timer1ACallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask){

    /*
     * Rutina de interrupción del timer1A que permite cambiar la secuencia de las señales
     * transmitidas a través de la bobina Tx. Itera sobre las señales activas en el arreglo
     * CoilTx_freqList.
     */

    SetCoilTxSignal(CoilTx_freqList[SignalToTransmit]);

    SignalToTransmit++;
    if(SignalToTransmit == CoilTx_ActiveSignals){
        SignalToTransmit = 0;
        Contador_Secuencias++;
    }
}

void UpdateCoilTxFreqList(bool mode, CoilFreq fs){

    /*
     * Esta función permite agregar (ADD_SIGNAL) o eliminar (REMOVE_SIGNAL)
     * señales del arreglo CoilTx_freqList, manejando todos los casos para señales
     * duplicadas o inexistentes en el arreglo.
     */

    int i = 0;
    if(mode == ADD_SIGNAL){                                 // Modo de añadir señales
        if(CoilTx_ActiveSignals == 7)                       // Si ya se han añadida las 7 señales máximas a transmitir
            return;                                         // Retorna, no se añade ninguna más
        for(i=0; i<7; i++){                                 // Sino, itera sobre las señales ya presentes en la FreqList
            if(CoilTx_freqList[i] == fs)                    // Si ya está en la lista
                return;                                     // Retorna, no se añde ninguna
        }
        CoilTx_freqList[CoilTx_ActiveSignals] = fs;         // Si no estaba agregada, añade la señal al final de la lista
        CoilTx_ActiveSignals += 1;                          // Incrementa el contador de señales activas
        return;
    }
    if(mode == REMOVE_SIGNAL){                              // Modo remover señales
        for(i=2; i<7; i++){                                 // Busca la señal a remover en las 5 últimas casillas disponibles (0 y 1 no se remueven nunca)
            if(CoilTx_freqList[i] == fs)                    // Si se encuentra la frecuencia en la lista
                break;                                      // Sale del ciclo, la variable "i" guarda la posición del elemento
        }
        if(i == 7)                                          // Si i es 7 es porque recorrió completo el arreglo y no encontró la frecuencia
            return;                                         // En ese caso, retorna, la frecuencia a eliminar no existe
        for(i=i; i<7-1; i++){                               // Elimina el elimento encontrado sobreescibiéndolo con el siguiente en la lista
            CoilTx_freqList[i] = CoilTx_freqList[i + 1];    // Recorre todos los elementos
        }
        CoilTx_freqList[i] = none;                          // El último elemento es "none"
        CoilTx_ActiveSignals -= 1;                          // Se decrementa el contador de señales activas
    }

}




//// ---------------- Utiles cache ----------------
//
///*********************************************************************
//* @fn      SimplePeripheral_disableCache
//*
//* @brief   Disables the instruction cache and sets power constaints
//*          This prevents the device from sleeping while streaming
//*
//* @param   None.
//*
//* @return  None.
//*/
//static void SimplePeripheral_disableCache()
//{
//    //uint_least16_t hwiKey = Hwi_disable();
//    Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
//    Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
//    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_DISABLED, true);
//    //Hwi_restore(hwiKey);
//}
//
///*********************************************************************
//* @fn      SimplePeripheral_enableCache
//*
//* @brief   Enables the instruction cache and releases power constaints
//*          Allows device to sleep again
//*
//* @param   None.
//*
//* @return  None.
//*/
//static void SimplePeripheral_enableCache()
//{
//    //uint_least16_t hwiKey = Hwi_disable();
//    Power_releaseConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
//    Power_releaseConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
//    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true);
//    //Hwi_restore(hwiKey);
//}

