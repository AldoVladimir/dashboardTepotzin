#include "fft.h"

// float w_NUTTAL[N/2] = {0.0003628,0.00036571,0.00037447,0.00038909,0.00040963,0.00043615,0.00046874,0.0005075,0.00055256,0.00060406,0.00066216,0.00072704,0.0007989,0.00087797,0.00096448,0.0010587,0.0011609,0.0012713,0.0013903,0.0015182,0.0016554,0.0018023,0.0019591,0.0021264,0.0023045,0.0024939,0.0026951,0.0029085,0.0031347,0.0033741,0.0036272,0.0038948,0.0041772,0.0044751,0.0047891,0.0051199,0.005468,0.0058341,0.0062189,0.0066231,0.0070474,0.0074924,0.0079591,0.008448,0.00896,0.0094958,0.010056,0.010642,0.011255,0.011894,0.012561,0.013258,0.013984,0.01474,0.015528,0.016349,0.017203,0.018091,0.019015,0.019975,0.020972,0.022007,0.023082,0.024197,0.025352,0.02655,0.027792,0.029077,0.030408,0.031785,0.033209,0.034681,0.036203,0.037776,0.0394,0.041076,0.042806,0.04459,0.04643,0.048327,0.050281,0.052294,0.054366,0.0565,0.058695,0.060952,0.063273,0.065659,0.068111,0.070628,0.073214,0.075867,0.07859,0.081383,0.084248,0.087184,0.090192,0.093274,0.096431,0.099662,0.10297,0.10635,0.10981,0.11335,0.11697,0.12067,0.12444,0.1283,0.13223,0.13625,0.14035,0.14453,0.1488,0.15315,0.15758,0.16209,0.16669,0.17137,0.17613,0.18098,0.18591,0.19093,0.19603,0.20121,0.20648,0.21183,0.21727,0.22279,0.22839,0.23407,0.23984,0.24569,0.25162,0.25763,0.26372,0.26989,0.27613,0.28246,0.28886,0.29534,0.30189,0.30852,0.31522,0.32199,0.32883,0.33575,0.34273,0.34978,0.35689,0.36407,0.37131,0.37861,0.38597,0.39339,0.40086,0.40839,0.41598,0.42361,0.43129,0.43902,0.44679,0.45461,0.46247,0.47036,0.4783,0.48626,0.49426,0.50229,0.51034,0.51842,0.52652,0.53465,0.54278,0.55094,0.5591,0.56728,0.57546,0.58364,0.59183,0.60001,0.60819,0.61636,0.62452,0.63267,0.6408,0.64891,0.657,0.66507,0.67311,0.68111,0.68908,0.69702,0.70491,0.71277,0.72057,0.72833,0.73603,0.74368,0.75127,0.7588,0.76626,0.77366,0.78098,0.78824,0.79541,0.80251,0.80952,0.81645,0.82328,0.83003,0.83668,0.84324,0.84969,0.85605,0.86229,0.86843,0.87446,0.88037,0.88617,0.89185,0.89741,0.90284,0.90815,0.91333,0.91838,0.92329,0.92807,0.93271,0.93721,0.94157,0.94579,0.94986,0.95378,0.95756,0.96118,0.96465,0.96796,0.97112,0.97412,0.97696,0.97965,0.98217,0.98452,0.98672,0.98875,0.99061,0.99231,0.99384,0.9952,0.99639,0.99742,0.99827,0.99895,0.99947,0.99981,0.99998};
uint8_t w_NUTTAL[N/2] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,7,7,7,8,8,8,9,9,10,10,10,11,11,12,12,13,13,14,15,15,16,16,17,18,18,19,20,20,21,22,23,23,24,25,26,27,28,29,29,30,31,32,33,34,35,36,38,39,40,41,42,43,45,46,47,48,50,51,52,54,55,57,58,59,61,62,64,65,67,69,70,72,73,75,77,78,80,82,84,85,87,89,91,93,95,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,141,143,145,147,149,151,153,155,157,159,161,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,194,196,198,199,201,203,205,207,209,210,212,214,215,217,219,220,222,223,225,226,228,229,231,232,233,235,236,237,238,239,241,242,243,244,245,246,246,247,248,249,250,250,251,252,252,253,253,254,254,254,255,255,255,255,255,255,255};

float X_Re_g[N] = {};
float X_Im_g[N] = {};

// int contador = 0;


void ApplyWindow(float *x, FFT_WINDOW win, uint16_t Nfft){

    uint16_t i;
    uint16_t Nfft_2 = Nfft/2;
    // float *w;
    uint8_t *w;

    switch(win){

        case NUTTAL:
            w = w_NUTTAL;
        break;

        default:
            return;
    }

    for(i=0; i<Nfft_2; i++){
        x[i] = x[i]*(((float)w[i])/256);
    }

    for(i=Nfft_2; i<Nfft; i++){
        x[i] = x[i]*(((float)w[Nfft - i])/256);
    }
}

void fft_CooleyTukey(float* x, float* y, uint16_t Nfft, bool length){

    /*
     * Algoritmo FFT Cooley & Tukey.
     * Parámetros:
     *              @x: Apuntador a vector de muestras (señal de entrada)
     *              @y: Apuntador a vector de resultado
     *              @Nfft: Número de muestras de FFT (debe ser potencia de 2)
     *              @lenght: MIRROR_LEN / HALF_LEN, para resultado en espejo o simple
     */

    // contador = 0;

    fft_radix2(x, X_Re_g, X_Im_g, Nfft, 1);
    fftComplex2Real(X_Re_g, X_Im_g, y, Nfft, length);

}



void fft_radix2(float* x, float* X_Re, float* X_Im, unsigned int Nfft, unsigned int s) {

    // At the lowest level pass through (delta T=0 means no phase).

    if (Nfft == 1) {
        X_Re[0] = x[0];
        X_Im[0] = x[0];
        return;
    }

    uint16_t k = 0;
    float t_Re;
    float t_Im;
    uint16_t N_2 = Nfft/2;
    uint16_t _2s = 2*s;

    // Cooley-Tukey: recursively split in two, then combine beneath.
    fft_radix2(x, X_Re, X_Im, N_2, _2s);
    fft_radix2(x+s, X_Re + N_2, X_Im + N_2, N_2, _2s);

    float _2pi_N = 2*PI/Nfft;
    float sinw, cosw;
    float cosw_XRe_plus_sinw_XIm, cosw_XIm_minus_sinw_XRe;

    for (k = 0; k < N_2; k++) {

        cosw = cos(_2pi_N*k);
        sinw = sin(_2pi_N*k);

        cosw_XRe_plus_sinw_XIm  = cosw*X_Re[k + N_2] + sinw*X_Im[k + N_2];
        cosw_XIm_minus_sinw_XRe = cosw*X_Im[k + N_2] - sinw*X_Re[k + N_2];

        t_Re = X_Re[k];
        t_Im = X_Im[k];

        X_Re[k]       = t_Re + (cosw_XRe_plus_sinw_XIm);
        X_Im[k]       = t_Im + (cosw_XIm_minus_sinw_XRe);
        
        X_Re[k + N_2] = t_Re - (cosw_XRe_plus_sinw_XIm);
        X_Im[k + N_2] = t_Im - (cosw_XIm_minus_sinw_XRe);
    }
}

void fftComplex2Real(float* X_Re, float* X_Im, float* y, int Nfft, bool length){
    uint16_t i;

    if(length == HALF_LEN)
        Nfft = Nfft/2;

    for(i=0; i<Nfft; i++){
        // Para obtener la PSD se divide la energía entre el ancho de bin Df = Fs/Nfft = 52kHz/256
        //y[i] += (X_Re[i]*X_Re[i] +  X_Im[i]*X_Im[i])*0.004923076;   // 256
        y[i] += (X_Re[i]*X_Re[i] +  X_Im[i]*X_Im[i])*0.00984615;    // 512
    }
}



// ---------------- Deprocated ----------------

//void fft_CooleyTukey_CacheAsRAM(float* x, float* y, uint16_t Nfft, bool length){
//
//    /*
//     * Algoritmo FFT Cooley & Tukey.
//     * Parámetros:
//     *              @x: Apuntador a vector de muestras (señal de entrada)
//     *              @y: Apuntador a vector de resultado
//     *              @Nfft: Número de muestras de FFT (debe ser potencia de 2)
//     *              @lenght: MIRROR_LEN / HALF_LEN, para resultado en espejo o simple
//     */
//
//    static float *X_Re = NULL;
//    static float *X_Im = NULL;
//
//    SimplePeripheral_disableCache();                // Disable instruction cache to use for the buffer
//    X_Re = (float *)GPRAM_BASE;                     // GPRAM_BASE es la base de memoria del cache
//    X_Im = (float *)GPRAM_BASE + N;
//
//    contador = 0;
//    fft_radix2(x, X_Re, X_Im, Nfft, 1);
//    fftComplex2Real(X_Re, X_Im, y, Nfft, length);
//
//    SimplePeripheral_enableCache();
//}
//
//static void SimplePeripheral_disableCache(){
//    /*
//     * Disables the instruction cache and sets power constaints
//     * This prevents the device from sleeping while streaming
//     */
//
//    //uint_least16_t hwiKey = Hwi_disable();
//    Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
//    Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
//    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_DISABLED, true);
//    //Hwi_restore(hwiKey);
//}
//
//static void SimplePeripheral_enableCache(){
//    /*
//     * Enables the instruction cache and releases power constaints
//     * Allows device to sleep again
//     */
//
//    //uint_least16_t hwiKey = Hwi_disable();
//    Power_releaseConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
//    Power_releaseConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
//    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true);
//    //Hwi_restore(hwiKey);
//}
