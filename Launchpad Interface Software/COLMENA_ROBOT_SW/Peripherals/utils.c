#include "utils.h"


float GetMedian(float* V, const int size){
    /*
     * Esta función obtiene la mediana de un vector "V" de tamaño "size"
     */
    //int i = 0;
    //float Vaux[N];

    //for(i=0; i<size; i++){
    //    Vaux[i] = V[i];
    //}

    qsort(V, size, sizeof(float), &MedianCompare);
    return V[size/2];
}

int MedianCompare(const void *_a, const void *_b) {
    /*
     * Esta función se emplea a través de la función Qsort para
     * ordear los datos de un vector.
     */

    float *a, *b;

    a = (float *) _a;
    b = (float *) _b;

    return (*a - *b);
}


