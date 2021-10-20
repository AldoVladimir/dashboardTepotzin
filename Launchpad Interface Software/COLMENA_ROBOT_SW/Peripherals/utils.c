#include "utils.h"


float GetMedian(float* V, const int size){
    /*
     * Esta funci�n obtiene la mediana de un vector "V" de tama�o "size"
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
     * Esta funci�n se emplea a trav�s de la funci�n Qsort para
     * ordear los datos de un vector.
     */

    float *a, *b;

    a = (float *) _a;
    b = (float *) _b;

    return (*a - *b);
}


