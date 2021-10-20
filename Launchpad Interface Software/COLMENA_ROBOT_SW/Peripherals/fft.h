
#ifndef PERIPHERALS_FFT_H_
#define PERIPHERALS_FFT_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
#define MIRROR_LEN  true
#define HALF_LEN    false

#define N 512               // Número de mediciones

void fft_CooleyTukey(float* x, float* y, uint16_t Nfft, bool length);
void fft_radix2(float* x, float* X_Re, float* X_Im, unsigned int Nfft, unsigned int s);
void fftComplex2Real(float* X_Re, float* X_Im, float* y, int Nfft, bool length);

typedef enum{
  RECT,
  BLACKMAN,
  NUTTAL,
  FLATTOP
} FFT_WINDOW;

void ApplyWindow(float *x, FFT_WINDOW win, uint16_t Nfft);

#endif /* PERIPHERALS_FFT_H_ */
