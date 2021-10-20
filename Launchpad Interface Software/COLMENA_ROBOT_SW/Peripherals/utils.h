/*
 * utils.h
 *
 *  Created on: 23 may. 2021
 *      Author: CASTI
 */

#ifndef PERIPHERALS_UTILS_H_
#define PERIPHERALS_UTILS_H_

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>

float GetMedian(float* V, const int size);
int MedianCompare(const void *_a, const void *_b);

#endif /* PERIPHERALS_UTILS_H_ */
