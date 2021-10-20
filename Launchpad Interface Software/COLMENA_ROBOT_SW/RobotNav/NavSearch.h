

#ifndef ROBOTNAV_NAVSEARCH_H_
#define ROBOTNAV_NAVSEARCH_H_

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#include "RF_Utils.h"
#include "Peripherals/Coil.h"
#include "Peripherals/Motors.h"
#include "StateMachine.h"


struct NavState;

void NavSearch(struct NavState * NavS, bool mode);

void RotationalSearch(struct NavState * NavS);
void StaticSearch(struct NavState * NavS);

#define STATIC_SEARCH       true
#define ROTATIONAL_SEARCH   false

#endif /* ROBOTNAV_NAVSEARCH_H_ */
