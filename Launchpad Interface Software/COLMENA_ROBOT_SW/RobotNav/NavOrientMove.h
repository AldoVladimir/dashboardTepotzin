
#ifndef ROBOTNAV_NAVORIENTMOVE_H_
#define ROBOTNAV_NAVORIENTMOVE_H_

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include "Peripherals/Coil.h"
#include "Peripherals/SensorsManager.h"
#include "Peripherals/Motors.h"
#include "RobotNav/RF_Utils.h"
#include "StateMachine.h"

#define TIME_TO_ON_SEC  1

struct NavState;

void NavOrientMove(struct NavState * NavS);


#endif /* ROBOTNAV_NAVORIENTMOVE_H_ */
