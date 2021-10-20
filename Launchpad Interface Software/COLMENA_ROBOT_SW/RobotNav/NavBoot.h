
#ifndef ROBOTNAV_NAVBOOT_H_
#define ROBOTNAV_NAVBOOT_H_

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include "Peripherals/Radio.h"
#include "Peripherals/Coil.h"
#include "Peripherals/SensorsManager.h"
//#include "Peripherals/Radio.h"
#include "Peripherals/Gpio.h"
#include "Peripherals/FlashMem.h"
#include "Peripherals/Motors.h"
#include "RobotNav/Params_Manager.h"
#include "RobotNav/RF_Utils.h"
#include "RobotNav/StateMachine.h"

#define TIME_TO_ON_SEC  1

struct NavState;
enum Ch_type;


void NavBoot(struct NavState * NavS);
void WaitForFirstDetection(enum Ch_type acs_ch);


#endif /* ROBOTNAV_NAVBOOT_H_ */
