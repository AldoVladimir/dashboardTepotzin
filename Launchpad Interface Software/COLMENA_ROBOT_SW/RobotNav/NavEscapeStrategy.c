#include "NavEscapeStrategy.h"


void NavEscapeStrategy(struct NavState * NavS){

    /*
     * Estado Escape Strategy. Este estado verifica cual es la situación de la
     * que hay que escapar y con base en ello determina la mejor forma estrategia.
     */

    switch(NavS->EscStr){

        case EXIT_THE_CLUSTER:
            MoveRobotSteps(TRANSLATION_MOVEMENT, 5, DIR_BACKWARD, NavS);
            MoveRobotSteps(ROTATION_MOVEMENT, 4, DIR_CLOCKWISE, NavS);
            MoveRobotSteps(TRANSLATION_MOVEMENT, 5, DIR_FORWARD, NavS);
        break;

        case ESCAPE_LOW_SYS_CURRENT:

        break;

        case ESCAPE_SKIDDING_TIRES:

        break;

        case ESCAPE_JAMMED_TIRES:

        break;

        default:
            MoveRobotSteps(TRANSLATION_MOVEMENT, 5, DIR_BACKWARD, NavS);
            MoveRobotSteps(ROTATION_MOVEMENT, 4, DIR_CLOCKWISE, NavS);
            MoveRobotSteps(TRANSLATION_MOVEMENT, 5, DIR_FORWARD, NavS);
        break;

    }



}




