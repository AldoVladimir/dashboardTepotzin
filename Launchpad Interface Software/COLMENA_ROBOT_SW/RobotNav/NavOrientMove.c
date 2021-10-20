#include "NavOrientMove.h"


void NavOrientMove(struct NavState * NavS){

    /*
     * Estado Orient and Move de la máquina de estados. Recibe el vector de dirección
     * óptimo y orienta al robot en la dirección de dicho vector. Traslada al robot
     * linealmente hacia adelante con una magnitud proporcional a la magnitud del vector.
     */

    /* ---> El robot realiza la rutina para orientarse en la dirección del
     *      vector de dirección óptimo previamente evaluado.
     */

    //float ObjMag = NavS->DirVector[0];                                    // Magnitud objetivo
    float ObjAng = NavS->DirVector[1];                                      // Ángulo objetivo

    int ObjSteps = (int)round(ObjAng / NavS->AngStep);                      // Número de pasos angulares objetivo a rotar
    bool rot_dir = ObjSteps >= 0 ? DIR_CLOCKWISE : DIR_COUNTERCLOCKWISE;    // Selecciona la dirección de giro según el signo del ángulo objetivo

    MoveRobotSteps(ROTATION_MOVEMENT, abs(ObjSteps), rot_dir, NavS);                     // Rota la cantidad de pasos objetivo

    /* ---> Una vez que el robot se orientó en la dirección óptima, realiza
     *      la secuencia de avance según la magnitud del vector de dirección.
     *      La magnitud del vector indica qué tan próxima está la fuente de
     *      emisión del robot actual.
     */

    MoveRobotSteps(TRANSLATION_MOVEMENT, 5, DIR_FORWARD, NavS);                     // Rota la cantidad de pasos objetivo

}


/*
switch(MovSt){
    case ERROR_LOW_SYS_CURRENT:
        // Si se tuvo un error de corriente de paneles
        // baja, se debe entrar a un estado para deshacer
        // los últimos pasos que hizo el robot.
    break;

    case ERROR_SKIDDING_TIRES:
        // Si las ruedas están patinando, debe entrar a un estado
        // para intentar alguna estrategia que dé tracción al robot.
    break;

    case ERROR_JAMMED_TIRES:
        // Si las ruedas están atascadas, debe entrar a un estado
        // que implemente una estrategia que le permita salir.
    break;

    case SUCCESS_MOVE_DONE:
        // Si el movimiento fue existoso, continúa con el siguiente
        // estado de la máquina de estados de navegación.
    break;
}
*/
