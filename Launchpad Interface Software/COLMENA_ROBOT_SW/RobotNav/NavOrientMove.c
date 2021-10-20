#include "NavOrientMove.h"


void NavOrientMove(struct NavState * NavS){

    /*
     * Estado Orient and Move de la m�quina de estados. Recibe el vector de direcci�n
     * �ptimo y orienta al robot en la direcci�n de dicho vector. Traslada al robot
     * linealmente hacia adelante con una magnitud proporcional a la magnitud del vector.
     */

    /* ---> El robot realiza la rutina para orientarse en la direcci�n del
     *      vector de direcci�n �ptimo previamente evaluado.
     */

    //float ObjMag = NavS->DirVector[0];                                    // Magnitud objetivo
    float ObjAng = NavS->DirVector[1];                                      // �ngulo objetivo

    int ObjSteps = (int)round(ObjAng / NavS->AngStep);                      // N�mero de pasos angulares objetivo a rotar
    bool rot_dir = ObjSteps >= 0 ? DIR_CLOCKWISE : DIR_COUNTERCLOCKWISE;    // Selecciona la direcci�n de giro seg�n el signo del �ngulo objetivo

    MoveRobotSteps(ROTATION_MOVEMENT, abs(ObjSteps), rot_dir, NavS);                     // Rota la cantidad de pasos objetivo

    /* ---> Una vez que el robot se orient� en la direcci�n �ptima, realiza
     *      la secuencia de avance seg�n la magnitud del vector de direcci�n.
     *      La magnitud del vector indica qu� tan pr�xima est� la fuente de
     *      emisi�n del robot actual.
     */

    MoveRobotSteps(TRANSLATION_MOVEMENT, 5, DIR_FORWARD, NavS);                     // Rota la cantidad de pasos objetivo

}


/*
switch(MovSt){
    case ERROR_LOW_SYS_CURRENT:
        // Si se tuvo un error de corriente de paneles
        // baja, se debe entrar a un estado para deshacer
        // los �ltimos pasos que hizo el robot.
    break;

    case ERROR_SKIDDING_TIRES:
        // Si las ruedas est�n patinando, debe entrar a un estado
        // para intentar alguna estrategia que d� tracci�n al robot.
    break;

    case ERROR_JAMMED_TIRES:
        // Si las ruedas est�n atascadas, debe entrar a un estado
        // que implemente una estrategia que le permita salir.
    break;

    case SUCCESS_MOVE_DONE:
        // Si el movimiento fue existoso, contin�a con el siguiente
        // estado de la m�quina de estados de navegaci�n.
    break;
}
*/
