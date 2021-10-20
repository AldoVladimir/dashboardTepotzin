
#ifndef PERIPHERALS_COMMONDECLARATIONS_H_
#define PERIPHERALS_COMMONDECLARATIONS_H_



typedef enum {
  ERROR_LOW_SYS_CURRENT,        // Corriente entregada por los páneles está baja. Posible sombra
  SUCCESS_NORMAL_SYS_CURRENT,   // La corriente de los paneles es normal
  ERROR_SKIDDING_TIRES,         // Llantas del robot están patinando
  ERROR_JAMMED_TIRES,           // Llantas del robot están atascadas
  SUCCESS_MOVE_DONE             // El movimiento se completó sin problemas
} Movement_Status;

typedef enum {
    EXIT_THE_CLUSTER,
    ESCAPE_LOW_SYS_CURRENT,
    ESCAPE_SKIDDING_TIRES,
    ESCAPE_JAMMED_TIRES
} EscapeStrategy;


#endif /* PERIPHERALS_COMMONDECLARATIONS_H_ */
