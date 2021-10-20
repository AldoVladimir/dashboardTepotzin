#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "Board.h"
#include "Peripherals/Radio.h"
#include "RobotNav/StateMachine.h"
#include "Peripherals/Coil.h"
#include "Peripherals/COMMS_Manager.h"
#include "Peripherals/Motors.h"
#include "Peripherals/I2c.h"
#include "Peripherals/SensorsManager.h"

#include "Peripherals/Timer.h"
#include "Peripherals/FlashMem.h"
#include "RobotNav/Params_Manager.h"


#include <ti/drivers/UART.h>

int contador = 0;

/*
 * Interfaz de debug para los robots
 */

void *mainThread(void *arg0){

    UART_Handle uart;
    UART_Params uartParams;

    GPIO_init();
    UART_init();

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        while (1);
    }


    InitRF();
    ReceiveData();

    while(1){
        if(NEW_DATA_FROM_ROBOT){
            contador++;
            NEW_DATA_FROM_ROBOT = false;

            UART_write(uart, IncomingCommand.data_RX, 254);
        }
    }

}

//#include <ti/display/DisplayUart.h>
//
//int contador = 0;
//
///*
// * Interfaz de debug para los robots
// */
//
//void *mainThread(void *arg0){
//
//    GPIO_init();
//    Display_init();
//
//    Display_Params params;
//    Display_Params_init(&params);
//    Display_Handle hSerial = Display_open(Display_Type_UART, &params);
//
//    if (hSerial == NULL) {
//        while (1);
//    }
//
//    InitRF();
//    ReceiveData();
//
//    while(1){
//        if(NEW_DATA_FROM_ROBOT){
//            contador++;
//            NEW_DATA_FROM_ROBOT = false;
//
//            // Display_printf(hSerial, 0, 0, "Hello Serial!");
//            Display_printf(hSerial, 0, 0, "%s", (char *)IncomingCommand.data_RX);
//        }
//    }
//
//}
