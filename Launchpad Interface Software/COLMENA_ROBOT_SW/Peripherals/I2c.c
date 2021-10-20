/************************************************************
 * I2c.c  Created on: 29/01/2019							*
 *															*
 *      Universidad Nacional Autonoma de Mexico             *
 *          Instituto de Ciencias Nucleares                 *
 *     Laboratorio de Instrumentacion Espacial LINX         *
 *                                                          *
 *  Desarrollador: Ing. Manuel Andres Herrera Juarez.       *
 *                                                          *
 ************************************************************/


#include "I2c.h"

/*
 * Funcion que inicializa el modulo I2C a 400KHz
 *
 * pin_SDA  =   Pin que sera usado por el bus SDA del I2C
 *              IOID_0 a IOID_31
 *
 * pin_SCL  =   Pin que sera usado por el bus SCL del I2C
 *              IOID_0 a IOID_31
 */
void initI2C(uint32_t pin_SDA, uint32_t pin_SCL){
    //Inicializa GPIO
	InitGPIO();

	//Habilita la alimentacion al dominio Serial. El Modulo I2C pertenece a este dominio.
	if(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL)!=PRCM_DOMAIN_POWER_ON){
		PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
		while(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL)!=PRCM_DOMAIN_POWER_ON);
	}

	//Habilita la señal de reloj al módulo I2C y actualiza ajustes.
	PRCMPeripheralRunEnable(PRCM_PERIPH_I2C0);
	PRCMPeripheralSleepEnable(PRCM_PERIPH_I2C0);
	PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_I2C0);
	PRCMLoadSet();

	//Configura los pines del bus I2C (SDA y SCL). Resistencia PullUp, Colector Abierto
	IOCPinTypeI2c(I2C0_BASE, pin_SDA, pin_SCL);

	//Configura el modulo I2C como maestro (El microcontrolador gobierna a los dispositivos externos).
	HWREG(I2C0_BASE + I2C_O_MCR) = 0x00000010;

	//Asigna valor al registro TPR segun la velocidad del microcontrolador y del I2C
	/*
	 * Periodo de la señal de Reloj I2C I2CCLK_PRD = 2*(1+TPR)*(SCL_LP+SCL_HP)*CLK_PRD
	 * TPR = I2CCLK_PRD/(2*(SCL_LP + SCL_HP)*CLK_PRD) - 1
	 * I2CCLK_PRD, CLK_PRD (System Period) en ns
	 *	SCL_LP I2CCLK Periodo en bajo Adimensional
	 *	SCL_HP I2CCLK Periodo en alto Adimensional
	 *		Si SCL_LP = 6, SCL_HP=4, CLK_PRD= 1/(48MHz), I2CCLK_PRD=1/(400KHz)
	 *		Entonces TPR = 5 = 0x05
	 */
	HWREG(I2C0_BASE + I2C_O_MTPR) = 0x05;
}

/*
 * Escribe byte a dispositivo por bus I2C
 *
 * device_Address   =   Entero sin signo de 7 bits que especifica la direccion I2C del dispositivo a comunicarse
 *
 * device_Register  =   Direccion de 8 bits del registro del dispositivo externo donde se escribira.
 *
 * data =   Entero sin signo de 8 bits. Dato a escribir.
 *
 */
void writeByte_I2C(uint8_t device_Address, uint8_t device_Register, uint8_t data){
    //Forza retraso de 15 ciclos de reloj
    CPUdelay(5);

    // Establece la direccion del dispositivo indicando que se quiere escribir
	I2CMasterSlaveAddrSet(I2C0_BASE, device_Address, false);

	// Escribe en el bus la direccion del registro del dispositivo a escribir
	I2CMasterDataPut(I2C0_BASE, device_Register);

	//Envia la direccion del dispositivo, indicando a la transaccion que es la direccion
	I2CMasterControl(I2C0_BASE, 0x03);

	//Espera a que el bus I2C se libere y retorna si error
	while(I2CMasterBusy(I2C0_BASE));
	if(HWREG(I2C0_BASE + I2C_O_MSTAT)&I2C_MSTAT_ERR!= I2C_MASTER_ERR_NONE){
		return;
	}

	//Pon en el bus el byte a escribir
	I2CMasterDataPut(I2C0_BASE, data);

	//Envia dato indicando a la transaccion que es el ultimo byte a enviar
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

	//Espera a que el bus I2C se libere y retorna si error
	while(I2CMasterBusy(I2C0_BASE));
	if(HWREG(I2C0_BASE + I2C_O_MSTAT)&I2C_MSTAT_ERR!= I2C_MASTER_ERR_NONE){
		return;
	}
	return;
}

/*
 * Lee un byte de un dispositivo por bus I2C
 *
 * device_Address   =   Entero sin signo de 7 bits que especifica la direccion I2C del dispositivo a comunicarse
 *
 * device_Register  =   Direccion de 8 bits del registro del dispositivo externo donde se escribira.
 *
 */

uint8_t readByte_I2C(uint8_t device_Address, uint8_t device_Register){
    //Forza retraso de 15 ciclos de reloj
	CPUdelay(5);

	// Establece la direccion del dispositivo indicando que se quiere escribir, para enviar la direccion
	I2CMasterSlaveAddrSet(I2C0_BASE, device_Address, false);

	// Escribe en el bus la direccion del registro del dispositivo a leer
	I2CMasterDataPut(I2C0_BASE, device_Register);

	//Envia la direccion del dispositivo, indicando a la transaccion que es la direccion
	I2CMasterControl(I2C0_BASE, 0x03);

	//Espera a que el bus I2C se libere y retorna 0 si error
	while(I2CMasterBusy(I2C0_BASE));
	if(HWREG(I2C0_BASE + I2C_O_MSTAT)&I2C_MSTAT_ERR!= I2C_MASTER_ERR_NONE){
		return 0;
	}

	//Forza retraso de 15 ciclos de reloj
	CPUdelay(5);

	// Establece la dirección del dispositivo en modo lectura
	I2CMasterSlaveAddrSet(I2C0_BASE, device_Address, true);

	//Inicia transaccion I2C
	I2CMasterControl(I2C0_BASE, 0x07);

	//Espera a que el bus I2C se libere y retorna 0 si error
	while(I2CMasterBusy(I2C0_BASE));
	if(HWREG(I2C0_BASE + I2C_O_MSTAT)&I2C_MSTAT_ERR!= I2C_MASTER_ERR_NONE){
		return 0;
	}

	//Retorna el valor leido
	return (uint8_t)I2CMasterDataGet(I2C0_BASE);
}

/*
 * Lee varios bytes de un dispositivo por bus I2C
 *
 * device_Address   =   Entero sin signo de 7 bits que especifica la direccion I2C del dispositivo a comunicarse
 *
 * device_Register  =   Direccion base de 8 bits del registro del dispositivo externo donde se escribira.
 *
 * out_Buffer   =   Apuntador a la primer localidad de memoria del arreglo donde se guardaran los datos leidos
 *
 * size =   Numero de bytes a leer
 */

uint8_t readBytes_I2C(uint8_t device_Address, uint8_t device_Register, uint8_t* out_Buffer, uint8_t size){
	uint32_t index;
	uint32_t err;

	//Forza retraso de 15 ciclos de reloj
	CPUdelay(5);

	// Establece la direccion del dispositivo indicando que se quiere escribir, para enviar la direccion
	I2CMasterSlaveAddrSet(I2C0_BASE, device_Address, false);

	// Escribe en el bus la direccion del registro del dispositivo a leer
	I2CMasterDataPut(I2C0_BASE, device_Register);

	//Envia la direccion del dispositivo, indicando que es una escritura de un byte
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	//Espera a que el bus I2C se libere y retorna 0 si error
	while(I2CMasterBusy(I2C0_BASE));
	if(HWREG(I2C0_BASE + I2C_O_MSTAT)&I2C_MSTAT_ERR!= I2C_MASTER_ERR_NONE){
			return 0;
	}

	//Forza retraso de 15 ciclos de reloj
	CPUdelay(5);

	// Inicializa Index igual a cero
	index = 0;

	// Establece la direccion del dispositivo en modo lectura
	I2CMasterSlaveAddrSet(I2C0_BASE, device_Address, true);

	// Inicializa transaccion I2C, indicando que es el primer byte a recibir
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

	//Etiqueta de loop para leer n datos
	sendloop:

	//Espera a que el bus I2C se libere y retorna 0 si error
	while(I2CMasterBusy(I2C0_BASE));
	err = I2CMasterErr(I2C0_BASE);
	if(err!=I2C_MASTER_ERR_NONE){
		if(err&I2C_MASTER_ERR_ARB_LOST==0x00){
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
		}
		return 0;
	}

	//Lee Byte y guarda en buffer, incrementa contador.
	out_Buffer[index] = (uint8_t)I2CMasterDataGet(I2C0_BASE);
	index++;

	//Si faltan al menos 2 bytes por leer, Envia una transaccion I2C indicando que la lecura continua y salta a la Etiqueta
	if(index != size-1){
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		goto sendloop;
	}

	//Si es el ultimo byte a leer, indicalo en la Transaccion I2C
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	//Espera a que el bus I2C se libere y si no hay error guarda el ultimo byte en el buffer
	while(I2CMasterBusy(I2C0_BASE));
	err = I2CMasterErr(I2C0_BASE);
	if(err==I2C_MASTER_ERR_NONE){
		out_Buffer[index] = (uint8_t)I2CMasterDataGet(I2C0_BASE);
	}

	//Retorna cuenta
	return size;
}
