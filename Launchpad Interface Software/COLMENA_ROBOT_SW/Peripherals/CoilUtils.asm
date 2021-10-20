* utils.c contiene funciones útiles para la generación de señales
* en tiempo crítico

		.sect ".text:genNewSignalValue_Sine"
		.align  4
		.clink
		.thumbfunc genNewSignalValue_Sine
		.thumb
		.global genNewSignalValue_Sine   ; Función para generar un nuevo valor de señal y transmitir por SSI
		.global osci_h				; Vector de estructuras DigOscilator
*		.global num_active			; Número de señales activas
*		.global w					; Vectores de factores para salida al DAC-8bits (0 a 255)

genNewSignalValue_Sine:
		MOV R0, #0				; Inicializa R0 = 0, será el registro que lleve la suma de la señal "sum_t"
		MOV R1, #13				; Carga R1 con un 13, cantidad de iteraciones
		LDR R2, osci_h_Addr		; Carga en R2 la dirección del vector de osci_handler
		MOV R6, R2				; Guarda la dirección en R6 para usarla en el ciclo externo
CYC:
		LDRB R3, [R2], #1		; Apunta "CAMPO ACTIVE". Carga en R3 el valor de la direccion de osci_h. Postinc R2 para apuntar a "CAMPO a1"
		CMP R3, #1				; Verifica si Active es 1
		BNE	NEXT				; Si no son iguales (la señal no está activa), va a NEXT
		LDRB R3, [R2], #1		; Carga R3 = "a1". Postincrementa R2 para apuntar a "CAMPO y(n-1)"
		LDRSB R4, [R2], #1		; Carga R4 = "y(n-1)". Postincrementa R2 para apuntar a "CAMPO y(n-2)"
		MUL R3, R3, R4			; Carga R3 = "a1*y(n-1)", en formato Q12
		ASR R3, R3, #6			; Corrimiento aritmético a R3 para obtener valor en formato Q6
		LDRSB R5, [R2]			; Carga R5 = "y(n-2)". R2 permanece apuntando al "CAMPO y(n-2)"
*		SUBS R3, R3, R5			; Carga R3 -> "y(n) = a1*y(n-1) - y(n-2)"
*		ADD R0, R0, R3			; Suma a R0, "sum_t = sum_t + y(n) del oscilador iésimo"
*		ADD R0, R0, #32			; Suma a R0, "sum_t = sum_t + 2^Q6/2 del oscilador iésimo"
		SUBS R3, R3, R5			; Carga R0 -> "y(n) = a1*y(n-1) - y(n-2)"
		ADDS R0, R3, #32		; Suma a R0, "sum_t = sum_t + 2^Q6/2 del oscilador iésimo"
		IT	MI					; Si el resultado de la suma es menor que 0
		MOVMI R0, #0			; Se carga 0 en R0
UPDT:
		STRB R4, [R2], #-1		; Guarda en "CAMPO y(n-2)" lo que había en "CAMPO y(n-1)". R2 apunta a "CAMPO y(n-1)"
		STRB R3, [R2]			; Guarda en "CAMPO y(n-1)" el valor que acaba de calcular "y(n)" (R3)
		B OUT					; Si ya se generó una señal se manda a realizar el cálculo de salida. Ya no habrá otra salida señal
NEXT:
		ADD R6, R6, #4			; Apunta a la siguiente estructura DigOscilator del arreglo
		MOV R2, R6				; Guarda la dirección en R2 para usarla en el ciclo
		SUBS R1, #1				; Resta 1 a R1 (contador de ciclo)
		BNE.N CYC				; Si la resta anterior no es cero, vuelve a label CYC
*		LDR R2, num_active_Addr	; Carga en R2 la dirección de la variable de señales activas
*		LDR R2, [R2]			; Carga en R2 el valor num_active
*		LDR R1, w_Addr			; Carga en R1 la dirección del vector de factores W
*		LDRB R1, [R1, R2]		; Carga el valor del vector de factores W en R1. Con un offset dado por num_active
OUT:
		MOV R1, #255			; Carga 255 a R1 para tener una amplitud de 3.3V
		MUL R0, R0, R1			; Carga en R0 el valor de "w*y(n)"
		LSR R0, R0, #6			; Corrimiento lógigo a R0 para obtener valor en formato Q6 "w*sum_t/2^Q6"
		MOV	R1, #0x1300			; Carga R1 con el DAC_BASE
		ADD R0, R1, R0			; Suma "DAC_BASE + w*sum_t = 0x1300 + 0x00**"
FIN:
		BX LR					; Regresa de la llamada de la funcion



osci_h_Addr		.word osci_h
* num_active_Addr	.word num_active
* w_Addr			.word w
