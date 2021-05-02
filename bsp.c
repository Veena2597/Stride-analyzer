#include <stdio.h>		// Used for printf()
#include "xparameters.h"	// Contains hardware addresses and bit masks
#include "qpn_port.h"
#include "bsp.h"
#include "stepanalyzer.h"
#include "xintc.h"		// Interrupt Drivers
#include "xtmrctr.h"		// Timer Drivers
#include "xtmrctr_l.h" 		// Low-level timer drivers
#include "xil_printf.h" 	// Used for xil_printf()
#include "xil_exception.h"
#include "xgpio.h" 		// LED driver, used for General purpose I/i
#include "xspi.h"
#include "xspi_l.h"
#include "PmodNAV.h"
#include "sleep.h"
#include "lcd.h"
#include "stdlib.h"

#define GPIO_CHANNEL1 1

void debounceInterrupt(); // Write This function
void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber);
void btn_handler();
void nav_handler();

XIntc sys_intc; // Create ONE interrupt controllers XIntc
static XGpio sys_btns;
static XGpio sys_nav;
static XTmrCtr axiTimer;
static XGpio dc;
static XSpi spi;

XSpi_Config *spiConfig;	/* Pointer to Configuration data */

// Suggest Creating two int's to use for determining the direction of twist
Xuint32 nav_data;
Xuint32 btn_data;
u32 controlReg;
int timerTrigger = 0;
char time[1];

void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber)
{
	/*if(stage == 1 && timerTrigger<10){
		caliMag();
		timerTrigger += 1;
	}
	else if(stage == 1 && timerTrigger>=10){
		stage = 2;
		timerTrigger = 0;
		lcdPrint("STAND STILL", 35, 160);
	}*/
	if(stage == 2 && timerTrigger<4){
		//time = (4-timerTrigger)+'0';
		itoa(3-timerTrigger,time,10);
		//xil_printf("%c\n", time);
		lcdPrint(&time, 115, 195);
		//xil_printf("%d\n",timerTrigger);
		timerTrigger += 1;
	}
	else if(stage == 2 && timerTrigger>=4){
		stage = -1;
		timerTrigger = 0;
	}
	else{
		timerTrigger = 0;
	}
}
void btn_handler(){

	XGpio_SetDataDirection(&sys_btns, 1, 0xFFFFFFFF);
	btn_data = XGpio_DiscreteRead(&sys_btns, 1);
	XGpio_DiscreteClear(&sys_btns, 1, BUTTON_INTERRUPT);
	stateCall(btn_data);
	XGpio_InterruptClear(&sys_btns, BUTTON_INTERRUPT);
}

void nav_handler(){
	dispatch(TICK);
	//XGpio_InterruptClear(&sys_nav, NAV_INTERRUPT);
	//usleep(500000);
}

/*..........................................................................*/
void BSP_init(void) {
	/* Setup LED's, etc */
	/* Setup interrupts and reference to interrupt handler function(s)  */

	/*
	 * Initialize the interrupt controller driver so that
	 * it is ready to use, specify the device ID that is generated in
	 * xparameters.h
	 */
	XStatus Status;
	Status = XST_SUCCESS;

	Status = XIntc_Initialize(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID);
	if ( Status != XST_SUCCESS )
	{
		if( Status == XST_DEVICE_NOT_FOUND )
		{
			xil_printf("XST_DEVICE_NOT_FOUND...\r\n");
		}
		else
		{
			xil_printf("a different error from XST_DEVICE_NOT_FOUND...\r\n");
		}
		xil_printf("Interrupt controller driver failed to be initialized...\r\n");
		return XST_FAILURE;
	}

	Status = XGpio_Initialize(&sys_btns, XPAR_AXI_GPIO_BTN_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XGpio_Initialize(&sys_nav, XPAR_GPIO_NAV_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XTmrCtr_Initialize(&axiTimer, XPAR_AXI_TIMER_0_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialize timer fail!\n");
		return XST_FAILURE;
	}

	Status = XIntc_Connect(&sys_intc,
					XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR,
					(XInterruptHandler)XTmrCtr_InterruptHandler,
					(void *)&axiTimer);

	Status = XIntc_Connect(&sys_intc,XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_BTN_IP2INTC_IRPT_INTR,//XPAR_MICROBLAZE_0_AXI_INTC_BTNS_IP2INTC_IRPT_INTR,
						(XInterruptHandler)btn_handler, &sys_btns);

	Status = XIntc_Connect(&sys_intc,XPAR_MICROBLAZE_0_AXI_INTC_GPIO_NAV_IP2INTC_IRPT_INTR,//XPAR_MICROBLAZE_0_AXI_INTC_BTNS_IP2INTC_IRPT_INTR,
							(XInterruptHandler)nav_handler, &sys_nav);

	//nav_data = XGpio_DiscreteRead(&sys_nav, 1);
	//xil_printf("I'm done");
}

/*..........................................................................*/
void QF_onStartup(void) {                 /* entered with interrupts locked */

	/* Enable interrupts */
	//xil_printf("\n\rQF_onStartup\n"); // Comment out once you are in your complete program

	// Start interrupt controller
	XStatus Status;

	Status = XGpio_Initialize(&dc, XPAR_SPI_DC_DEVICE_ID);
	if (Status != XST_SUCCESS)  {
		xil_printf("Initialize GPIO dc fail!\n");
		return XST_FAILURE;
	}

	XGpio_SetDataDirection(&dc, 1, 0x0);

	spiConfig = XSpi_LookupConfig(XPAR_SPI_DEVICE_ID);
	if (spiConfig == NULL) {
		xil_printf("Can't find spi device!\n");
		return XST_DEVICE_NOT_FOUND;
	}

	Status = XSpi_CfgInitialize(&spi, spiConfig, spiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialize spi fail!\n");
		return XST_FAILURE;
	}

	XSpi_Reset(&spi);
	controlReg = XSpi_GetControlReg(&spi);
	XSpi_SetControlReg(&spi,
			(controlReg | XSP_CR_ENABLE_MASK | XSP_CR_MASTER_MODE_MASK) &
			(~XSP_CR_TRANS_INHIBIT_MASK));

	XSpi_SetSlaveSelectReg(&spi, ~0x01);

	initLCD();
	clrScr();

	/* Backgroud with Red display */
	Display();

	Status = XIntc_Start(&sys_intc, XIN_REAL_MODE);
	if ( Status != XST_SUCCESS )
	{
		xil_printf("Interrupt controller driver failed to start...\r\n");
		return XST_FAILURE;
	}

	// Enable interrupt controller
	XIntc_Enable(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR);
	XIntc_Enable(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_BTN_IP2INTC_IRPT_INTR);
	XIntc_Enable(&sys_intc, XPAR_MICROBLAZE_0_AXI_INTC_GPIO_NAV_IP2INTC_IRPT_INTR);
	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	XTmrCtr_SetHandler(&axiTimer, TimerCounterHandler, &axiTimer);
	XTmrCtr_SetOptions(&axiTimer, 0,
				XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);
	XTmrCtr_SetResetValue(&axiTimer, 0, 0xFA0A1EFF);//0xFFFF0000
	XTmrCtr_Start(&axiTimer, 0);
	//xil_printf("Timer start!\n");

	XGpio_InterruptEnable(&sys_btns, BUTTON_INTERRUPT);
	//xil_printf("Initialized btns!\r\n");
	XGpio_InterruptGlobalEnable(&sys_btns);
	//xil_printf("Initialized btns!\r\n");

	XGpio_InterruptEnable(&sys_nav, NAV_INTERRUPT);
	//xil_printf("Initialized nav!\r\n");
	XGpio_InterruptGlobalEnable(&sys_nav);
	//xil_printf("Initialized nav!\r\n");

	//nav_data = XGpio_DiscreteRead(&sys_nav, 1);
	// register handler with Microblaze
	microblaze_register_handler((XInterruptHandler)XIntc_DeviceInterruptHandler,
			(void*)XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID);

	//Enable interrupts on MicroBlaze
	microblaze_enable_interrupts();
	//xil_printf("Interrupts enabled!\r\n");
}


void QF_onIdle(void) {        /* entered with interrupts locked */

    QF_INT_UNLOCK();                       /* unlock interrupts */

    {
    	// Write code to increment your interrupt counter here.
    	// QActive_postISR((QActive *)&AO_Lab2A, ENCODER_DOWN); is used to post an event to your FSM

    }
}

/* Do not touch Q_onAssert */
/*..........................................................................*/
void Q_onAssert(char const Q_ROM * const Q_ROM_VAR file, int line) {
    (void)file;                                   /* avoid compiler warning */
    (void)line;                                   /* avoid compiler warning */
    QF_INT_LOCK();
    for (;;) {
    }
}

/* Interrupt handler functions here.  Do not forget to include them in lab2a.h!
To post an event from an ISR, use this template:
QActive_postISR((QActive *)&AO_Lab2A, SIGNALHERE);
Where the Signals are defined in lab2a.h  */

/******************************************************************************
*
* This is the interrupt handler routine for the GPIO for this example.
*
******************************************************************************/
void GpioHandler(void *CallbackRef) {
	// Increment A counter
}

void TwistHandler(void *CallbackRef) {
	//XGpio_DiscreteRead( &twist_Gpio, 1);

}

void dispatch (QSignal sig){
	Q_SIG((QHsm *) &AO_Lab3A) = sig;
	QHsm_dispatch((QHsm *)&AO_Lab3A);
}


void debounceTwistInterrupt(){
	// Read both lines here? What is twist[0] and twist[1]?
	// How can you use reading from the two GPIO twist input pins to figure out which way the twist is going?
}

void debounceInterrupt() {
	QActive_postISR((QActive *)&AO_Lab3A, CALIBRATE);
	// XGpio_InterruptClear(&sw_Gpio, GPIO_CHANNEL1); // (Example, need to fill in your own parameters
}
