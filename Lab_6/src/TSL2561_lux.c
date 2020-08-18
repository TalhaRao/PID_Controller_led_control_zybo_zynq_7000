/*
 * TSL2561_lux.c
 *
 *  Created on: Dec 18, 2017
 *      Author: Talha
 */

//**********************************************************************
//Includes
#include <math.h>
#include <stddef.h>
#include <xiicps.h>
#include <xil_exception.h>
#include <xil_printf.h>
#include <xil_types.h>
#include <xscugic.h>
#include <xstatus.h>

//#include "lux.h"


//**********************************************************************
//Constant Definitions
//Created in xparameters.h

#define IIC_DEVICE_ID		XPAR_XIICPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define IIC_INT_VEC_ID		XPAR_XIICPS_0_INTR



// The slave address to send to and receive from.

//#define IIC_SLAVE_ADDR		0x39
#define SLAVE_ADDR			0x39
#define IIC_SCLK_RATE		100000

//**********************************************************************
//Register Values

#define CONTROL_POWERON		0x03
#define COMMAND_BIT			0x80
#define	TIMING_SEL			0x01
#define INT_TIME_1xG		0x02
#define INT_TIME_16xG		0x12
#define CHAN0_LOW        	0x0C

//**********************************************************************
//Lux Values

#define Ratio_RANGE_0		0
#define Ratio_RANGE_1		0.50
#define	Ratio_RANGE_2		0.61
#define Ratio_RANGE_3		0.80
#define Ratio_RANGE_4		1.30

#define lux_0_1				(((0.0304 * ch0) - (0.062 * ch0) * (pow(ratio, 1.4))))

#define lux_1_2				(0.0224 * ch0 - 0.031 * ch1)

#define lux_2_3				(0.0128 * ch0 - 0.0153 * ch1)

#define lux_3_4				(0.00146 * ch0 - 0.00112 * ch1)

//**********************************************************************
// Function Prototypes

int i2c_init(u16 DeviceId);
static int SetupInterruptSystem(XIicPs *IicPsPtr);

void Handler(void *CallBackRef, u32 Event);


//**********************************************************************
// Variable Definitions

static XIicPs Iic;						/* Instance of the IIC Device */
XScuGic InterruptController;	/* Instance of the Interrupt Controller */

//Send and Receive Buffers
u8 SendBuffer[4];    			/* Buffer for Transmitting Data */
u8 RecvBuffer[4];    			/* Buffer for Receiving Data */
float ch0;
float ch1;
float ratio;
char adcChannels[4];			//To store values read from ADC channel 0 and 1


// The following counters are used to determine when the entire buffer has
// been sent and received.

volatile u32 SendComplete;
volatile u32 RecvComplete;
volatile u32 TotalErrorCount;

//****************************************************************************
// Initialize I2c Communication
int i2c_init(u16 DeviceId) {
	int Status;
	XIicPs_Config *Config;
	/*
	 * Initialize the IIC driver so that it's ready to use
	 * Look up the configuration in the config table, then initialize it.
	 */
	Config = XIicPs_LookupConfig(DeviceId);
	if (NULL == Config) {
		return XST_FAILURE;
	}
	Status = XIicPs_CfgInitialize(&Iic, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	Status = XIicPs_SelfTest(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	/*
	 * Connect the IIC to the interrupt subsystem such that interrupts can
	 * occur. This function is application specific.
	 */
	Status = SetupInterruptSystem(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handlers for the IIC that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the IIC driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	XIicPs_SetStatusHandler(&Iic, (void *) &Iic, Handler);

	/*
	 * Set the IIC serial clock rate.
	 */
	XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);
	return XST_SUCCESS;
}

//********************************************************************
//Power On the Sensor
int powerOnSensor() {

	//Select Command Register
	SendBuffer[0] = COMMAND_BIT;

	//Set control bit to power on the device
	SendBuffer[1] = CONTROL_POWERON;

	SendComplete = FALSE;

	//Send 2 bytes to slave to initiate ADC conversion
	XIicPs_MasterSend(&Iic, SendBuffer, 2, SLAVE_ADDR);

	//Wait for bytes to be sent
	while (!SendComplete) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}
	//Check if receive byte is 0x03
	RecvComplete = FALSE;

	XIicPs_MasterRecv(&Iic, RecvBuffer, 1, SLAVE_ADDR);

	while (!RecvComplete) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}
	if (RecvBuffer[0] == 51) {

		xil_printf("Device Is Turned On \n\n\r");
	} else {

		xil_printf("Device initialization Failed \n\n\r");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


//********************************************************************
// Set gain value where 0:1X and 1:16X

int setGain(int gain){
	//Select Timing Register
	SendBuffer[0] = TIMING_SEL | COMMAND_BIT;

	switch (gain){

	case 0:
		//Default Integration Time 402ms and 1xGain
		SendBuffer[1] = INT_TIME_1xG;
		break;

	case 1:

		//Default Integration Time 402ms and 16xGain
		SendBuffer[1] = INT_TIME_16xG;
		break;

	default:
		//Default Integration Time 402ms and 1xGain
		SendBuffer[1] = INT_TIME_1xG;
		break;
	}


	SendComplete = FALSE;
	//Write 2 bytes on device
	XIicPs_MasterSend(&Iic, SendBuffer, 2,	SLAVE_ADDR);

	while (!SendComplete) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}


//***************************************************************
//Get Lux Value

float getLux(){

	float ratio = 0;
	float lux = 0;

	//To Read 4 bytes of data from register(0x0C | 0x80)
	// ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
	SendBuffer[0] = CHAN0_LOW | COMMAND_BIT; // +i ;

	SendComplete = FALSE;
	XIicPs_MasterSend(&Iic, SendBuffer, 1, SLAVE_ADDR);

	while (!SendComplete) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}

	RecvComplete = FALSE;

	//Reading 4 Bytes
	XIicPs_MasterRecv(&Iic, RecvBuffer, 4, SLAVE_ADDR);

	while (!RecvComplete) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}

	//Reterivig channel values in a char array
	for (int i = 0; i < 4; i++) {
		adcChannels[i] = (char)RecvBuffer[i];
	}



	// Convert the data
	ch0 = (adcChannels[1] * 256 + adcChannels[0]);
	ch1 = (adcChannels[3] * 256 + adcChannels[2]);


	ratio = ch1/ch0;


	//Lux Calculation according to the formulas given in DataSheet
	if((ratio > Ratio_RANGE_0) && (ratio <= Ratio_RANGE_1)){

		lux = lux_0_1;
	}
	else if((ratio > Ratio_RANGE_1) && (ratio <= Ratio_RANGE_2)){

		lux = lux_1_2;
	}
	else if((ratio > Ratio_RANGE_2) && (ratio <= Ratio_RANGE_3)){

		lux = lux_2_3;
	}
	else if((ratio > Ratio_RANGE_3) && (ratio <= Ratio_RANGE_4)){

		lux = lux_3_4;
	}
	else if((ratio > Ratio_RANGE_4)){

		lux = 0;
	}



	return lux;
}

//***************************************************************
//Print float value of lux

void printLux(float lux) {

	int whole, thousandths;
	whole = lux;
	thousandths = (lux - whole) * 1000;
	xil_printf("Total LUX: %0d.%03d \n\n\r", whole, thousandths);

}


//*******************************************************************************/
void Handler(void *CallBackRef, u32 Event)
{
	/*
	 * All of the data transfer has been finished.
	 */
	if (0 != (Event & XIICPS_EVENT_COMPLETE_RECV)){
		RecvComplete = TRUE;
	} else if (0 != (Event & XIICPS_EVENT_COMPLETE_SEND)) {
		SendComplete = TRUE;
	} else if (0 == (Event & XIICPS_EVENT_SLAVE_RDY)){
		/*
		 * If it is other interrupt but not slave ready interrupt, it is
		 * an error.
		 * Data was received with an error.
		 */
		TotalErrorCount++;
	}
}

//*******************************************************************************/
static int SetupInterruptSystem(XIicPs *IicPsPtr)
{
	int Status;
	XScuGic_Config *IntcConfig; /* Instance of the interrupt controller */

	Xil_ExceptionInit();

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(&InterruptController, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
				&InterruptController);

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(&InterruptController, IIC_INT_VEC_ID,
			(Xil_InterruptHandler)XIicPs_MasterInterruptHandler,
			(void *)IicPsPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the Iic device.
	 */
	XScuGic_Enable(&InterruptController, IIC_INT_VEC_ID);


	/*
	 * Enable interrupts in the Processor.
	 */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}
