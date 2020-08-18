//**********************************************************************
//Includes
#include "xparameters.h"
#include "xiicps.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "TSL2561_lux.h"
//#include "PWM_RGB.h"
//**********************************************************************
//Constant Definitions
//Created in xparameters.h

#define IIC_DEVICE_ID		XPAR_XIICPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define IIC_INT_VEC_ID		XPAR_XIICPS_0_INTR
#define GAIN_1X				0
#define GAIN_16X			1
#define LED_VALUE			70
//**********************************************************************
// Variable Definitions

float lux;

//**********************************************************************
//Main Function to Initialize the Program
int main(void) {
	int Status;
	int Led_value = 70;
	xil_printf("I2C Light Sensor \r\n");

	//Initialze ADC and Timers for generating PWM
	pwmInit();

	//Initialize i2c device and Power On the sensor
	Status = i2c_init(IIC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("IIC Sensor not Initialized \r\n");
		return XST_FAILURE;
	}
	//Turn On the sensor to start the conversion
	Status = powerOnSensor();
	if (Status != XST_SUCCESS) {
		xil_printf("IIC Sensor not Powered On \r\n");
		return XST_FAILURE;
	}

	//Set the Gain for the sensor 0:1x and 1:16X
	Status = setGain(GAIN_1X);
	if (Status != XST_SUCCESS) {
		xil_printf("IIC Sensor not Powered On \r\n");
		return XST_FAILURE;
	}
	while (1) {
		//Read Input voltage from ADC and change RGB color accordingly
		updateLED(Led_value);

		//Read ch0 and ch1 registers and convert them to Lux
		lux = getLux();

		//Print Floating point Value of Lux
		printLux(lux);
	}
	xil_printf("Successfully ran IIC Master Interrupt Example Test\r\n");
	return XST_SUCCESS;
}
