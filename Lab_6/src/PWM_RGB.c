/*
 * PWM_RGB.c
 *
 *  Created on: Dec 18, 2017
 *      Author: Talha
 */

/***************************** Include Files ********************************/

#include <stddef.h>
#include <xil_printf.h>
#include <xil_types.h>
#include <xparameters.h>
#include <xstatus.h>
#include <xsysmon.h>
#include <xsysmon_hw.h>
#include <xttcps_hw.h>

/************************** Constant Definitions ****************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define SYSMON_DEVICE_ID 	XPAR_SYSMON_0_DEVICE_ID


/**************************** Type Definitions ******************************/
#define PCLK_FREQ_HZ		XPAR_XTTCPS_0_CLOCK_HZ /* Input freq */
#define TTC_NUM_DEVICES		3
#define MAX_LOOP_COUNT		0xFF



/***************** Macros (Inline Functions) Definitions ********************/

#define printf xil_printf /* Small foot-print printf function */
#define MAX_TIMER_COUNT  65535

/************************** Function Prototypes *****************************/

//static int readXADCInput(u16 SysMonDeviceId);
static int SysMonFractionToInt(float FloatNum);
static int initializeTTC();
void updateLED(int scale);
//void generateFullRGB(float vin);

/************************** Variable Definitions ****************************/

//static XSysMon SysMonInst;      /* System Monitor driver instance */


static u32 TimerCounterBaseAddr[] = {
	XPAR_XTTCPS_0_BASEADDR,
	XPAR_XTTCPS_1_BASEADDR,
	XPAR_XTTCPS_2_BASEADDR
};


//****************************************************************************
// MAIN FUNCTION THAT INITIALIZES THE TRIPLE TIMER COUNTERS AND GENERATES
// PWM WAVE TO DRIVE THE RGB LED

int pwmInit(void)
{

	int Status;

	//Initialize TTC
	Status = initializeTTC();

	if (Status != XST_SUCCESS) {
			xil_printf("Triple Timer Counter is initialization Failed\r\n");
			return XST_FAILURE;
		}

	printf("Triple Timer Counter is initialized\r\n");

	//Never comes here
	return XST_SUCCESS;

}


/*
**************************************************************************
*
*
* This function runs a test on the System Monitor/ADC device using the
* driver APIs.
* This function does the following tasks:
*	- Initiate the System Monitor device driver instance
*	- Run self-test on the device
*	- Setup the sequence registers to continuously monitor on-chip
*	temperature, VCCINT and VCCAUX
*	- Setup configuration registers to start the sequence
*	- Read the latest on-chip temperature, VCCINT and VCCAUX
*
* @param	SysMonDeviceId is the XPAR_<SYSMON_ADC_instance>_DEVICE_ID value
*		from xparameters.h.
*
* @return
*		- XST_SUCCESS if the example has completed successfully.
*		- XST_FAILURE if the example has failed.
*
* @note   	None
*
***************************************************************************
int readXADCInput(u16 SysMonDeviceId)
{
	int Status;
	XSysMon_Config *ConfigPtr;

	u32 VccAuxRawData;

	float vAux14;
	float VccAuxData;

	XSysMon *SysMonInstPtr = &SysMonInst;

	printf("\r\nEntering the SysMon Polled Example. \r\n");


	 * Initialize the SysMon driver.

	ConfigPtr = XSysMon_LookupConfig(SysMonDeviceId);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	XSysMon_CfgInitialize(SysMonInstPtr, ConfigPtr,
				ConfigPtr->BaseAddress);


	 * Self Test the System Monitor/ADC device

	Status = XSysMon_SelfTest(SysMonInstPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	 * Disable the Channel Sequencer before configuring the Sequence
	 * registers.

	XSysMon_SetSequencerMode(SysMonInstPtr, XSM_SEQ_MODE_SAFE);



	 * Disable all the alarms in the Configuration Register 1.

	XSysMon_SetAlarmEnables(SysMonInstPtr, 0x0);



	 * Setup the Averaging to be done for the channels in the
	 * Configuration 0 register as 16 samples:

	XSysMon_SetAvg(SysMonInstPtr, XSM_AVG_16_SAMPLES);


	 * Setup the Sequence register for 1st Auxiliary channel
	 * Setting is:
	 *	- Add acquisition time by 6 ADCCLK cycles.
	 *	- Bipolar Mode
	 *
	 * Setup the Sequence register for 16th Auxiliary channel
	 * Setting is:
	 *	- Add acquisition time by 6 ADCCLK cycles.
	 *	- Unipolar Mode

	Status = XSysMon_SetSeqInputMode(SysMonInstPtr, XSM_SEQ_CH_AUX00);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XSysMon_SetSeqAcqTime(SysMonInstPtr, XSM_SEQ_CH_AUX14 |
						XSM_SEQ_CH_AUX00);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}



	 * Enable the averaging on the following channels in the Sequencer
	 * registers:
	 * 	- On-chip Temperature, VCCINT/VCCAUX  supply sensors
	 * 	- 1st/16th Auxiliary Channels
	  *	- Calibration Channel

	Status =  XSysMon_SetSeqAvgEnables(SysMonInstPtr, XSM_SEQ_CH_TEMP |
						XSM_SEQ_CH_VCCINT |
						XSM_SEQ_CH_VCCAUX |
						XSM_SEQ_CH_AUX00 |
						XSM_SEQ_CH_AUX14 |
						XSM_SEQ_CH_CALIB);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	 * Enable the following channels in the Sequencer registers:
	 * 	- On-chip Temperature, VCCINT/VCCAUX supply sensors
	 * 	- 1st/16th Auxiliary Channel
	 *	- Calibration Channel

	Status =  XSysMon_SetSeqChEnables(SysMonInstPtr, XSM_SEQ_CH_TEMP |
						XSM_SEQ_CH_VCCINT |
						XSM_SEQ_CH_VCCAUX |
						XSM_SEQ_CH_AUX00 |
						XSM_SEQ_CH_AUX14 |
						XSM_SEQ_CH_CALIB);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}



	 * Set the ADCCLK frequency equal to 1/32 of System clock for the System
	 * Monitor/ADC in the Configuration Register 2.

	XSysMon_SetAdcClkDivisor(SysMonInstPtr, 32);



	 * Set the Calibration enables.

	XSysMon_SetCalibEnables(SysMonInstPtr,
				XSM_CFR1_CAL_PS_GAIN_OFFSET_MASK |
				XSM_CFR1_CAL_ADC_GAIN_OFFSET_MASK);


	 * Enable the Channel Sequencer in continuous sequencer cycling mode.

	XSysMon_SetSequencerMode(SysMonInstPtr, XSM_SEQ_MODE_CONTINPASS);


	 * Wait till the End of Sequence occurs

	XSysMon_GetStatus(SysMonInstPtr);  Clear the old status
	while ((XSysMon_GetStatus(SysMonInstPtr) & XSM_SR_EOS_MASK) !=
			XSM_SR_EOS_MASK);


	while (1)
		{


		//Reads the raw voltage on pin AD14
			VccAuxRawData = XSysMon_GetAdcData(&SysMonInst, XSM_CH_AUX_MIN + 14);


			vAux14 = XSysMon_RawToVoltage(VccAuxRawData);

			generateFullRGB(vAux14/3);

			//Drops value of voltage after 2 significant digits to stop from printing all the voltage fluctuations
			vAux14 =  (((float)((int)(vAux14 * 10 + 0.5))) / 10.);

			//Print Voltage value if the voltage is changed i.e not the fluctuations
			if (VccAuxData != vAux14) {
				VccAuxData = vAux14;


				printf("\r\nThe Current VAUX14 data is %0d.%03d Volts. \r\n",
						(int) (VccAuxData), SysMonFractionToInt(VccAuxData));

			}
		}

	return XST_SUCCESS;
}


void RGBControl() {
	u32 VccAuxRawData;
	float vAux14;

	//Reads the raw voltage on pin AD14
	VccAuxRawData = XSysMon_GetAdcData(&SysMonInst, XSM_CH_AUX_MIN + 14);

	vAux14 = XSysMon_RawToVoltage(VccAuxRawData);

	generateFullRGB(3 / 3);
}
*/


/****************************************************************************/
/**
*
* This function converts the fraction part of the given floating point number
* (after the decimal point)to an integer.
*
* @param	FloatNum is the floating point number.
*
* @return	Integer number to a precision of 3 digits.
*
* @note
* This function is used in the printing of floating point data to a STDIO device
* using the xil_printf function. The xil_printf is a very small foot-print
* printf function and does not support the printing of floating point numbers.
*
*****************************************************************************/
int SysMonFractionToInt(float FloatNum)
{
	float Temp;

	Temp = FloatNum;
	if (FloatNum < 0) {
		Temp = -(FloatNum);
	}

	return( ((int)((Temp -(float)((int)Temp)) * (1000.0f))));
}

//********************************************************************************************




int initializeTTC()
{
	u32 RegValue;
	u32 LoopCount;
	u32 TmrCtrBaseAddress;
	u32 MatchValue;


	for (LoopCount = 0; LoopCount < TTC_NUM_DEVICES; LoopCount++) {
		/*
		 * Set the timer counter number to use
		 */
		TmrCtrBaseAddress = TimerCounterBaseAddr[LoopCount];

		//Set the clock control register and do not use a clock prescaler
		RegValue = 0;

		XTtcPs_WriteReg(TmrCtrBaseAddress, XTTCPS_CLK_CNTRL_OFFSET,
						  RegValue);


		/*
		 * Set the Match register. This determines the duty cycle of the
		 * waveform. The waveform output will be toggle each time this
		 * value is reached.
		 */
		MatchValue = 0;

		XTtcPs_WriteReg(TmrCtrBaseAddress, XTTCPS_MATCH_0_OFFSET,
				  MatchValue);

		/*
		 * Set the Counter Control Register
		 */
		RegValue =
			~(XTTCPS_CNT_CNTRL_DIS_MASK |
			  XTTCPS_CNT_CNTRL_EN_WAVE_MASK) &
			(XTTCPS_CNT_CNTRL_MATCH_MASK |
			 XTTCPS_CNT_CNTRL_RST_MASK	|
			 XTTCPS_CNT_CNTRL_POL_WAVE_MASK);
		XTtcPs_WriteReg(TmrCtrBaseAddress, XTTCPS_CNT_CNTRL_OFFSET,
				  RegValue);

		/*
		 * Write to the Interrupt enable register. The status flags are
		 * not active if this is not done.
		 */
		XTtcPs_WriteReg(TmrCtrBaseAddress, XTTCPS_IER_OFFSET,
				  XTTCPS_IXR_INTERVAL_MASK);
	}

	return XST_SUCCESS;
}


//********************************************************************************************

void updateLED(int scale) {

	//convert value to 0 - 1
	float dutyCycle  = (float)100/100;

	XTtcPs_WriteReg(XPAR_XTTCPS_0_BASEADDR, XTTCPS_MATCH_0_OFFSET,(dutyCycle * MAX_TIMER_COUNT));

}

//********************************************************************************************

