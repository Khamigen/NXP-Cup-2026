
/*
 * Copyright 2003-20xx Haute �cole ARC Ing�ni�rie, Switzerland. 
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Haute �cole ARC Ing�ni�rie nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    nxpcup_ARC.c
 * @brief   Application entry point.
 */

extern "C"
{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

#include "Modules/mSpi.h"
#include "Modules/mDac.h"
#include "Modules/mAccelMagneto.h"
#include "Modules/mGyro.h"
#include "Modules/mTimer.h"
#include "Modules/mCpu.h"
#include "Modules/mSwitch.h"
#include "Modules/mLeds.h"
#include "Modules/mAd.h"
#include "Modules/mDelay.h"
#include "Modules/mRS232.h"
#include "Modules/mVL6180x.h"

#include "Applications/gInput.h"
#include "Applications/gCompute.h"
#include "Applications/gOutput.h"
}
// popcycle header
#include <Popcycle/Pixy2_LaneTracking.h>
#include <Popcycle/Motor_Control.h>

/* Pixy 2 */
#include "Pixy/Pixy2SPI_SS.h"

#define K_MAIN_INTERVAL (100 / kPit1Period)

// Measuring speed and meaning
static float sSpeedMotLeft;
static float sSpeedMotRight;
#define kSpeedTabSize 100
static UInt16 sSpeedIndex;
static float sSpeedTab[kSpeedTabSize][2];
// Table containing the image (and various infos) of the
// digital camera, the test board is used for the Labview app
static UInt16 sImageTabTest[200];
// Measurement of the accelerometer and the magnetometer
static SRAWDATAEnum sAccel;   // in g
static SRAWDATAEnum sMagneto; // in micro teslas
//static UInt8 sAccelMagnetoStatus;
static float sYaw;   // in degree
static float sRoll;  // in degree
static float sPitch; // in degree

// Angular velocity of gyro in degrees
static float sAngVel_X;
static float sAngVel_Y;
static float sAngVel_Z;

// Measurement of motor current and battery voltage
static float sIMotLeft;
static float sIMotRight;
static float sUBatt;
static bool sFaultLeft;
static bool sFaultRight;

// Distance measured by the LIDAR in mm
static UInt8 sDistFront;

/*
 * @brief   Application entry point.
 */
int main(void)
{

	static UInt8 sPixelValMoy;
	static bool sImageOk = false;
	static Int16 sDly;
	static UInt16 sIntTime = 25000;
	Int32 aWakeIntMain;
	Int8 aCharTab[50];
	UInt32 i = 0;
	bool aRet;
	float aDuty = 0;

	// Table containing the image (and various infos) of the digital camera
	UInt8 aImageTab[200];
	// Sensor value
	SRAWDATAEnum aAccel;   // in g
	SRAWDATAEnum aMagneto; // in micro teslas
	float aYaw;			   // in degree
	float aRoll;		   // in degree
	float aPitch;		   // in degree
	// Measuring speed and meaning
	float aSpeedMotLeft;
	float aSpeedMotRight;

#if (kWithLidar)
	UInt8 aDistFront;
#endif

	//--------------------------------------------------------------------
	// Device and card setup
	//--------------------------------------------------------------------
	// PLL Config --> CPU 100MHz, bus and periph 50MH z
	mCpu_Setup();

	// Config and start switches and pushers
	mSwitch_Setup();
	mSwitch_Open();

	// Config and start of LEDs
	mLeds_Setup();
	mLeds_Open();

	// Config and start of ADC
	mAd_Setup();
	mAd_Open();

	// Config and start of SPI
	mSpi_Setup();
	mSpi_Open();

	// Config and start non-blocking delay by PIT
	mDelay_Setup();
	mDelay_Open();

	// Timer Config for Speed Measurement and PWM Outputs for Servos
	mTimer_Setup();
	mTimer_Open();

	// Setup FXOS8700CQ
	mAccelMagneto_Setup();
	mAccelMagneto_Open();

	// Setup FXAS21002C
	mGyro_Setup();
	mGyro_Open();

	// Config and start of the DAC0 used to drive the driver LED lighting
	mDac_Setup();
	mDac_Open();

	// Setup and start of motor and servo PWM controls and speed measurement
	mTimer_Setup();
	mTimer_Open();

	// Enable IRQ at the CPU -> Primask
	__enable_irq();

	// UART 4 monitoring image
	mRs232_Setup();
	mRs232_Open();

/*	mTimer_SetServoDuty(1, 1);
	SDK_DelayAtLeastUs(15000000, SystemCoreClock); //3s
	mTimer_SetServoDuty(1, -0.6);
	SDK_DelayAtLeastUs(15000000, SystemCoreClock); //
	mTimer_SetServoDuty(1, -0.4);*/
	Motor_Init();


	//mTimer_SetServoDuty(0, 0.6);
	//mTimer_SetServoDuty(0, -0.6);
	//mTimer_SetServoDuty(0, 0);

	// Lidar --> I2C
#if (kWithLidar)
	mVL6180x_Setup();
	mVL6180x_Open();
	mVL6180x_StartRange();
#endif

	// SPI0 --> camera SPI --> reset + idle command
	//mSpi_MLX75306_Reset();

	//--------------------------------------------------------------------
	// Init and calibration of accelerometer and magnetometer
	//--------------------------------------------------------------------
#if (kWithAccel)

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// !!! The calibration takes almost 1mn and you have to turn the car in
	//     all directions during this time of init
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Init of FXOS8700CQ
	mAccelMagneto_Init();
	// Calibration of the offset of the accelerometer
	mAccelMagneto_Accel_Calibration();
	// Calibration of the offset magnetometer
	mAccelMagneto_Mag_Calibration();

	// Init of FXAS21002C
	mGyro_Init();
	// Calibration of the gyro offset
	mGyro_Calibration();
#endif

/*    //Pixycam --> create an instance
	Pixy2SPI_SS pixy;
	pixy.init();
	//pixy.version->print();
	bool first = true;

	aWakeIntMain = mDelay_GetDelay(kPit1, 500 / kPit1Period);

	PRINTF("Hello World\n");
	pixy.setLED(255, 0, 0); // Set RGB LED of Pixy to red
*/
	//Pixy initialization for linetracking Redline to indicate init successful
	Pixy2SPI_SS pixy;
	pixy.init();
	pixy.setLED(255,0,0);
	pixy.changeProg("line");
	//pixy.line.setMode();
	//--------------------------------------------------------------------
	// Infinite loop -> round robin
	//--------------------------------------------------------------------
	for (;;)
	{
		sDistFront = sAngVel_X;

		//-----------------------------------------------------------------
		// Pixy 2: Get Lines
		// This function requests for all lines detected by pixy
		//-----------------------------------------------------------------

		//Note: with a console output the program is buggy while debugging
/*		if (pixy.line.numVectors)
		{
			printf("Detected ");
			printf("%i", pixy.line.numVectors);
			printf("\n");

			for (i = 0; i < pixy.line.numVectors; i++)
			{
				printf("  line  ");
				printf("%i", (int)i);
				printf(": ");
				pixy.line.vectors[i].print();
			}
		}
*/
		float steer = Pixy2_LaneTracking(pixy);
		mTimer_SetServoDuty(0,steer);
		/*if(pixy.line.numVectors >= 2)
		    {
		        // Determine left and right lines
		        auto v1 = pixy.line.vectors[0];
		        auto v2 = pixy.line.vectors[1];
		        //auto v1_avg, v2_avg;
		        int mid1 = (v1.m_x0 + v1.m_x1)/2;
		        int mid2 = (v2.m_x0 + v2.m_x1)/2;

		        int leftX = (mid1 < mid2) ? mid1 : mid2;
		        int rightX = (mid1 < mid2) ? mid2 : mid1;

		        int laneCenterX = (leftX + rightX)/2;
		        int frameCenterX = 39; // Pixy2 line mode width / 2
		        int error = laneCenterX - frameCenterX;
		        float lenkung = error * -0.05;
		        float max = 0.6;
		        if (lenkung>max){
		        	lenkung = max;
		        }
		        else if (lenkung<-max){
		        	lenkung=-max;
		        }
		        // Send error to servo / motor control
		        mTimer_SetServoDuty(0, lenkung); // example proportional control

		    }*/
/* Temporally comment everything beneath
		//--------------------------------------------------------------------
		// Depending on the position of the switch 1 we go from the test state (if = 1, see below) to the automatic state.
		//--------------------------------------------------------------------
		// Automatic mode
		if (mSwitch_ReadSwitch(kSw1) == true)
		{
			gInput_Execute();
			gCompute_Execute();
			gOutput_Execute();

			if (mDelay_IsDelayDone(kPit1, aWakeIntMain))
			{
				mDelay_ReStart(kPit1, aWakeIntMain, K_MAIN_INTERVAL);

				//-----------------------------------------------------------------
				// Start exposition and read the image
				//-----------------------------------------------------------------
				//sIntTime=5;	// 5ms
				//mSpi_MLX75306_StartIntegration(kCamera2, sIntTime);
				//mSpi_MLX75306_ReadPicture(kCamera2, aImageTab);

#if (kWithAccel)
				//-----------------------------------------------------------------
				// Read accelerometer and magnetometer
				//-----------------------------------------------------------------
				mAccelMagneto_ReadData(&aAccel, &aMagneto, &aYaw, &aRoll, &aPitch);
#endif

#if (kWithLidar)

				//--------------------------------------------------------------------
				// Reading distance LIDAR (mm)
				//--------------------------------------------------------------------
				mVL6180x_GetRange(aDistFront);
#endif
				//-----------------------------------------------------------------------------
				// Reading the rotation speed of the motors
				// Motor A = motor left -> negative value = backward, value pos = forward
				// Motor B = right engine
				//-----------------------------------------------------------------------------
				mTimer_GetSpeed(&aSpeedMotLeft, &aSpeedMotRight);
			}
		}
		//--------------------------------------------------------------------
		// Mode test
		//--------------------------------------------------------------------
		else
		{

			// Reading of the accelerometer and the magnetometer
			// Angle in degrees and acceleration in g
#if (kWithAccel)
			aRet = mAccelMagneto_ReadData(&sAccel, &sMagneto, &sYaw, &sRoll, &sPitch);
			aRet = mGyro_ReadData_mDPS(&sAngVel_X, &sAngVel_Y, &sAngVel_Z);
#endif

#if (kWithLidar)

			//--------------------------------------------------------------------
			// Reading the distance measured by the LIDAR (mm)
			//--------------------------------------------------------------------
			mVL6180x_GetRange(&sDistFront);
#endif
			//-----------------------------------------------------------------------------
			// Reading the rotation speed of the motors
			// Motor A = left motor (rpm) -> negative value = backward, value pos = forward
			// Motor B = right engine (rpm)
			//-----------------------------------------------------------------------------
			mTimer_GetSpeed(&sSpeedMotLeft, &sSpeedMotRight);
			sSpeedTab[sSpeedIndex][0] = sSpeedMotLeft;
			sSpeedTab[sSpeedIndex][1] = sSpeedMotRight;
			sSpeedIndex++;
			sSpeedIndex %= kSpeedTabSize;

			// Depending on the position of the switches (switch 2 and 3) the push buttons,
			// the servo, the DC motors and the camera are tested.
			if (mDelay_IsDelayDone(kPit1, sDly) == true)
			{
				mDelay_ReStart(kPit1, sDly, 300 / kPit1Period);

				// Test of LEDS
				mLeds_Toggle(kMaskLed1 + kMaskLed2 + kMaskLed3 + kMaskLed4);

				// Tests of servo
				// The 2 push buttons allow to move in one direction and the other
				if (mSwitch_ReadPushBut(kPushButSW1) == true)
				{
					aDuty += 0.05;
					if (aDuty > 1)
					{
						aDuty = 1;
					}
					mTimer_SetServoDuty(0, aDuty);
				}
				else if (mSwitch_ReadPushBut(kPushButSW2) == true)
				{
					aDuty -= 0.05;
					if (aDuty < -1)
					{
						aDuty = -1;
					}
					mTimer_SetServoDuty(0, aDuty);
				}
				else
				{
					aDuty = 0;
					mTimer_SetServoDuty(0, 0);
				}

				// If the switch 2 is ON the pilot potentiom�tres the engines
				// if the pot1 pilot the exposure time of the cam�ra
				if ((mSwitch_ReadSwitch(kSw2) == true) && (mSwitch_ReadSwitch(kSw3) == false))
				{
					// motor rest
					// Pot1 left engine
					// Pot1 right engine
					mTimer_SetMotorDuty(mAd_Read(kPot1), mAd_Read(kPot2));
				}
				else if ((mSwitch_ReadSwitch(kSw2) == false) && (mSwitch_ReadSwitch(kSw3) == false))
				{
					sIntTime = mAd_ReadCamera(kPot1);
				}
				else
				{
					// Set DAC 0 buffer output value, between 0 and 4095 --> LED driver
					// Between 0 and 100% --> 0 and 1.0
					mDac_SetDac0Output((mAd_Read(kPot1) + 1.0) / 2.0);
				}

				// Reading left and right motor current and battery voltage
				sIMotLeft = mAd_Read(kIHBridgeLeft);
				sIMotRight = mAd_Read(kIHBridgeRight);
				sUBatt = mAd_Read(kUBatt);
				sFaultLeft = mTimer_GetFaultMoteurLeft();
				sFaultRight = mTimer_GetFaultMoteurRight();

				//						// Start exposition � la lumi�re
				//						mSpi_MLX75306_StartIntegration_old(kCamera2,sIntTime);
				//
				//						// Test de la cam�ra
				//						mSpi_MLX75306_ReadPictureTest(kCamera2,sImageTabTest);
				//
				//						// Valeur moyenne des pixels
				//						sPixelValMoy=sImageTabTest[156];

				//-----------------------------------------------------------------------
				// Acquiring the non-blocking image of the camera 1
				// The acquisition time comes from the potentiometers
				//-----------------------------------------------------------------------
				//sImageOk=mSpi_GetImageCam1_Pot(sIntTime,sImageTabTest, &sPixelValMoy);

				//-----------------------------------------------------------------------
				// Acquiring the non-blocking image of camera 2
				// The acquisition time comes from the potentiometers
				//-----------------------------------------------------------------------
				//sImageOk=mSpi_GetImageCam2_Pot(sIntTime,sImageTabTest, &sPixelValMoy);

				/*if(sImageOk==true)
						{
								mRs232_Uart4WriteString("L:");

								// Yellow trace in Labview
								for(i=0;i<143;i++)
									{
										sprintf(aCharTab,"%X,",sImageTabTest[13+i]);
										mRs232_Uart4WriteString(aCharTab);
									}
								// Red trace in Labview
								for(i=0;i<143;i++)
									 {
										 sprintf(aCharTab,"%X",sImageTabTest[13+i]);
										 mRs232_Uart4WriteString(aCharTab);
										 if(i==142)
											 {
												 mRs232_Uart4WriteString("\r\n");
											 }
										 else
											 {
												 mRs232_Uart4WriteString(",");
											 }
									 }
						}
			}
		}*/
	}

	return 0;
}
