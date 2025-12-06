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
 * @file    NXPCUP-MICROE-2021.cpp
 * @brief   Application entry point.
 */


extern "C"
{
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

//hier die SD-card
#include "sdcard/sdmmc_config.h"

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

//kPit1Period ist die Periodenlaenge des Timers in ms, normalerweise ist diese 1ms
//das main Intervall soll 34ms sein
#define	DELTA_T			 34
#define K_MAIN_INTERVAL (DELTA_T / kPit1Period)
#define PI				 3.141592653589793
//Anzahl der Magnete pro Radumdrehung
#define kNbPole 		 6
//IMAGEXMOD ist die Schrittweite der Pixel in x Richtung, d.h. jeder wievielte Pixel verwendet werden soll
#define IMAGEXMOD		 4

//hier 8 Programme = DIP Schalter 1 bis 3
#define	PROGRENNEN 		 0
#define PROGSDDEBUG	 	 1

#define PROGTESTSPECIAL	 2
#define PROGTESTRADLENK	 3
#define PROGTESTBESCHL	 4
#define PROGTESTBREMS	 5
#define PROGFOLLOWOBJECT 6
#define PROGFOLLOWLINE   7
//hier die Zustaende
#define ZINIT			 1
#define ZSTART			 2
#define ZRUN			 3
#define ZSTOP			 4
#define ZHALT			 5


// Zustandsvariablen
static Int8 Programm, Programm_old;
static Int8 Zustand, Zustand_old;
static bool Startbutton,Startbutton_old;
static bool Button2,Button2_old;
static bool Motoron;
static float Pot1,Pot2;

// Table containing the image of the digital camera,
static UInt16 sImage[106];
static Int16 diffImage[106];

//SD-card buffer Sachen
#if(SD_ENABLED)

#define SD_DATA_BLOCK_COUNT (1U)
/*! @brief Start data block number accessed in card */
#define SD_DATA_BLOCK_START (100U)
/*! @brief Data buffer size. */
#define SD_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * (2*SD_DATA_BLOCK_COUNT+1U))
SDK_ALIGN(uint8_t g_SDdataWrite[SD_BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
//nun die Definition der SD-card
sd_card_t g_sd;

#endif

//Wagenwerte
float Masse=1320.0;		//in Gramm
float Spurbreite=0.135; //in Meter
float Radstand=0.175;	//in Meter
float Shoehe=0.04;		//Schwerpunkthoehe ueber Boden in Meter
float Sabstand=0.06;	//Schwerpunktabstand vor Hinterachse in Meter
float Kamerahoehe=0.4; //in Meter ueber dem Boden
float Radradius=0.03;	//in Meter

// Measurement of the accelerometer and the magnetometer
static SRAWDATAEnum sAccel;   // in g
static SRAWDATAEnum sMagneto; // in micro teslas

//SD-card buffer Sachen
#if(SD_ENABLED)

#define SD_DATA_BLOCK_COUNT (1U)
/*! @brief Start data block number accessed in card */
#define SD_DATA_BLOCK_START (100U)
/*! @brief Data buffer size. */
#define SD_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * (2*SD_DATA_BLOCK_COUNT+1U))
SDK_ALIGN(uint8_t g_SDdataWrite[SD_BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
//nun die Definition der SD-card
sd_card_t g_sd;

#endif

//Wagenwerte
float Masse=1320.0;		//in Gramm
float Spurbreite=0.135; //in Meter
float Radstand=0.175;	//in Meter
float Shoehe=0.04;		//Schwerpunkthoehe ueber Boden in Meter
float Sabstand=0.06;	//Schwerpunktabstand vor Hinterachse in Meter
float Kamerahoehe=0.4; //in Meter ueber dem Boden
float Radradius=0.03;	//in Meter

//Datenaustausch Strukturen
typedef enum laneElement{
	undefined,
	straight,
	curveL,
	curveR,
	crossing,
	chicane
} laneElement;

/*
typedef struct laneData {
	float errorBuffer[MA_WINDOW_SIZE];
	float currentError;
	int LaneCenter;
	laneElement detectedLaneElement;
} laneData;
*/
Int8 lese_Programm(void);
short sprintfr8(char *ptr,int zahl, const char *str);
void zeige_Wert(int wert);
void signal_init(void);

#if (SD_ENABLED)

short write2SD(sd_card_t *card, const char *str);
short sdprintf8(sd_card_t *card,int zahl,const char *str);
void logPixyVectors(sd_card_t *card, const pixyLineVector (&vec)[2], int time);
#endif

Int8 lese_Programm(void);
short sprintfr8(char *ptr,int zahl, const char *str);
void zeige_Wert(UInt8 wert);
void signal_init(void);

#if (SD_ENABLED)

short write2SD(sd_card_t *card, const char *str);
short sdprintf8(sd_card_t *card,int zahl,const char *str);

#endif

/*
 * @brief   Application entry point.
 */
int main(void)
{
#if (SD_ENABLED)
	//mit einem lokalen Zeiger agieren ist einfacher
	sd_card_t *card = &g_sd;
#endif

	//der genutzte Timer
	static Int16 sDelay;

	bool doneinitflag;
	Int16 algorith,startflag,zeigewert,testi;

	//ErrorCode for ErrorHandling
	int errorCode = 0;

	//Werte die gesetzt werden
	float steer;

	float aDuty;
	float aUMotLeft,aUMotRight;
	// Measuring speed
	float aSpeedMotLeft;
	float aSpeedMotRight;

	// Sensor value
	SRAWDATAEnum aAccel;   // in g
	SRAWDATAEnum aMagneto; // in micro teslas
	float aYaw;			   // in degree
	float aRoll;		   // in degree
	float aPitch;		   // in degree
	float aAngVel_X=0.0;		//mdps = milli degree per second ?
	float aAngVel_Y=0.0;		//mdps = milli degree per second ?
	float aAngVel_Z=0.0;		//mdps = milli degree per second ?
	float Winkelsum_x,Winkelsum_y,Winkelsum_z;

	//Variablen fuer die genutzten Programme
	clock_t timeakt,timemax;
	UInt16 timermark1;
	Int16 difftimer1,difftimer2,mdifftimer1,mdifftimer2;

	pixyLineVector vectorData[2] = {
	    {0, 0, 0, 0},
	    {0, 0, 0, 0}
	};


	static float ILeft[10]={0,0,0,0,0,0,0,0,0,0};
	static float ILeftmittel=0;
	static int   ILeftindex=0;
	static float IRight[10]={0,0,0,0,0,0,0,0,0,0};
	static float IRightmittel=0;
	static int   IRightindex=0;
	float ILeftmitteldummy,IRightmitteldummy,IMittelfrueh;
	float dummy,nmax;
	float Usoll1,Usoll2,deltat;
	Int16   i,j,z;

	//initieren der structs
	//laneData ld = {{0}, 0.0f, 38, undefined};
	//laneData  pld = &ld;

	doneinitflag=false;
	Programm=Programm_old=-1;
	Zustand=Zustand_old=ZINIT;
	Startbutton_old=false;
	Button2=false;
	Motoron=false;


#if (SD_ENABLED)
	BOARD_InitSDPins();
#endif

	//BOARD_InitBootPins();
	//BOARD_InitBootClocks();
	//BOARD_InitBootPeripherals();

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

	// Enable IRQ at the CPU -> Primask
	__enable_irq();

	// UART 4 monitoring image
	mRs232_Setup();
	mRs232_Open();

	//Motor init
	Motor_Init();


	sDelay = mDelay_GetDelay(kPit1, K_MAIN_INTERVAL);
	//PRINTF("Hello World\n");
	printf("HellO World: %ld %d\n",clock(),CLOCKS_PER_SEC);

	Pixy2SPI_SS pixy;
	int pixyInitReturnCode = pixy.init();
	//Pixy Error ins erste Bit vom errorCode codieren
	if (pixyInitReturnCode != 0) {
		errorCode |= (1 << 0);
	};


	pixy.getVersion();
	pixy.version->print();
	printf("HellO World: %ld\n",clock());
	pixy.setLED(0, 255, 0);
	pixy.setLamp(1, 1);
	pixy.changeProg("line");
	//checking for errors after setup
	if(errorCode != 0){
		zeige_Wert(errorCode);
		zeige_Wert(errorCode);
		zeige_Wert(errorCode);
	};



	//--------------------------------------------------------------------
	// Infinite loop -> round robin
	//--------------------------------------------------------------------
	for (;;)
	{
		if (mDelay_IsDelayDone(kPit1, sDelay))
		{
			mDelay_ReStart(kPit1, sDelay, K_MAIN_INTERVAL);

			//aktuelles Programm bestimmen
			Programm=lese_Programm();
			if(Programm!=Programm_old) {
				Zustand_old=Zustand;
				doneinitflag=false;
				Zustand=ZINIT;
				Programm_old=Programm;
			}

			//nun die Eingabebuttons lesen
			Startbutton=mSwitch_ReadPushBut(kPushButSW2);
			Button2=mSwitch_ReadPushBut(kPushButSW1);
			Pot1=mAd_Read(kPot1);
			Pot2=mAd_Read(kPot2);
			Motoron=mSwitch_ReadSwitch(kSw4);


			//jetzt die Programme durchgehen
			//Die Zustaende werden dann in jedem Programm beruecksichtigt

			if(Programm==PROGRENNEN) {
				if(Zustand==ZINIT) {
					//kritische Groessen signalisieren

					//Dinge die man nur einmal machen moechte
					if(doneinitflag==false) {
						steer = -0.2;
						mTimer_SetServoDuty(0,steer);
						usleep(3049000);   // 0.5 Sekunden warten

						steer = 0.2;
						mTimer_SetServoDuty(0,steer);
						usleep(3049000);   // 0.5 Sekunden warten
						steer = 0.0;
						mTimer_SetServoDuty(0,steer);
						doneinitflag=true;
					}

					//START bei Startbutton=true
					if((Startbutton==true)&&(Startbutton_old==false)) {
						mLeds_Write(kMaskLed2,kLedOff);
						mLeds_Write(kMaskLed3,kLedOn);
						mLeds_Write(kMaskLed4,kLedOff);

						printf("START!\n");

						Zustand_old=Zustand;
						Zustand=ZSTART;

						doneinitflag=false;
						startflag=0;
					}
				}
				else if(Zustand==ZSTART) {
					//hier Startdinge erledigen

					//RUN bei Startbutton=true
					if((Startbutton==true)&&(Startbutton_old==false)) {
						if(startflag<2) {
							startflag++;
						}
						else {
							mLeds_Write(kMaskLed1,kLedOff);
							mLeds_Write(kMaskLed2,kLedOff);
							mLeds_Write(kMaskLed3,kLedOff);
							mLeds_Write(kMaskLed4,kLedOff);

							usleep(30490000);   // 5 Sekunden warten


							mLeds_Write(kMaskLed1,kLedOn);

							startflag=0;
							Zustand_old=Zustand;
							Zustand=ZRUN;
						}
					}
				}
				else if(Zustand==ZRUN) {
					timeakt=(clock_t)(testi*K_MAIN_INTERVAL);	//get time
					if(			((Startbutton==true)&&(Startbutton_old==false))
							  ||(timeakt>10000)) {	//goes to stop after 10000 timeunit (sec, ms??)
						printf("STOP!\n");
						Zustand_old=Zustand;
						Zustand=ZSTOP;
					}
					else {

						//hier steering für Programm Rennen
						//should be refactored to return error -> different func to convert error to steer

						steer = Pixy2_LaneTracking(pixy);

						if(steer == 0){
							mLeds_Write(kMaskLed4, kLedOn);
						}else if(steer != 0){
							mLeds_Write(kMaskLed4, kLedOff);
						};

						mTimer_SetServoDuty(0,steer);
						//Pot2 is beeing read after Program is being read
						Motor_SetSpeed(Pot2);
					}

					testi++;
				}
				else if(Zustand==ZSTOP) {


					Motor_SetSpeed(-1);	//stopping motor

					Zustand_old=Zustand;
					Zustand=ZHALT;
				}
				else if(Zustand==ZHALT) {
					//Parameter per LED anzeigen
					if((Button2==true)&&(Button2_old==false)) {
						zeigewert=(++zeigewert)%2;
					}

					if(zeigewert==0) {
					}
					else if(zeigewert==1) {
						//vierfacher nmax Wert

						mLeds_Write(kMaskLed2,kLedOff);
						mLeds_Write(kMaskLed3,kLedOff);
						mLeds_Write(kMaskLed4,kLedOn);
					}

					usleep(18000000);

					//Reset bei Startbutton=true
					if((Startbutton==true)&&(Startbutton_old==false)) {
						printf("RESET!\n");
						Zustand_old=Zustand;
						Zustand=ZINIT;
					}

				}

				Startbutton_old=Startbutton;
				Button2_old=Button2;
			}
			else if(Programm == PROGSDDEBUG){
				if(Zustand==ZINIT) {
									//kritische Groessen signalisieren

									//Dinge die man nur einmal machen moechte
									if(doneinitflag==false) {
										steer = -0.2;
										mTimer_SetServoDuty(0,steer);
										usleep(3049000);   // 0.5 Sekunden warten

										steer = 0.2;
										mTimer_SetServoDuty(0,steer);
										usleep(3049000);   // 0.5 Sekunden warten
										steer = 0.0;
										mTimer_SetServoDuty(0,steer);

										doneinitflag=true;
									}

									//START bei Startbutton=true
									if((Startbutton==true)&&(Startbutton_old==false)) {
										mLeds_Write(kMaskLed2,kLedOff);
										mLeds_Write(kMaskLed3,kLedOn);
										mLeds_Write(kMaskLed4,kLedOff);

										printf("START!\n");

										Zustand_old=Zustand;
										Zustand=ZSTART;

										doneinitflag=false;
										startflag=0;
									}
								}
								else if(Zustand==ZSTART) {
									//hier Startdinge erledigen

									//RUN bei Startbutton=true
									if((Startbutton==true)&&(Startbutton_old==false)) {
										if(startflag<2) {
											startflag++;
										}
										else {
											mLeds_Write(kMaskLed1,kLedOff);
											mLeds_Write(kMaskLed2,kLedOff);
											mLeds_Write(kMaskLed3,kLedOff);
											mLeds_Write(kMaskLed4,kLedOff);

											usleep(30490000);   // 5 Sekunden warten

					#if (SD_ENABLED)
											SYSMPU_Enable(SYSMPU, false);
											BOARD_SD_Config(card, NULL, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

											if (SD_Init(card))
											{
												mLeds_Write(kMaskLed2,kLedOn);
												printf("\nSD card init failed.\n");
												return 0;
											}
					#endif
											testi=1;
											mLeds_Write(kMaskLed1,kLedOn);

											startflag=0;
											Zustand_old=Zustand;
											Zustand=ZRUN;
										}
									}
								}
								else if(Zustand==ZRUN) {
									timeakt=(clock_t)(testi*K_MAIN_INTERVAL);	//get time
									if(			((Startbutton==true)&&(Startbutton_old==false))
											  ||(timeakt>10000)) {	//goes to stop after 10000 timeunit (sec, ms??)
										printf("STOP!\n");
										Zustand_old=Zustand;
										Zustand=ZSTOP;
									}
									else {

										//hier steering für Programm Rennen
										//should be refactored to return error -> different func to convert error to steer

										steer = Pixy2_LaneTrackingDebug(pixy, vectorData);
										if(steer == 0){
											mLeds_Write(kMaskLed4, kLedOn);
										}else if(steer != 0){
											mLeds_Write(kMaskLed4, kLedOff);
										};

										mTimer_SetServoDuty(0,steer);
										//Pot2 is beeing read after Program is being read
										Motor_SetSpeed(Pot2);
				#if (SD_ENABLED)
										//
										logPixyVectors(card, vectorData, timeakt);

				#endif
				#if (SD_ENABLED)
										write2SD(card,"\n");
				#endif

										//Zeitmarke nehmen fuer die Bestimmung der Berechnungsdauer (in ms)
										timermark1=mDelay_GetDelay(kPit1,sDelay);

										//Hier die Berechnungen durchfuehren

										//Zeitdauer der Berechnung bestimmen (in ms)
										difftimer1=timermark1-mDelay_GetDelay(kPit1,sDelay);
									}

									testi++;
								}
								else if(Zustand==ZSTOP) {

				#if (SD_ENABLED)
									write2SD(card,"\n");
									write2SD(card,NULL);
									SD_Deinit(card);
				#endif

									zeigewert=0;
									Motor_SetSpeed(-1);	//stopping motor
									Zustand_old=Zustand;
									Zustand=ZHALT;
								}
								else if(Zustand==ZHALT) {
									//Parameter per LED anzeigen
									if((Button2==true)&&(Button2_old==false)) {
										zeigewert=(++zeigewert)%2;
									}

									if(zeigewert==0) {
									}
									else if(zeigewert==1) {
										//vierfacher nmax Wert
										mLeds_Write(kMaskLed2,kLedOff);
										mLeds_Write(kMaskLed3,kLedOff);
										mLeds_Write(kMaskLed4,kLedOn);
									}

									usleep(18000000);

									//Reset bei Startbutton=true
									if((Startbutton==true)&&(Startbutton_old==false)) {
										printf("RESET!\n");
										Zustand_old=Zustand;
										Zustand=ZINIT;
									}

								}

								Startbutton_old=Startbutton;
								Button2_old=Button2;

			}

		}
	}

	return 0;
}

Int8 lese_Programm(void) {
	Int8 ergebnis=0;
	if(mSwitch_ReadSwitch(kSw1)==true) {
		ergebnis += 4;
	}
	if(mSwitch_ReadSwitch(kSw2)==true) {
		ergebnis += 2;
	}
	if(mSwitch_ReadSwitch(kSw3)==true) {
		ergebnis += 1;
	}
	return(ergebnis);
}

void signal_init(void) {
	mLeds_Write(kMaskLed1,kLedOn);
	mLeds_Write(kMaskLed2,kLedOn);
	mLeds_Write(kMaskLed3,kLedOn);
	mLeds_Write(kMaskLed4,kLedOn);
	usleep(1000000); // ca. 0.164 Sekunden
	mLeds_Write(kMaskLed1,kLedOff);
	mLeds_Write(kMaskLed2,kLedOff);
	mLeds_Write(kMaskLed3,kLedOff);
	mLeds_Write(kMaskLed4,kLedOff);

	return;
}

void zeige_Wert(int wert) {

	//erstmal ein flash
	mLeds_Write(kMaskLed1,kLedOff);
	mLeds_Write(kMaskLed2,kLedOff);
	mLeds_Write(kMaskLed3,kLedOff);
	mLeds_Write(kMaskLed4,kLedOff);
	usleep(1000000);
	mLeds_Write(kMaskLed1,kLedOn);
	mLeds_Write(kMaskLed2,kLedOn);
	mLeds_Write(kMaskLed3,kLedOn);
	mLeds_Write(kMaskLed4,kLedOn);
	usleep(1000000);
	mLeds_Write(kMaskLed1,kLedOff);
	mLeds_Write(kMaskLed2,kLedOff);
	mLeds_Write(kMaskLed3,kLedOff);
	mLeds_Write(kMaskLed4,kLedOff);
	usleep(1000000);

	//dann die unteren 4 Bit
	if(wert&(1<<3)) { mLeds_Write(kMaskLed1,kLedOn);}
	if(wert&(1<<2)) { mLeds_Write(kMaskLed2,kLedOn);}
	if(wert&(1<<1)) { mLeds_Write(kMaskLed3,kLedOn);}
	if(wert&1)	 	{ mLeds_Write(kMaskLed4,kLedOn);}
	usleep(6000000);
	//dann ein flash
	mLeds_Write(kMaskLed1,kLedOff);
	mLeds_Write(kMaskLed2,kLedOff);
	mLeds_Write(kMaskLed3,kLedOff);
	mLeds_Write(kMaskLed4,kLedOff);
	usleep(1000000);
	mLeds_Write(kMaskLed1,kLedOn);
	mLeds_Write(kMaskLed2,kLedOn);
	mLeds_Write(kMaskLed3,kLedOn);
	mLeds_Write(kMaskLed4,kLedOn);
	usleep(1000000);
	mLeds_Write(kMaskLed1,kLedOff);
	mLeds_Write(kMaskLed2,kLedOff);
	mLeds_Write(kMaskLed3,kLedOff);
	mLeds_Write(kMaskLed4,kLedOff);
	//Pause 3 Sekunden besser im aufrufenden Programm
	//usleep(18000000);

	return;
}

/* Die folgende Routine erzeugt aus einer Integerzahl einen ASCII-String und haengt einen String str    */
/* hinten an. Dabei wird die Zahl rechtsbuendig in die ersten INTBREITE Zeichen geschrieben und bis zu  */
/* 15 Zeichen des Strings str angehaengt. Der Speicherplatz von ptr muss also INTBREITE + 16 Byte gross */
/* sein       																							*/

#define INTBREITE 8
short sprintfr8(char *ptr,int zahl, const char *str)
{
	short i,flagm;
	flagm=0;

	i=INTBREITE-1;

	if(zahl<0) { flagm=1; zahl=-zahl; }

	//nun die Zahl in ptr[] rechtsbuendig eintragen
	do {
		ptr[i--]= (char)(zahl%10) + '0';
		zahl/=10;
	}
	while((zahl>0)&&(i>0));
	if(flagm==1)  ptr[i--]='-';

	//nach links mit SPACEs auffuellen
	while(i>=0) ptr[i--]=' ';

	//nun noch str[] anhaengen, aber nicht mehr als 15 Zeichen
	ptr += INTBREITE; i=15;
	while((str!=NULL)&&(*str!=0)&&(i-->0)) *ptr++=*str++;
	*ptr=0;

	return(INTBREITE+15-i);
}

#if (SD_ENABLED)
/* die folgende Routine schreibt einen String in den SD_BUFFER (anhaengend) und wenn dieser			    */
/* SD_DATA_BLOCK_COUNT*FSL_SDMMC_DEFAULT_BLOCK_SIZE==WRITE_SIZE Byte erreicht oder ueberschreitet,		*/
/* wird er auf die SD geschrieben und um den Wert WRITE_SIZE entsprechend verkleinert.                  */
/* wenn man eine NULL als String uebergibt, wird ein Buffer der mindestens WRITE_SIZE Bytes enthaelt	*/
/* soweit geschrieben, dass nur noch weniger als WRITE_SIZE Bytes (ggf. 0) enthalten sind.				*/
/* wenn er >0 und <WRITE_SIZE Zeichen enthaelt, wird mit '\n' aufgefuellt und geschrieben, wenn er kein	*/
/* Zeichen enthaelt geschieht nichts. Die NULL wirkt also wie ein flush!                                */

short write2SD(sd_card_t *card, const char *str)
{
	static unsigned int blockstart = SD_DATA_BLOCK_START;
	static unsigned short bufferanfang=0;
	unsigned short 	i;
	unsigned char c;

	if(str!=NULL) {
		while(((c=*str++)!=0)&&(bufferanfang<SD_BUFFER_SIZE)) g_SDdataWrite[bufferanfang++]=c;
	}
	else {
		while(    (   (bufferanfang>0)&&(bufferanfang<(SD_DATA_BLOCK_COUNT*FSL_SDMMC_DEFAULT_BLOCK_SIZE)) )
				||(   (bufferanfang>(SD_DATA_BLOCK_COUNT*FSL_SDMMC_DEFAULT_BLOCK_SIZE))
				    &&(bufferanfang<(2*SD_DATA_BLOCK_COUNT*FSL_SDMMC_DEFAULT_BLOCK_SIZE))) )
		{
			g_SDdataWrite[bufferanfang++]='\n';
		}
	}

	while(bufferanfang>=(SD_DATA_BLOCK_COUNT*FSL_SDMMC_DEFAULT_BLOCK_SIZE)) {
		if (kStatus_Success != SD_WriteBlocks(card, g_SDdataWrite,blockstart,SD_DATA_BLOCK_COUNT))
		{
			printf("\nWrite multiple data blocks failed.\n");
			return -1;
		}
		else {
			blockstart+=SD_DATA_BLOCK_COUNT;
			bufferanfang -=(SD_DATA_BLOCK_COUNT*FSL_SDMMC_DEFAULT_BLOCK_SIZE);
			i=bufferanfang;
			while(i>0) g_SDdataWrite[i]=g_SDdataWrite[--i+(SD_DATA_BLOCK_COUNT*FSL_SDMMC_DEFAULT_BLOCK_SIZE)];
		}
	}

	return 1;
}

/* Die folgende Routine wird als printf Ersatz zum Schreiben auf die SD-Karte verwendet  		   */
/* Es kann nur eine Integerzahl in ASCII-Darstellung geschrieben werden, welcher bis zu 15 Zeichen */
/* des String str angehaengt werden																   */

short sdprintf8(sd_card_t *card,int zahl,const char *str)
{
	char   string[INTBREITE+20];
	short  k;

	if(str==NULL)
	{
		k=write2SD(card,NULL);
	}
	else if((k=sprintfr8(string,zahl,str))>0)
	{
		write2SD(card,string);
	}
	return k;
}

void logPixyVectors(sd_card_t *card, const pixyLineVector (&vec)[2], int time)
{
    sdprintf8(card, time, "; ");

    for (int i = 0; i < 2; i++) {
        sdprintf8(card, vec[i].m_x0, "; ");
        sdprintf8(card, vec[i].m_y0, "; ");
        sdprintf8(card, vec[i].m_x1, "; ");
        sdprintf8(card, vec[i].m_y1, "; ");
    }
    write2SD(card, "\n");
}

#endif

