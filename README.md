# README for NXP-Cup26 - Team PopCycle

## Branching 

>Using straight forward branching strategy: Main, Develop, feature/, bugfix/, etc.

Branch-Naming: 	
		
		main 	-> Most currenct Version running on Car
		
		develop -> Staging Branch for bringing together feature-branches
		
		feature/<featureName>	-> Branch working on specific feature
		
		bugfix/<bugName>	-> Branch working on fixing specfic bug

## User-Guide

4 Dip switches can be used to specify programs the car runs on ( binary encoded ) 

Currently Only Prog 1 -> "PROGRENNNEN" should be used. ( Switchstate: 0 0 0 1 ) 

### Programs
Each Program steps throgh 5 States which can be programmed: ZINIT, ZSTART, ZRUN, ZSTOP, ZHALT

ZINIT -> for setting stuff ONCE. It automatically progresses into ZSTART state

ZSTART -> Waits for User to press "Startbutton" (kPushButSW2). Then waits for 5 seconds. Turns LED1 on and then progresses into ZRUN state.

ZRUN -> Timed State. Measures timeakt and switches to state ZSTOP after "timeakt" > 10000 ( line 426)

ZSTOP -> Currently stops Motor Motor_SetSpeed(-1). The progresses to state ZHALT

ZHALT -> Can be used to display parameters on the 4 LEDS. After pushing Startbutton again state will be reset and set to ZINIT again, repeating the cycle of a program. 


## Prog 1 "PROGRENNEN"
ZRUN:
	calls lanetracking algo. Steers and sets Motorspeed according to poti2 position ( Position is only read during setup ) 
	Writes Steer parameter to sdcard
