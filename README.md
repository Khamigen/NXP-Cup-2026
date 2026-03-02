# README for NXP-Cup26 - Team PopCycle

## Branching 

>Using straight forward branching strategy: Main, Develop, feature/, bugfix/, etc.

Branch-Naming: 	
		
		main 	-> Most currenct Version running on Car
		
		develop -> Staging Branch for bringing together feature-branches
		
		feature/<featureName>	-> Branch working on specific feature
		
		bugfix/<bugName>	-> Branch working on fixing specfic bug

## Programs

> kSwitch 2-4 work as a binary-number encoder. Its used to define a program ranging from 0 to 8 which will be run in the infinte loop of the MC.

0: RACE
1: Timedrace -> like Race but stops after 10 secs aprox.
2: Debug1 -> Level1 Debugging with SingleVectorDetected and FrameOffsetX
3: Debug2 -> Level2 Debugging with additional Vector coordinates on top.
