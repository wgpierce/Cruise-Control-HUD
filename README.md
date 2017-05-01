# Cruise-Control-HUD
Cruise control HUD MiniProject for ECE 362 and Purdue Spark Challenge


## Setup

Configuration and project files have been omitted since they carry no useful information
and are a pain to track

If you do want to use this code yourself, using CodeWarrior, do the following:

Open CodeWarrior, and navigate through these generic dialogs.
The setting we used are right below the corresponding generic tag.

File -> New Project -> \<family\> -> \<specific member\> -> \<whichever connections\> -> Next -> Finish                  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; HCS12C FAMILY &nbsp;&nbsp;&nbsp;&nbsp; MC9S12C32 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; USB BDM Multilink 

After that, you can just copy the code from the main.c file in this repository to replace the main.c in 
your projectFolder/Sources/ folder
		
## Technologies used

#### Hardware
- MC9S12C32 microcontroller board
- OBD to UART Cable
- LIDAR
- Custom PCB with power wired to OBD board
- Shift Registers
- 4x7-segment Displays
- Printrbot Simple Metal

#### Software
- CodeWarrior
	- USBMLT
- Eagle
- Autodesk Fusion

See Documentation for more specific details
