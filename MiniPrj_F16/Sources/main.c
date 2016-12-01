/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2016
***********************************************************************
 Section 1 	   			 		  			 		  		
 
 Team ID: 01

 Project Name: Cruise Control HUD

 Team Members:

   - Team/Doc Leader: Patrick May      Signature: ______________________
   
   - Software Leader: William Pierce   Signature: ______________________

   - Interface Leader: < ? >     Signature: ______________________

   - Peripheral Leader: < ? >    Signature: ______________________


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************
Section 2 	   			 		  			 		  		

Table of Contents

  Description                    Section
  --------------------------------------
  Metadata and Academic Honesty  1
  Table of Contents              2
  Objective                      3
  Success Criteria               4
  Coding Styles                  5
  Progress Report                6
  TODO section                   7
  Code                           8

Method od implementation is in separate word document

***********************************************************************
Section 3
    
  The objective of this Mini-Project is to .... < ? >


***********************************************************************
Section 4
  
 List of project-specific success criteria (functionality that will be
 demonstrated):

 1.

 2.

 3.

 4.

 5.
 
***********************************************************************
Section 5
  
  Project-specific coding styles:
  
 * Naming convention is snake_case
  
 * Tabs are used to determine indentation level
 
 * Active low values are suffixed with _N
  
 * #defines are UPPERCASE 
  
 * Function definitions have parenthesis on line after 
   function signature as 
   
   baz foo(bar)
   {
   
   }
 
 * 
 
 * 
 
 * 
 
   
   
***********************************************************************
Section 6

  Progress report:
  
  Date code started: Nov 11, 2016

  Note that progress is also tracked within git project

  Update history (add an entry every time a significant change is made)
  (append this list downwards):

  Data    Name      Update
  ------------------------------------------
  Nov 11  Will      Wrote shift register and LED driver

  Nov 12  Will      finished multidigit operation and
                    adjusted timing parameters for LED driver
  
  Nov 18  Will      Changed LED driver to not use a delay
                    wrote rudimentary LIDAR driver 
  
  Nov 30  Will      (PCB now made and soldered)
                    fixed many port declarations
                    tested LED up/down/dash
                    disable MISO in SPI for use as GPIO pin
                    set slave select as input to prevent shorting
                    double-buffer LEDs (shift out 2 digits at a time)
                    reverse digit display order                             
                    invert polarities for the parts that are actually on the board
                    attempt software low-pass filter
  
          Tyler
		  
		      Pat
		  
		  
		  
		  
		  

***********************************************************************
Section 7

  TODO tasks
  
  Person to do this task : Tasks to be done 
  -----------------------------------------
  
  <Will>                   write low pass filter in software for LIDAR

  <Patrick>                write OBD interface driver

  <Tyler>                  documentation

************************************************************************
Section 8
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h" /* derivative-specific definitions */
#include <mc9s12c32.h>  /* internal port location delcarations */

/* Special ASCII characters */
#define CR 0x0D		// ASCII return 
#define LF 0x0A		// ASCII new line 


/* PORT DECLARATIONS */
/***********************************************************************
  Internal logic of the program is the same except for
  ports used. If different ports are to be used, set
  appropriate delcarations here and initializations in
  initialization section
************************************************************************
*/


/* Serial-to-parallel shift register controls */  

#define SR_MR_N PTAD_PTAD4   //shift register master reset
#define SR_HCLK PTM_PTM5 //shift register clock input
#define SR_IN   PTM_PTM4 //shift register serial input
#define SR_OE_N PTM_PTM2 //shift register output enable
                           //disable MISO for this

#define SR_TCLK PTM_PTM1 //storage register clock input
                           //disable slave select (SS) and set
                           //to input to prevent shorting
                           //because double counted on the
                           //schematic

#define LIDAR_trigger_N PTT_PTT3
#define LED_down_N PTT_PTT2
#define LED_dash_N PTT_PTT1
#define LED_up_N   PTT_PTT0

#define LIDAR_PWM ATDDR0H

/* LED position select masks */

//AND PTT with this one
#define D_NONE 0x0F

//OR PTT with these
#define DIGIT0 0b10000000
#define DIGIT1 0b01000000
#define DIGIT2 0b00100000
#define DIGIT3 0b00010000


/* Manual LED decoding, based on abel file
                A B C D E F G DP
[ 0, 0, 0, 0]->[1,1,1,1,1,1,0];
[ 0, 0, 0, 1]->[0,1,1,0,0,0,0];
[ 0, 0, 1, 0]->[1,1,0,1,1,0,1];
[ 0, 0, 1, 1]->[1,1,1,1,0,0,1];
[ 0, 1, 0, 0]->[0,1,1,0,0,1,1];
[ 0, 1, 0, 1]->[1,0,1,1,0,1,1];
[ 0, 1, 1, 0]->[1,0,1,1,1,1,1];
[ 0, 1, 1, 1]->[1,1,1,0,0,0,0];
[ 1, 0, 0, 0]->[1,1,1,1,1,1,1];
[ 1, 0, 0, 1]->[1,1,1,1,0,1,1];
[ 1, 0, 1, 0]->[1,1,1,0,1,1,1];

*/

#define ZERO    0b11111100
#define ONE     0b01100000
#define TWO     0b11011010
#define THREE   0b11110010
#define FOUR    0b01100110
#define FIVE    0b10110110
#define SIX     0b10111110
#define SEVEN   0b11100000
#define EIGHT   0b11111110
#define NINE    0b11110110
#define DECIMAL 0b00000001

/* END PORT DELCARATIONS */


/* All functions after main declared here */
//LED-related functions
void LED_wait(void);
void shift_out(char);
void send_byte(char);
void print_digit(char);
void select_disp(char);
void print_number(unsigned short, unsigned short);

//LIDAR-related functions
unsigned char get_LIDAR(void);
void sample_LIDAR(void);

//Debugging & Utility functions
void wait(int);
char inchar(void);
void outchar(char);

//SCI-related functions
void transmit_string(char[]);
void receive_string(char[], char);
void transmit_char(char x);
char search_buffer(char str[], char *returnVal);
void clear_buffer(void);
void wait_for_response(char numBytes);


//OBD board-related functions
void initialize_OBD(void);
void request_speed(void);

/* Global Variable declarations */
char leftpb_flag	= 0;  // left pushbutton flag
char rghtpb_flag	= 0;  // right pushbutton flag
char prevleftpb = 0;    // previous pushbutton states
char prevrghtpb	= 0;

#define TSIZE 81	// transmit buffer size (80 characters)
char tbuf[TSIZE];	// SCI transmit display buffer
char tin	= 0;	// SCI transmit display buffer IN pointer
char tout	= 0;	// SCI transmit display buffer OUT pointer

char rbuf[TSIZE];	// SCI recieve display buffer
char rin	= 0;	// SCI receive display buffer IN pointer
char rout	= 0;	// SCI receive display buffer OUT pointer

<<<<<<< HEAD
//LIDAR variables
unsigned int distance;
#define BETA 4

unsigned char new_data;
unsigned int smooth_dist = 0;

//ODB interfacing variables
char currSpeed = 0; //Current speed value
char speedRequested = 0;
char responseByte[] = {0x1D, 0x00}; //Response byte of the obd board
char searchVal = 0;

#define COUNT_LIMIT 100
int dist_count = 0;

//LED state variable
#define NUM_DIGITS 4
char cur_digit = 0;

 	   		
/*	 	   		
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void)
{
/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40;  //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, no interrupts */
  SCIBDH = 0x00; //set baud rate to 9600
  SCIBDL = 0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 = 0x00; //$9C = 156
  SCICR2 = 0x0C; //initialize SCI for program-driven operation
  DDRB   = 0x10; //set PB4 for output mode
  PORTB  = 0x10; //assert DTR pin on COM port
  SCICR2_RIE = 1;   //Need to set recieve interrupts on
      

/* Initialize the SPI to 6 Mbs */
  SPIBR  = 0x10; //see documentation for initialization  
  SPICR1 = 0b01011101; //LSB out first
  SPICR2 = 0b00001001; //SS not used
                 //enable bidirectional mode
                 //and so disable MISO                 
    
/* Initialize RTI for 2.048 ms interrupt rate */	
  RTICTL = 0x48; //value pulled from timing excel sheet
  CRGINT_RTIE = 1;  //enable/reset RTI
  
/* Initialize TIM Ch 7 (TC7) for periodic interrupts at 50 Hz for each 
    7-segment display = every 5 ms */
    
  TSCR1 = 0x80; //enable timer
  TSCR2 = 0x0E; //set prescale to 64
  TIOS =  0x80; //set ch7 for output compare
  TIE =   0x80; //enable ch7 interrupt
  TC7 =   1875; //trigger value for 50ms interrupts for each LED 
  //TC7 = (24E6 / 64) / x Hz 	   			 		  			 		  		
                //the value here determines the refresh rate of the LEDs
                //50Hz is the experimental minumum for the LED refresh 
                //rate to not be noticeable to the human eye
               
                //Note that the longer this is, the less time
                //other parts of the program have to run


/* Initialize digital I/O port pins */

/* PORT INITIALIZATIONS */
  /* Initialize PWM  */
  
  
  /*  Initialize ATD   */
  ATDCTL2 = 0x80; //normal program driven operation
  ATDCTL3 = 0x08; //sample only channel 0, non-FIFO, non-frozen
  ATDCTL4 = 0x80 | ATDCTL4; //8 bit resolution, use nominal scalar values
            
  //  Initialize Port AD pins 6 and 7 for use as digital inputs
  DDRAD  = 0x10; //program port AD pin 4 for output (SR_MR_N)     
  ATDDIEN= 0x00; //program PAD7 and PAD6 pins as digital inputs
     
  DDRT   = 0xFF; //set port T as output

  DDRM   = 0xF7; //set port M pins as output
                 //set slave select (SS) as input and disabled
                 //set MISO pin as output
  
  
/* END PORT INITIALIZATIONS */


/* Initialize states of peripheral devices */   
  
  PTT = PTT & D_NONE;  //LED display to display digit 0
 
  SR_OE_N = 1;
  SR_OE_N = 0;   //enable output
  
  SR_MR_N = 0;
  SR_MR_N = 1;   //disable reset
                 
    
  //turn on LIDAR for continuous measurement
  LIDAR_trigger_N = 0;
  
  //Turn lights off initially
  LED_down_N = 1;
  LED_dash_N = 1;
  LED_up_N   = 1; 
  	      
}

	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void)
{
  DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;
  initialize_OBD();	
  for(;;) {

    if(!speedRequested){ //If we've gotten the speed, time to ask for it again
    request_speed();
    speedRequested = 1;
    }

    if(search_buffer(responseByte, &searchVal)){
      if((((searchVal+1) % TSIZE) < rin) || ((rout > rin) && (searchVal+1 < rin+TSIZE))){ //Make sure we'ver received the second byte
        currSpeed = rbuf[(searchVal+1) % TSIZE]; //Get byte after response byte
        clear_buffer();
        speedRequested = 0;
      }
    }

  }
     
}

/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void) 
{ 
  
  sample_LIDAR();
  // clear RTI interrupt flag
	CRGFLG = CRGFLG | 0x80;
}

/*
***********************************************************************                       
  TIM interrupt service routine	  		
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
  //print_number(distance, distance);
  print_number(2345,6789);


  // clear TIM CH 7 interrupt flag 
  TFLG1 = TFLG1 | 0x80;
}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
  int statusReg = 0;
    //Ready for new data
  
  if(SCISR1_RDRF == 1) {
  //Recieved character, put it in buffer
    rbuf[rin] = SCIDRL;
    rin = (rin + 1) % TSIZE;
  }else{
    //Need to transmit character      
    if(tin != tout){
        while(!SCISR1_TDRE){ } //Time to send out the next character
        SCIDRL = tbuf[tout];
        tout = (tout + 1) % TSIZE;
    } else { 
        SCICR2_SCTIE = 0; //Finished transmitting, disable interrupt 
    }
  }
  statusReg = SCISR1; //Clear SCI register flags
}

/*
***********************************************************************
  shiftout: Transmits the character x to external shift 
            register using the SPI.  
***********************************************************************
*/
 
void shift_out(char x)
{  
  int i;
  while (SPISR_SPTEF == 0);
  //LEDs are active low, so invert logic
  SPIDR = x ^ 0xFF;
  //SPIDR = 0x00;
  for (i = 0; i < 15; i++);
}

/*
*********************************************************************** 
  send_byte: writes character x to the LED
***********************************************************************
*/

void send_byte(char x)
{
  // shift out character
  shift_out(x);
  // pulse shift register clock line low->high->low
  SR_TCLK = 0;
  SR_TCLK = 1;
  SR_TCLK = 0;
   
}

/*
*********************************************************************** 
  print_digit: writes character x to the LED
***********************************************************************
*/

void print_digit(char x) 
{
  switch (x) {
    case 1:
      x = ONE;
      break;
    case 2:
      x = TWO;
      break;
    case 3:
      x = THREE;
      break;
    case 4:
      x = FOUR;
      break;
    case 5:
      x = FIVE;
      break;
    case 6:
      x = SIX;
      break;
    case 7:
      x = SEVEN;
      break;
    case 8:
      x = EIGHT;
      break;
    case 9:
      x = NINE;
      break;
    default:
      x = ZERO;
  }
  
  send_byte(x);
  
}

/*
*********************************************************************** 
  shift_disp: shifts LED display to the desired position
              select non 0-3 digit to deselect all positions
***********************************************************************
*/

void select_disp(char cur_digit)
{
  PTT = PTT & D_NONE; //clear all select digits
  
  //select desired digit
  switch (cur_digit) {
    case 0:
      PTT = PTT | DIGIT0;
      break;
    case 1:
      PTT = PTT | DIGIT1;
      break;
    case 2:
      PTT = PTT | DIGIT2;
      break;
    case 3:
      PTT = PTT | DIGIT3;
      break;
    default:
      break;
  }
  
}

/*
*********************************************************************** 
  print_number: displays one digit of two binary encoded (non-BCD) 
                numbers on the two LED displays
***********************************************************************
*/

void print_number(unsigned short x, unsigned short y)
{
  char i = 0;
  short exp = 10;
  short last_exp = 1;
  
  cur_digit = (char)((cur_digit + 1) % NUM_DIGITS);
  
  for (; i < cur_digit; i++, last_exp = exp, exp *= 10);
  select_disp(NUM_DIGITS - 1 - cur_digit); //mirror the digits
  
  print_digit((char)((x % exp) / last_exp));
  print_digit((char)((y % exp) / last_exp));
  
}

/*
*********************************************************************** 
  get_LIDAR: triggers ATD AN0 and reads value from that as the LIDAR
             value
***********************************************************************
*/

unsigned char get_LIDAR() 
{
  //note LIDAR is set to continuously read,
  //so no need to activate it  
  ATDCTL5 = 0x00;
  while (ATDSTAT0_SCF == 0);  
  return LIDAR_PWM;  
}

void sample_LIDAR(void)
{
  
  distance = get_LIDAR();
  return;
  
  new_data = get_LIDAR();
  smooth_dist = (smooth_dist << BETA) - smooth_dist;
  smooth_dist += new_data;
  smooth_dist >>= BETA; 
  
  if (++dist_count >= COUNT_LIMIT) {
    dist_count = 0;
    distance = smooth_dist;
  }
    
  
  /*
  
  //average the LIDAR value, effectively low-pass filtering it  
  //TODO: write actual low pass filter
  count = (short)((count + 1) % COUNT_LIMIT);
  distance_total += get_LIDAR();
  
  if (count == 0) {
    distance = (short)(distance_total / COUNT_LIMIT);
    distance_total = 0;
  }
  */
  
  
}

/*
*********************************************************************** 
  wait: waits for approximately n milliseconds
        note this can get interrupted, if not
        called from an interrupt
***********************************************************************
*/

void wait(int n)
{
  int i;
  for (;n > 0; n--) {
    for (i = 2000; i > 0; i--);
  }
}

//Places string to transmit into the buffer and sets transmit interrupt
void transmit_string(char str[]){
  int i = 0;
  while(*(str+i) != 0) { //Load string into buffer
    tbuf[tin] = (*(str+i));
    tin = (tin + 1) % TSIZE;
    i++;
  }
  //Enable interrupts
    //tbuf[tin] = 0x00;//Insert end character
    //tin = (tin + 1) % TSIZE;
  SCICR2_SCTIE = 1;
}

//Reads string from buffer into memory location
void receive_string(char str[],char strlen) {
  int i = 0;
  while(i < strlen) {
    str[i] = rbuf[rout];
    rout = (rout + 1) % TSIZE;
    i++;
  }
  //str[i] = 0x00; //Denote end of string
}

//Waits for a response of a specified length
void wait_for_response(char numBytes){
  while(rin < rout + numBytes){};
}
//Clears circular buffer
void clear_buffer(){
  rout = rin;
}

//Searches buffer for a string
char search_buffer(char str[], char *returnVal){
  char i = rout;
  char currentStrVal = 0;
  while(i != rin){
    if(rbuf[i] == str[currentStrVal]){
      currentStrVal++;
      if(str[currentStrVal] == 0){
        //Reached end of search string, found answer
        *returnVal = i-currentStrVal+1;
        return 1;
      }
    }
    i = (i + 1) % TSIZE;
  }
  return 0;
}

//Initializes OBD board 
void initialize_OBD(){
  //Send atz
  transmit_string("atz");
  wait(1000);
  clear_buffer(); //Responds with firmware no. don't need
  //Send atsp0
  transmit_string("atsp0");
  wait(1000);
  clear_buffer();
  //Wait for "OK" 
}

//Requests speed
void request_speed(){
  char outstring[3];
  outstring[0] = 0x01;
  outstring[1] = 0x0D;
  outstring[2] = 0x00; //End of string
  transmit_string(outstring);  
//Send 01 0D
//Wait for response
}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
    while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}
