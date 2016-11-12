/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2016
***********************************************************************
 Section 1 	   			 		  			 		  		
 
 Team ID: < ? >

 Project Name: Cruise Control HUD

 Team Members:

   - Team/Doc Leader: < Patrick May >      Signature: ______________________
   
   - Software Leader: < William Pierce >      Signature: ______________________

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

          Tyler
		  
		      Pat
		  
		  
		  
		  
		  

***********************************************************************
Section 7

  TODO tasks
  
  Person to do this task : Tasks to be done 
  -----------------------------------------
  Will                     finish driver for multidigit operation
  
  <Will>                   adjust timing parameters 
  
  <>                       write LIDAR driver 

  <>                       write OBD interface driver



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
#define RCLK PTT_PTT2
#define SRCLR_N PTT_PTT3

/* Docking Board LED bit selectors */
#define leftLED PTT_PTT1
#define rghtLED PTT_PTT0


/* END PORT DELCARATIONS */

/* LED position select masks */

#define DIGIT0 0b01111111
#define DIGIT1 0b10111111
#define DIGIT2 0b11011111
#define DIGIT3 0b11101111

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


/* All functions after main initialized here */
void shiftout(char);
void send_byte(char);
void print_digit(char);
void shift_disp(void);
short to_bcd(short);
void print_number(short);

void wait(int);
char inchar(void);
void outchar(char);


/* Global Variable declarations */
char leftpb	= 0;  // left pushbutton flag
char rghtpb	= 0;  // right pushbutton flag
char prevleftpb = 0; // previous pushbutton states
char prevrghtpb	= 0;

char count = 0;
short num = 0;

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
         
/* Initialize the SPI to 6 Mbs */
  SPIBR  = 0x10; //see documentation for initialization  
  SPICR1 = 0x5D; //LSB out first
  SPICR2 = 0x00; //normal operation
  
/* Initialize RTI for 2.048 ms interrupt rate */	
  RTICTL = 0x48; //value pulled from timing excel sheet
  CRGINT_RTIE = 1;  //enable/reset RTI
  
/* Initialize TIM Ch 7 (TC7) for periodic interrupts every 1.000 ms  */
  
   TSCR1 = 0x80; //enable timer
   TSCR2 = 0x0D; //set prescale to 32
   TIOS =  0x80; //set ch7 for output compare
   TIE =   0x80; //enable ch7 interrupt
   TC7 =   750;  //trigger value for 1ms interrupts  
  	 	   			 		  			 		  		

/* Initialize digital I/O port pins */

/* PORT INITIALIZATIONS */


  //  Initialize Port AD pins 6 and 7 for use as digital inputs
  DDRAD  = 0x00; //program port AD for input mode
  ATDDIEN= 0xC0; //program PAD7 and PAD6 pins as digital inputs
     
  DDRT   = 0xFF; //set port T as output


  
/* END PORT INITIALIZATIONS */


/* Initialize states of peripheral devices */   
  
  PTT = DIGIT2;  //LED display to display digit 0
 
  SRCLR_N = 0;   //initially clear LED
  SRCLR_N = 1;
  
  
  	      
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

  for(;;) {
  
    wait(1000);

    num++;

    if (leftpb) {
      //update display for each
      leftpb = 0; 
      count = (char)((count + 1) % 10);
      print_digit(count);

      leftLED = leftLED ^ 1;

    }

    if (rghtpb) {
      rghtpb = 0;
      //shift through displays
      shift_disp();

      print_digit(count);

      rghtLED = rghtLED ^ 1;

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
  //Debounce buttons
  //Note this asserts the button flag upon pushing the button down
  char currLeft = PORTAD0_PTAD7;
  char currRght = PORTAD0_PTAD6;
  
  if (prevleftpb == 1) {
    if (currLeft == 0) {
      leftpb = 1;
    }
  }
  
  if (prevrghtpb == 1) {
    if (currRght == 0) {
      rghtpb = 1;
    }
  }

  //set last states
  prevrghtpb = currRght;
  prevleftpb = currLeft;
  
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
  // clear TIM CH 7 interrupt flag 
 	TFLG1 = TFLG1 | 0x80; 
  
  print_number(num);


}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
 


}


/*
***********************************************************************
  shortwait: Delay for approx .02 ms
***********************************************************************
*/

void shortwait()
{
  int i;
  for (i = 0; i < 900; i++);
}

/*
***********************************************************************
  shiftout: Transmits the character x to external shift 
            register using the SPI.  
***********************************************************************
*/
 
void shiftout(char x)
{  
  int i;
  while (SPISR_SPTEF == 0);
  SPIDR = x;
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
  shiftout(x);
  // pulse LCD clock line low->high->low
  RCLK = 0;
  RCLK = 1;
  RCLK = 0;
   
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
  shift_disp: shifts LED display to the next position to the left 
***********************************************************************
*/

void shift_disp()
{
  cur_digit = (char)((cur_digit + 1) % 4); 
  PTT = PTT | 0xF0; //clear all select digits
  
  //select desired digit
  switch (cur_digit) {
    case 0:
      PTT = PTT & DIGIT0;
      break;
    case 1:
      PTT = PTT & DIGIT1;
      break;
    case 2:
      PTT = PTT & DIGIT2;
      break;
    case 3:
      PTT = PTT & DIGIT3;
      break;
    default:
      PTT = PTT & DIGIT0;
  }
  
}


/*
*********************************************************************** 
  to_bcd: converts binary encoded number as returned bcd number 
***********************************************************************
*/

short to_bcd(short x) {
  
  
  return x;
}


/*
*********************************************************************** 
  print_number: displays a binary encoded (non-BCD) number on the
                LED display
***********************************************************************
*/

void print_number(short x)
{
  char i = 0;
  short exp = 10;
  short last_exp = 1;
  char curr = 0;
  for (; i < 4; i++, exp *= 10) {
    shift_disp();
    //use modulo to cut off upper values 
    //use integer division to cut off lower values 
    curr = (char)((x % exp) / last_exp);
    print_digit(curr);
    shortwait(); //wait for a bit so it can be displayed
    
    last_exp = exp;
  }
}


/*
*********************************************************************** 
  wait: waits for approximately n milliseconds
***********************************************************************
*/

void wait(int n)
{
  int i;
  for (;n > 0; n--) {
    for (i = 2000; i > 0; i--);
  }
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