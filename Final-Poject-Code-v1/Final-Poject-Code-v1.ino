// Final Poject Code v1
// 4/25/24
//##################################################################################
// Names
// - Nathan Mcginnis
// - 
// - Ian Goff
//##################################################################################
// Requirements
// [] Clock, time and date
// [] Water sensor High/Low detection
//    [] Serial output when Low
// [] Tempurature sensor High/Low detection
//    [] Outputs voltage to digital pin(#?), goes to motor
//    [] Serial outputs the date and time
// [] On/Off button detection
// [] Stepper motor movement
//    [] Reads High/Low for left and right, moves motor
// [] Tempurature and humidity constant serial output
//##################################################################################

#define RDA 0x80
#define TBE 0x20 // <-- Lab 7,5 Values

// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0; // <-- Lab 7,5 Values
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;


void setup() {
  // put your setup code here, to run once:
  
  // Start the UART
  U0Init(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
 * UART FUNCTIONS
 */
void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char getChar()
{
  return *myUDR0;
}
void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
