// Final Poject Code v1
// 4/25/24

//##################################################################################
// ---NAMES---
// - Nathan Mcginnis
// - Chase Pruit
// - Ian Goff

//##################################################################################
// ---REQUIREMENTS---
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
// ---DEFINITIONS---

#define RDA 0x80
#define TBE 0x20 // <-- Lab 7,5 Values

#include <LiquidCrystal.h>
//LCD Display

#include <RTClib.h>
//Library for clock

#include <Servo.h>

// ---LCD VARIABLES---
INTERUPT_SPECIFICATIONS = 0;

// ---WATER SENSOR VARIABLES---
int tempwatervalue = 0;
int waterthreshold = 0;
int ventPos = 0;
int temp = 0;

//##################################################################################
/* ---PINS---
Start/Stop
2

WATER SENSOR
A15

LCD DISPLAY
//HAVE TO DECIDE WHAT PINS TO USE FOR LCD

STEPER MOTOR

DC MOTOR

*/
//##################################################################################
// ---POINTERS---
// Helps with pin mapping -->https://docs.arduino.cc/retired/hacking/hardware/PinMapping2560/

// UART PPOINTERS
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0; // <-- Lab 7,5 Values
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// TIMER POINTERS


// WATER SENSOR POINTER
/*
volatile unsigned char* port_watersensor = (unsigned char*) 0x; 
volatile unsigned char* ddr_watersensor  = (unsigned char*) 0x; 
volatile unsigned char* pin_watersensor  = (unsigned char*) 0x;
*/
//##################################################################################

//Start/Stop Pointers
volatile unsigned char *const Button = (volatile unsigned char *)0x2D;
DDRE &= ~(1 << PE4); //sets PWM 2 pin as an input
//bool previousState = false;
volatile bool Interupt = false;

//For LCD
RTC_DS1307 rtc;

void setup() {
  U0Init(9600); // Start the UART

  //sets interup to trigger for rising edge)
EICRA |= (1 << ISC01);
EICRA &= ~(1 << ISC00);
EIMSK |= (1 << INTO); //Sets interupt to pin 2

  // Water Sensor Setup
  *ddr_watersensor &= 0xEF; //sets PK7(A15) to inupit

  //Needs to be changed to UART
  Serial.begin(9600);
  Environment.begin();

  pinMode(PIN_UP, INPUT);
  pinMode(PIN_DOWN, INPUT);
  pinMode(PIN_SERVO_POW, OUTPUT);
  myservo.attach(4);
  pinMode(PIN_FAN, OUTPUT);
  
}

void loop() {

  if(Interupt){
void = Clock();
if (ButtonState = true){
ButtonState = false;
}
else{
  ButtonState = true;
}
Interupt = false;
  }

//for start/stop
//bool ButtonState = (*Button & (1 << PE4)) != 0;

//runs code when button is pressed and was previously giving 0
if (ButtonState = true/*&& !previousState*/){
//for display - change inputs when we figure that out
temp = fanControl();
void = Display(int a, int b);
ventControl();


}
//previousState = ButtonState;
}
//UART FUNCTIONS
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

void display(int a, int b){
  
lcd.begin(16, 2);
lcd.setCursor(0,0);
//Change temp to string that contains temps once we make it
lcd.print(TEMP)
lcd.setCursor(0, 8);
//Change HUMIDITY to string that contains humidity once we make it
lcd.print(HUMIDITY"%");
lcd.setCursor(0, 1);
//Change if statement vairables once we decide on them
if(WATER_LEVEL_VARIABLE < INTERUPT_SPECIFICATIONS){
lcd.print("Water Level Low!");
}}

void Clock(){

DateTime time = rct.now();

Serial.printLn(Time);
Serial.print(time.hour(), DEC);
Serial.print(":");
Serial.print(time.minute(), DEC);
Serial.print(":");
Serial.print(time.second(), DEC);
Serial.printLn(Date);
Serial.print(time.day(), DEC);
Serial.print(":");
Serial.print(time.month(), DEC);
Serial.print(":");
Serial.print(time.year(), DEC);

}

void water.sensor(){
  // int return?

    //tempwatervalue = analogRead();

    if (tempwatervalue > waterthreshold) {
        WATER_LEVEL_VARIABLE = 1;
      } 

    else {
        WATER_LEVEL_VARIABLE = 0;
      }

      return WATER_LEVEL_VARIABLE;
}

#define PIN_UP 6 //UP Button
#define PIN_DOWN 7 //DOWN Button
#define PIN_SERVO_POW 8 //Analog to control servo speed
void ventControl(){

  for(digitalRead(PIN_UP) == HIGH){
    if(ventPos << 90){
      ventPos++;
    }
    digitalWrite(PIN_SERVO_POW, HIGH); 
    myservo.write(ventPos);
    delay(50);
    
  }

  digitalWrite(PIN_SERVO_POW, LOW);
  

  for(digitalRead(PIN_DOWN) == HIGH){
    if(ventPos << -90){
      ventPos--;
    }
    digitalWrite(PIN_SERVO_POW, LOW); 
    myservo.write(ventPos);
    delay(50);
    
  }

  digitalWrite(PIN_SERVO_POW, LOW);

}

#define PIN_FAN 4 //Turns Fan ON and OFF
int  fanControl(){

  temp = Environment.readTemperature(); //READS TEMPERATURE

  if(temp >> 25){
    digitalWrite(PIN_FAN, HIGH); //Sends power to relay to popwer DC fan motor from external power source
  }
  else(){
    digitalWrite(PIN_FAN, LOW)
  }
  return temp;
}

