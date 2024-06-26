// Final Poject Code v7
// 5/12/24

//##################################################################################
// ---NAMES---
// - Nathan Mcginnis
// - Chase Pruit
// - Ian Goff

//##################################################################################
// ---REQUIREMENTS---
// [] Clock, time and date
// [] Water sensor High/Low detection
//    [] LCD(Serial?) output when Low
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
#include <Arduino_SensorKit.h>
//LCD Display

#include <RTClib.h>
//Library for clock

#include <Servo.h>
Servo myservo;

// ---LCD VARIABLES---
int INTERUPT_SPECIFICATIONS = 0;

// ---WATER SENSOR VARIABLES---
volatile unsigned int tempwatervalue = 0;
int waterthreshold = 0;
int ventPos = 0;
int temp = 0;
int WATER_LEVEL_VARIABLE = 0;
int hum = 0;


//##################################################################################
// ---POINTERS---
// Helps with pin mapping -->https://docs.arduino.cc/retired/hacking/hardware/PinMapping2560/

// UART PPOINTERS
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0; // <-- Lab 7,5 Values
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// ADC POINTERS
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//##################################################################################

//Start/Stop Pointers
volatile unsigned char *const Button = (volatile unsigned char *)0x2D;
volatile unsigned char* my_DDRE = (unsigned char*)  0x06;
my_DDRE &= ~(1 << PE4); //sets PWM 2 pin as an input
//bool previousState = false;
volatile bool Interupt = false;

//For LCD
RTC_DS1307 rtc;

//CODE FOR LIGHT PIN'S OUTPUTS
volatile unsigned char *const YELLOW = (volatile unsigned char *)0x101;
volatile unsigned char *const GREEN = (volatile unsigned char *)0x2D;
volatile unsigned char *const BLUE = (volatile unsigned char *)0x33;

// LCD pins <--> Arduino pins
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


void setup() {
  U0Init(9600); // Start the UART

  //sets interup to trigger for rising edge)
EICRA |= (1 << ISC01);
EICRA &= ~(1 << ISC00);
EIMSK |= (1 << INTO); //Sets interupt to pin 2

  // Water Sensor Setup
  adc_init();

  //Needs to be changed to UART
  Serial.begin(9600);
  //Environment.begin();

  //pinMode(PIN_UP, INPUT);
  //pinMode(PIN_DOWN, INPUT);
  //pinMode(PIN_SERVO_POW, OUTPUT);
  myservo.attach(11);
  //pinMode(PIN_FAN, OUTPUT);
  
  //sets up pins for lights
DDRH |= (1 << PH4); //sets PIN 7 as an output for yellow
DDRE |= (1 << PE3); //sets PIN 5 as an output for green
DDRG |= (1 << PG5); //sets PIN 4 as an output for blue

  pinMode(6, INPUT);
  pinMode(9, INPUT);
  pinMode(8, OUTPUT);
  myservo.attach(11);

}



unsigned int adctoprint(int printflag = 0) //ADC water sensor value 'filter'
{
volatile unsigned int outnum;
outnum = tempwatervalue;

if(outnum >= 1000){
  putChar(outnum / 1000 + '0');
  printflag = 1;
  outnum = outnum % 1000;
}

if(outnum >= 100 || printflag){
    putChar(outnum / 100 + '0');
  printflag = 1;
  outnum = outnum % 100;
}
if(outnum >= 10 || printflag){
    putChar(outnum / 10 + '0');
  printflag = 1;
  outnum = outnum % 10;
}

return;
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
  *YELLOW &= ~(1 << PH4); //turns off yellow LED
//for display - change inputs when we figure that out
temp = fanControl();
hum = humidity();
watersensor();
void = Display(int temp, int hum);
ventControl();

}
else{
*GREEN &= ~(1 << PE3); //turns off yellow LED
*YELLOW |= (1 << PH4); //TURNS on yellow LED
*BLUE &= ~(1 << PG5); //turns off blue LED
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

void display(int temp, int b){
  
  
lcd.begin(16, 2);
lcd.setCursor(0,0);
//Change temp to string that contains temps once we make it
lcd.print(temp);
lcd.setCursor(0, 8);
//Change HUMIDITY to string that contains humidity once we make it
lcd.print(hum);
lcd.setCursor(0, 1);
//Change if statement vairables once we decide on them
//if(WATER_LEVEL_VARIABLE < INTERUPT_SPECIFICATIONS){
//lcd.print("Water Level Low!");
}


void Clock(){

DateTime Time = rct.now();
putChar(Time);
putChar(time.hour(), DEC);
putChar(":");
putChar(time.minute(), DEC);
putChar(":");
putChar(time.second(), DEC);
putChar(Date);
putChar(time.day(), DEC);
putChar(":");
putChar(time.month(), DEC);
putChar(":");
putChar(time.year(), DEC);

}

void watersensor(){
    adc_read(1); //<-- Sets the input pin (A1)
    tempwatervalue = *my_ADC_DATA;
    adctoprint(tempwatervalue);
    if (tempwatervalue > waterthreshold) {
        WATER_LEVEL_VARIABLE = 1;
      } 

    else {
        WATER_LEVEL_VARIABLE = 0;
      }

      return WATER_LEVEL_VARIABLE;
}

volatile unsigned char *portVentFan = (unsigned char*)  0x15;
*portVentFan |= 0x40; //output fan pin to LOW
*portVentFan ^= 0x40; // XOR to toggle on
void ventControl(){

  while(digitalRead(6) == HIGH){
    if(ventPos << 90){
      ventPos++;
    }
    *portVentFan ^= 0x40; // XOR to toggle on
    myservo.write(ventPos);
    delay(15);
    
  }

  *portVentFan |= 0x40; //output fan pin to LOW
  

  while(digitalRead(9) == HIGH){
    if(ventPos << -90){
      ventPos--;
    }
    *portVentFan ^= 0x40; // XOR to toggle on
    myservo.write(ventPos);
    delay(15);
    
  }

   *portVentFan |= 0x40; //output fan pin to LOW

}

void EnvironmentreadTemperature(volatile unsigned int temtempurature){
  adc_read()
    temptempurature = *my_ADC_DATA;
    adctoprint(tempteurature);
    return temptempurature;
}

//#define PIN_FAN 4 //Turns Fan ON and OFF
volatile unsigned char *portDigFan = (unsigned char*)  0x01;
*portDigFan |= 0x40; //output fan pin to LOW
*portDigFan ^= 0x40; // XOR to toggle on
int  fanControl(){

  temp = EnvironmentreadTemperature(); //READS TEMPERATURE

  if(temp >> 23){
   *portDigFan ^= 0x40; // XOR to toggle on, Sends power to relay to popwer DC fan motor from external power source
  *GREEN &= ~(1 << PE3); //turns off green LED
  *BLUE |= (1 << PG5); //turns on blue LED
  }
  else(){
  *portDigFan |= 0x40; //output fan pin to LOW
  *BLUE &= ~(1 << PG5); //turns off blue LED
  *GREEN |= (1 << PE3); //turns on green LED
  }
  return temp;
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void EnvironmentreadHumidity(volatile unsigned int temphumid){
  adc_read(); //<-- Sets the input pin (A1)
    temphumid = *my_ADC_DATA;
    adctoprint(temphumid);
    return temphumid;
}

int humidity(){
  hum = EnvironmentreadHumidity();
  return hum;
}