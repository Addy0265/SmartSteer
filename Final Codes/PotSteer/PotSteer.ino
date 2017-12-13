/*
 * AVR_Sonar.c
 *
 * Created: 4/2/2015 7:23:19 PM
 *  Author: Adarsh Kosta
 */ 


#define F_CPU 16000000
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float heading;
float headingsum = 0;
float headingDegrees;
float finalangle = 0, refangle = 0;

#define INSTR_PER_US 16                   // instructions per microsecond (depends on MCU clock, 12MHz current)
#define INSTR_PER_MS 1600                // instructions per millisecond (depends on MCU clock, 12MHz current)
#define MAX_RESP_TIME_MS 20      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_MS 50 // echo cancelling time between sampling

#define THRESHOLD 20


#define ROT_ANGLE 180
#define PULSE_COUNT 5*ROT_ANGLE
#define PERIOD 65000
#define OPT 6
#define ANGLE_LIMIT 40

#define LEDPIN 13
#define PULPIN 11
#define DIRPIN 8

#define ECHO1 2
#define TRIG1 6
#define ECHO2 3
#define TRIG2 7
#define LEDPIN 13

volatile long result = 0;
volatile unsigned char up = 0;
volatile unsigned char running = 0;
volatile uint32_t timerCounter = 0;
volatile int dist[2] ={400, 400};

volatile int currentAngle = 0;
volatile int count = 0;
volatile int flag = 0;
volatile int Pulse_Count = 0; 

void setPin(int pin)
{
	digitalWrite(pin, HIGH);
}

void resetPin(int pin)
{
	digitalWrite(pin, LOW);
}

double getMax(double a, double b)
{
	if(abs(a) > abs(b))
	return a;
	else
	return b;
}

void setOCR(int value)
{
   OCR1A = value; 
}
void setICR(int value)
{
   ICR1 = value; 
}

void timInitPWM()
{ 
  pinMode(PULPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
   
  setOCR(0);     // compare match register 16MHz/256/2Hz           /// Motor Pulse given @ 10kHz
  setICR(PERIOD);
  
  TCCR1A |= (1<<COM1A1)|(1<<WGM11); //Fast PWM Mode with ICR1 as TOP, Clear on Compare Match
  TCCR1B |= (1<<WGM12)|(1<<WGM13);  
  TCCR1B |= (1 << CS10);            // 8 prescaler 
  TIMSK1 |= (1<<TOIE1);
}

ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  //digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);   // toggle LED pin
  
  if(count < Pulse_Count && flag == 1)
  {  
    count++;
  }
  else
  {
    flag = 0;
    setOCR(0);
    count = 0;
    Pulse_Count = 0;
  }  
}

void gotoAngle(int g_Angle)  //Loaded Function should not be called at a high rate 
{
   int deltaAngle = g_Angle - currentAngle;
   
   if(deltaAngle != 0)
   {
     if(deltaAngle < 0)
     {
       setPin(DIRPIN);
       deltaAngle = -deltaAngle; 
     }
     else
     {
        resetPin(DIRPIN); 
     }
     flag = 1;
     Pulse_Count = 5*deltaAngle;
     setOCR(PERIOD/2);
     while(flag); 
     currentAngle = g_Angle; 
     delay(g_Angle*2);            // delay for ensuring the motor reaches given angle.    
   }
}

void changeAngle(int dir)
{
     if(dir == -1)
     setPin(DIRPIN);
     else if(dir == 1)
     resetPin(DIRPIN);
     else
     return;
     
     flag = 1;
     Pulse_Count = 5*OPT;
     setOCR(PERIOD/2);
     while(flag);
     if(dir == 1)
     currentAngle -=OPT;
     else if(dir == -1)
     currentAngle +=OPT;
     delay(1);  
}

void changePAngle(int dir)
{
     if(dir == -1)
     setPin(DIRPIN);
     else if(dir == 1)
     resetPin(DIRPIN);
     else
     return;
     
     flag = 1;
     Pulse_Count = 5*4;
     setOCR(PERIOD/2);
     while(flag);
}

void getPotRefAngle()
{
  refangle = analogRead(A0)*0.267;
  Serial.print("RefAngle = "); Serial.println(refangle);
}

void getPotAngle()
{
  finalangle= analogRead(A0)*0.267 - refangle;
  //Serial.print("RefAngle = "); Serial.print(refangle);
  
  
}

void changeSAngle(int dir)
{
     int change, cAngle;
     getPotAngle();
     cAngle = finalangle;
     
     if(abs(cAngle) < ANGLE_LIMIT)
    { 
       if(dir == -1)
       setPin(DIRPIN);
       else if(dir == 1)
       resetPin(DIRPIN);
       else
       return;
       
       flag = 1;
       Pulse_Count = 5*OPT;
       setOCR(PERIOD/2);
       while(flag);
       
       getPotAngle();
       change = finalangle - cAngle;
       
       while(abs(change) > OPT+2 || abs(change) < OPT-2)
       {
          if(change > 0)
          changePAngle(1);
          else
          changePAngle(-1);
        
          getPotAngle();
          change = finalangle - cAngle;
          Serial.print("\tChange = "); Serial.print(change);  
          //Serial.print("\tCurrent Angle = "); Serial.println(currentAngle);   
          Serial.print("\tCurrentAngle :"); Serial.print(currentAngle);
          Serial.print("\tPotAngle = "); Serial.println(finalangle);   
       }
        if(dir == 1)
       currentAngle -=OPT;
       else if(dir == -1)
       currentAngle +=OPT;
       Serial.println("Angle Reached");
    }
    else 
    Serial.println("Potentiometer at LIMIT..!!");   
}

// timer overflow interrupt, each time when timer value passes 255 value
ISR(TIMER3_OVF_vect)
{
        
		if (up) 
		{       // voltage rise was detected previously
                timerCounter++; // count the number of overflows
                // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
                uint32_t ticks = timerCounter * 65535 + TCNT3;
                uint32_t max_ticks = (uint32_t)MAX_RESP_TIME_MS * INSTR_PER_MS; // this could be replaced with a value instead of multiplying
                if (ticks > max_ticks) 
				{
                        // timeout
                        up = 0;          // stop counting timer values
                        running = 0; // ultrasound scan done
                        result = -1; // show that measurement failed with a timeout (could return max distance here if needed)
                }
        }
		
    //PORTB ^= (1<<PORTB7);
	    
}

ISR(INT4_vect)
{
  if (running) 
  { //accept interrupts only when sonar was started
    if (up == 0) 
    { // voltage rise, start time measurement
        up = 1;
        timerCounter = 0;
        TCNT3 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result = (timerCounter * 65535 + TCNT3) / (58*2);
        running = 0;
     }
  }
  //PORTB ^= (1<<PORTB7);
  
}

ISR(INT5_vect)
{
  if (running) 
  { //accept interrupts only when sonar was started
    if (up == 0) 
    { // voltage rise, start time measurement
        up = 1;
        timerCounter = 0;
        TCNT3 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result = (timerCounter * 65535 + TCNT3) / (58*2);
        running = 0;
     }
  }
  //PORTB ^= (1<<PORTB7);
  
}


void enableSonar(int num)
{
  if(num == 1)
 {
   // turn on interrupts for INT4, connect Echo to INT4
  EIMSK = (1 << INT4); // enable interrupt on any(rising/droping) edge
  EICRB = (1 << ISC40);      // Turns on INT4
 } 
 if(num ==2)
 {
  // turn on interrupts for INT5, connect Echo to INT5
  EIMSK = (1 << INT5); // enable interrupt on any(rising/droping) edge
  EICRB = (1 << ISC50);      // Turns on INT5
 }
}

void tim3_Init()
{
  TCCR3A = 0;
  TCCR3B = 0;
  
  TCCR3B |= (1<<CS31); // select internal clock with no prescaling
  TCNT3 = 0; // reset counter to zero
  TIMSK3 = 1<<TOIE3; // enable timer interrupt 
}

void sonar(int num) 
{
  if(num == 1)
  {
  resetPin(TRIG1);
  delayMicroseconds(1);
  setPin(TRIG1);
  delayMicroseconds(10);
  resetPin(TRIG1);
  delayMicroseconds(1);	
  }
  else if(num == 2)
  {
  resetPin(TRIG2);
  delayMicroseconds(1);
  setPin(TRIG2);
  delayMicroseconds(10);
  resetPin(TRIG2);
  delayMicroseconds(1);
  }
  running = 1;			    
}


void PotTurn()
{  
 if(dist[0] < THRESHOLD && dist[1] > THRESHOLD && abs(currentAngle) < ANGLE_LIMIT)
 { 
   //getMagCurrentAngle(); 
  Serial.println("IN - 1"); 
   changeSAngle(1);
 }
 else if(dist[0] > THRESHOLD && dist[1] < THRESHOLD && abs(currentAngle) < ANGLE_LIMIT)
 {
   //getMagCurrentAngle();
   Serial.println("IN - 2"); 
   changeSAngle(-1);
 }
 else if(dist[0] < THRESHOLD && dist[1] < THRESHOLD)
 {
   //changeAngle(1);
   setPin(LEDPIN);
 } 
 else if(dist[0] > THRESHOLD && dist[1] > THRESHOLD)
 {
   //getMagCurrentAngle();  
   resetPin(LEDPIN);
   if(currentAngle != 0)
   {
     Serial.print("\tHERE");
    if(currentAngle < 0)
    changeSAngle(-1);
    else
    changeSAngle(1);
   }
    Serial.print("THERE");
   
 }
}

void setup()
{
  pinMode(ECHO1, INPUT);
  pinMode(TRIG1, OUTPUT); 
  pinMode(ECHO2, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  setPin(DIRPIN);
  noInterrupts();
  timInitPWM();
  tim3_Init();
  interrupts(); // enable all(global) interrupts	
  
  Serial.begin(9600);
    
  getPotRefAngle();
  currentAngle = 0;
  
  getPotRefAngle();
  Serial.print("RefAngle+ : "); Serial.println(refangle);
  
  gotoAngle(-135+refangle);
  getPotRefAngle();
  Serial.print("RefAngle+ : "); Serial.println(refangle);	
}


void loop()
{ 
  //gotoAngle(40);
  currentAngle = 0;
 /* 
  int i;
  for(i=0; i<3; i++)	
  {
    enableSonar(1);
    sonar(1); // launch measurement
    while(running == 1); 
    delay(DELAY_BETWEEN_TESTS_MS);
    dist[0] = result;
    enableSonar(2);
    sonar(2);
    while(running == 1);   
    delay(DELAY_BETWEEN_TESTS_MS);
    dist[1] = result;
    Serial.print("\tSONAR-1 :"); Serial.print(dist[0]); Serial.print(" mm");
    Serial.print("\tSONAR-2 :"); Serial.print(dist[1]); Serial.println(" mm");
    delay(1);
  }    
  
  
  //getMagCurrentAngle();
  PotTurn();   
 */
    								
}
