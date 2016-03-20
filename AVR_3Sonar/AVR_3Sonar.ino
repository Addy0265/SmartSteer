#define F_CPU 16000000

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#define PI_APP 3.14159265359

#define TICKS_PER_MS 15984               // instructions per millisecond (depends on MCU clock, 12MHz current)
#define MAX_RESP_TIME_MS 40      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_US 200 // echo cancelling time between sampling

#define THRESHOLD 20
#define DIST_LIMIT 500

#define ECHO1 18
#define TRIG1 4
#define ECHO2 2
#define TRIG2 5
#define ECHO3 3
#define TRIG3 6

#define LEDPIN 13

#define NUM_OF_READINGS 5

volatile long result1 = 0;
volatile unsigned char up1 = 0;
volatile unsigned char running1 = 0;
volatile uint32_t timerCounter1 = 0;

volatile long result2 = 0;
volatile unsigned char up2 = 0;
volatile unsigned char running2 = 0;
volatile uint32_t timerCounter2 = 0;

volatile long result3 = 0;
volatile unsigned char up3 = 0;
volatile unsigned char running3 = 0;
volatile uint32_t timerCounter3 = 0;

volatile uint16_t dummy[3] = {0,0,0};

volatile  uint32_t max_ticks = (uint32_t)MAX_RESP_TIME_MS*TICKS_PER_MS; 
//  = 480000 this could be replaced with a value instead of multiplying

volatile int distCnt[3] = {0};
volatile int distData1[NUM_OF_READINGS] = {0}; 
volatile int distData2[NUM_OF_READINGS] = {0};
volatile int distData3[NUM_OF_READINGS] = {0};
volatile int distSum1 = 0, distSum2 = 0, distSum3 = 0;

volatile int dist[3] ={1000, 1000, 1000};

volatile int time = 0, prev_time = 0, exec_time = 0;

volatile int dataCnt = 0;

/***********************************************************************************/
/* BASIC FUNCTIONS */

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

/***********************************************************************************/
/* INTERRUPT HANDLERS */

// Timer overflow interrupt for Left Sonar
ISR(TIMER3_OVF_vect)
{      
    if (up1) 
    {       // voltage rise was detected previously
    timerCounter1++; // count the number of overflows
    // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
    uint32_t ticks1 = timerCounter1 * 65535 + TCNT3;
        if (ticks1 > max_ticks) 
      {
          // timeout
          up1 = 0;          // stop counting timer values
          running1 = 0; // ultrasound scan done
          result1 = 1000; // show that measurement failed with a timeout (could return max distance here if needed)
      }
    }    
}

// Timer overflow interrupt for Center Sonar
ISR(TIMER4_OVF_vect)
{      
    if (up2) 
    {       // voltage rise was detected previously
    timerCounter2++; // count the number of overflows
    // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
    uint32_t ticks2 = timerCounter2 * 65535 + TCNT4;
        if (ticks2 > max_ticks) 
      {
          // timeout
          up2 = 0;          // stop counting timer values
          running2 = 0; // ultrasound scan done
          result2 = 1000; // show that measurement failed with a timeout (could return max distance here if needed)
      }
    }    
}

// Timer overflow interrupt for Right Sonar
ISR(TIMER5_OVF_vect)
{      
    if (up3) 
    {       // voltage rise was detected previously
    timerCounter3++; // count the number of overflows
    // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
    uint32_t ticks3 = timerCounter2 * 65535 + TCNT5;
        if (ticks3 > max_ticks) 
      {
          // timeout
          up3 = 0;          // stop counting timer values
          running3 = 0; // ultrasound scan done
          result3 = 1000; // show that measurement failed with a timeout (could return max distance here if needed)
      }
    }    
}


// Echo Interrupt Handler for Right Sonar
ISR(INT3_vect)
{
  if (running1) 
  { //accept interrupts only when sonar was started
    if (up1 == 0) 
    { // voltage rise, start time measurement
        up1 = 1;
        timerCounter1 = 0;
        TCNT3 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up1 = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result1 = (timerCounter1 * 65535 + TCNT3)/940;
        dummy[0] = TCNT3;
        running1 = 0;
     }
  }
}

// Echo Interrupt Handler for Left Sonar
ISR(INT4_vect)
{ 
  if (running2) 
  { //accept interrupts only when sonar was started
    if (up2 == 0) 
    { // voltage rise, start time measurement
        up2 = 1;
        timerCounter2 = 0;
        TCNT4 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up2 = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result2 = (timerCounter2 * 65535 + TCNT4)/940;
        dummy[1] = TCNT4;
        running2 = 0;
     }
  }
}

// Echo Interrupt Handler for Right Sonar
ISR(INT5_vect)
{
  if (running3) 
  { //accept interrupts only when sonar was started
    if (up3 == 0) 
    { // voltage rise, start time measurement
        up3 = 1;
        timerCounter3 = 0;
        TCNT5 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up3 = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result3 = (timerCounter3 * 65535 + TCNT5)/940;
        dummy[2] = TCNT5;
        running3 = 0;
     }
  }
}

/*************************************************************************************/
/* SONAR FUNCTIONS */

void enableSonar(int num)
{
  if(num == 1)
 {
  // turn on interrupts for INT5, connect Echo to INT5
  EIMSK |= (1 << INT3); // enable interrupt on any(rising/droping) edge
  EICRA |= (1 << ISC30);      // Turns on INT5
 }
  if(num == 2)
 {
   // turn on interrupts for INT4, connect Echo to INT4
  EIMSK |= (1 << INT4); // enable interrupt on any(rising/droping) edge
  EICRB |= (1 << ISC40);      // Turns on INT4
 } 
 if(num == 3)
 {
  // turn on interrupts for INT5, connect Echo to INT5
  EIMSK |= (1 << INT5); // enable interrupt on any(rising/droping) edge
  EICRB |= (1 << ISC50);      // Turns on INT5
 }
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
  running1 = 1;  
  }
  else if(num == 2)
  {
  resetPin(TRIG2);
  delayMicroseconds(1);
  setPin(TRIG2);
  delayMicroseconds(10);
  resetPin(TRIG2);
  delayMicroseconds(1);
  running2 = 1; 
  }    
  else if(num == 3)
  {
  resetPin(TRIG3);
  delayMicroseconds(1);
  setPin(TRIG3);
  delayMicroseconds(10);
  resetPin(TRIG3);
  delayMicroseconds(1);
  running3 = 1; 
  }      
}

void handleObstacle(int cnt)
{  
    while(cnt--)
    {
      // Sonar-1
      if(running1 == 0)
      {
        distData1[distCnt[0]] = result1;
        distCnt[0]++;
        
        if(distCnt[0] == NUM_OF_READINGS)
          distCnt[0] = 0;
        int n;  
        distSum1 = 0;
        for(n=0; n<NUM_OF_READINGS; n++)
        {
          distSum1 += distData1[n]; 
        }  
                
        dist[0] = distSum1/NUM_OF_READINGS;
        
        if(dist[0] > DIST_LIMIT && dist[0] != 1000)
          dist[0] = DIST_LIMIT;
        
        sonar(1); // launch measurement 
      }
      // Sonar-2
      if(running2 == 0)
      {
        distData2[distCnt[1]] = result2;
        distCnt[1]++;
        
        if(distCnt[1] == NUM_OF_READINGS)
          distCnt[1] = 0;
        int n;  
        distSum2 = 0;
        for(n=0; n<NUM_OF_READINGS; n++)
        {
          distSum2 += distData2[n]; 
        }  
                
        dist[1] = distSum2/NUM_OF_READINGS;
        
        if(dist[1] > DIST_LIMIT  && dist[1] != 1000)
          dist[1] = DIST_LIMIT;
        
         sonar(2); 
      }
      
      // Sonar-3
      if(running3 == 0)
      {
        distData3[distCnt[2]] = result3;
        distCnt[2]++;
        
        if(distCnt[2] == NUM_OF_READINGS)
          distCnt[2] = 0;
        int n;  
        distSum3 = 0;
        for(n=0; n<NUM_OF_READINGS; n++)
        {
          distSum3 += distData3[n]; 
        }  
                
        dist[2] = distSum3/NUM_OF_READINGS;
        
        if(dist[2] > DIST_LIMIT  && dist[2] != 1000)
          dist[2] = DIST_LIMIT;
        
         sonar(3); 
      }
    }
}

/*************************************************************************************/
/* INITIALIZATION FUNCTIONS */

void tim3_Init()
{
  TCCR3A = 0;
  TCCR3B = 0;
  
  TCCR3B |= (1<<CS30); // select internal clock with no prescaling
  TCNT3 = 0; // reset counter to zero
  TIMSK3 = 1<<TOIE3; // enable timer interrupt 
}

void tim4_Init()
{
  TCCR4A = 0;
  TCCR4B = 0;
  
  TCCR4B |= (1<<CS40); // select internal clock with no prescaling
  TCNT4 = 0; // reset counter to zero
  TIMSK4 = 1<<TOIE4; // enable timer interrupt 
}

void tim5_Init()
{
  TCCR5A = 0;
  TCCR5B = 0;
  
  TCCR5B |= (1<<CS50); // select internal clock with no prescaling
  TCNT5 = 0; // reset counter to zero
  TIMSK5 = 1<<TOIE5; // enable timer interrupt 
}

void initializeSonar()
{
  int m;
  
  for(m=0; m<NUM_OF_READINGS; m++)
  {
     distData1[m] = 1000;
     distData2[m] = 1000;
     distData3[m] = 1000;
  } 
}

void pinInit()
{
  pinMode(ECHO1, INPUT);
  pinMode(TRIG1, OUTPUT); 
  pinMode(ECHO2, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO3, INPUT);
  pinMode(TRIG3, OUTPUT);
  
  pinMode(LEDPIN, OUTPUT);
}

void Init()
{
  pinInit();
  
  noInterrupts();
  tim3_Init();
  tim4_Init();
  tim5_Init();
  interrupts(); // enable all(global) interrupts  
  Serial.begin(115200);
  
  enableSonar(1);
  enableSonar(2);
  enableSonar(3);
  
  initializeSonar(); 
}

/*****************************************************************************************************************************************************************/
/* PRINT FUNCTIONS */

void _printSonar()
{
   Serial.print(" SONAR-1 :"); Serial.print(dist[0]); Serial.print(" cm");
   Serial.print("  SONAR-2 :"); Serial.print(dist[1]); Serial.print(" cm");
   Serial.print("  SONAR-3 :"); Serial.print(dist[2]); Serial.print(" cm");
   Serial.print("  TCNT3 :"); Serial.print(dummy[0]);
   Serial.print("  TCNT4 :"); Serial.print(dummy[1]);
   Serial.print("  TCNT5 :"); Serial.println(dummy[2]);
}

/*************************************************************************************/
/* SETUP and LOOP */

void setup()
{
  Init();
}

void loop()
{   
  handleObstacle(5);
  _printSonar();    // Sonar data
  delay(20);
}

