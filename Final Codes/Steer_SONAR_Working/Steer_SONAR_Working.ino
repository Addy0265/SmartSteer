/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
int central_adc =511;              // need to be determined
int f_dir = 8;
int f_pulse = 11;
int x=0 , y=0 ;
int d , angle = 25;         
#define F_CPU 16000000
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  
float heading;
float headingsum = 0;
float headingDegrees;
float finalangle = 0, refangle = 0;
#define INSTR_PER_US 16                   // instructions per microsecond (depends on MCU clock, 12MHz current)
#define INSTR_PER_MS 16000                // instructions per millisecond (depends on MCU clock, 12MHz current)
#define MAX_RESP_TIME_MS 200      // timeout - ma time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_MS 50 // echo cancelling time between sampling
#define THRESHOLD 50
#define ROT_ANGLE 180
#define PULSE_COUNT 5*ROT_ANGLE
#define PERIOD 65000
#define OPT 4
#define ANGLE_LIMIT 40
#define LEDPIN 13
#define PULPIN 11
#define DIRPIN 8
#define ECHO1 2
#define TRIG1 6
#define ECHO2 3
#define TRIG2 7
#define LEDPIN 13
int DIRPIN1,PULPIN1,flag1,Pulse_Count1;
volatile long result = 0;
volatile unsigned char up = 0;
volatile unsigned char running = 0;
volatile uint32_t timerCounter = 0;
volatile int dist[2] ={400, 400};
volatile int currentAngle = 0;
volatile int count = 0;
volatile int flag = 0;
volatile int Pulse_Count = 0; 

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//BASIC FUNCTIONS
void _delay()
{
     delay(40);
}

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
void setOCR1(int value)
{
     OCR1B = value;
}
void setOCR2(int value)
{
        OCR1A = value;
}
void setICR(int value)
{
     ICR1 = value;
}
void setICR2(int value)
{
        ICR1 = value;
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void timInitPWM()
{ 
        pinMode(PULPIN, OUTPUT);
        pinMode(LEDPIN, OUTPUT);
        pinMode(DIRPIN, OUTPUT);
              TCCR1A = 0;
              TCCR1B = 0;
              TCNT1  = 0;
              TCCR4A = 0;
              TCCR4B = 0;
              TCNT4  = 0;
              TCCR5A = 0;
              TCCR5B = 0;
              TCNT5  = 0;  
              setOCR2(0);
              setICR2(PERIOD);
              TCCR1A |= (1<<COM1A1)|(1<<WGM11); 
              TCCR1B |= (1<<WGM12)|(1<<WGM13);  
              TCCR1B |= (1 << CS10);            
              TIMSK1 |= (1<<TOIE1);
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ISR(TIMER1_OVF_vect)          

{
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
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ISR(INT4_vect)
{
    if (running) 
    {   //accept interrupts only when sonar was started
        if (up == 0) 
        {   // voltage rise, start time measurement
            up = 1;
            timerCounter = 0;
            TCNT3 = 0; // reset timer counter
        }
        else 
        {   // voltage drop, stop time measurement
            up = 0;
           // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
            result = (timerCounter * 65535 + TCNT3) / (58*2);
            running = 0;
        }
    }  
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ISR(INT5_vect)
{
    if (running) 
    {   //accept interrupts only when sonar was started
        if (up == 0) 
        {   // voltage rise, start time measurement
            up = 1;
            timerCounter = 0;
            TCNT3 = 0; // reset timer counter
        }
        else 
        {   // voltage drop, stop time measurement
            up = 0;
           // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
            result = (timerCounter * 65535 + TCNT3) / (58*2);
            running = 0;
        }
    }  
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
     //Serial.println("Entering PotTurn()");
     if(dist[0] < THRESHOLD && dist[1] > THRESHOLD && abs(currentAngle) < ANGLE_LIMIT && current_adc > 100 && current_adc < 950)
     {   
         int check = 0;
         while(!check)
         {
             check = go_to_angle(-10);
         }
         check = 0;
     }
     else if(dist[0] > THRESHOLD && dist[1] < THRESHOLD && abs(currentAngle) < ANGLE_LIMIT  && current_adc > 100 && current_adc < 950)
            {                
                int check = 0;
                while(!check)
                {
                     check = go_to_angle(10);
                }
                check = 0;
            }
           else if(dist[0] < THRESHOLD && dist[1] < THRESHOLD  && current_adc > 100 && current_adc < 950)
                 {
                     setPin(LEDPIN);
                 } 
                else if(dist[0] > THRESHOLD && dist[1] > THRESHOLD  && current_adc > 100 && current_adc < 950)
                      {  
                          resetPin(LEDPIN);
                          if(currentAngle != 0)
                           {
                                int check = 0;
                                while(!check)
                                {
                                     check = go_to_angle(0);
                                }
                                check = 0;
    
                           } 
                      }
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ISR(TIMER3_OVF_vect)
{
        
	if (up) 
	{   // v]oltage rise was detected previously
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
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
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
}
void getPotAngle()
{
      finalangle= analogRead(A0)*0.267 - refangle;
      Serial.print("RefAngle = "); Serial.print(refangle);
      Serial.print("\tAngle = "); Serial.print(finalangle);
}
void changeSAngle(int dir)
{
     Serial.println(" ENTERING changeSAngle()");
     int change, cAngle;
     getPotAngle();
     cAngle = finalangle;    
     if(abs(finalangle) < ANGLE_LIMIT)
    { 
         if(dir == -1)
         {
               setPin(DIRPIN);
               delay(200);
         }
         else if(dir == 1)
         { 
               resetPin(DIRPIN);
               delay(200);
         }
         else
               return;
         flag = 1;
         Pulse_Count = 5*OPT;
         setOCR2(PERIOD/2);
         while(flag);
         getPotAngle();
         change = finalangle - cAngle;
         Serial.println(change);
         while(abs(change) > OPT+2 || abs(change) < OPT-2)
         {
              if(change > 0)
                  changePAngle(1);
              else
                  changePAngle(-1);
        
              getPotAngle();
              change = finalangle - cAngle;
              Serial.print("\tChange = "); Serial.print(change);  
              Serial.print("\tCurrent Angle = "); Serial.println(currentAngle);      
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
     //delay(1); 
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void changeAngle1(int dir)
{
     if(dir == -1)
         setPin(DIRPIN1);
     else if(dir == 1)
         resetPin(DIRPIN1);
     else
         return;
    
     flag1 = 1;
     Pulse_Count1 = 5*OPT;
     setOCR1(PERIOD/2);
     while(flag1);
     //delay(1); 
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void stop_motor(void)
{
    while(1)
    {
        digitalWrite(f_pulse,LOW);
    }
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
int go_to_angle(int angle)
{
    int ret = 0;
    int req_adc = angle/0.267 + central_adc;
    int current_adc = analogRead(A0);
    Serial.print("current angle:");
    Serial.println((current_adc - central_adc)*0.267);
    if(current_adc - req_adc>5)
    { 
       case_1(current_adc,req_adc);
    }
    else if(current_adc - req_adc<-5)
    {
       case_2(current_adc,req_adc);  
    }
    else 
    {
        ret =1;
        delay(300);
        Serial.println(current_adc-req_adc);
        Serial.println("stopped");
    }
    return ret;
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void case_1( int current_adc, int req_adc)
{
    changeAngle(1);
    Serial.print(current_adc-req_adc);
    delay(5);

}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void case_2( int current_adc, int req_adc)
{
    changeAngle(-1);
    Serial.print(current_adc-req_adc);
    delay(5);
}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void case_3(int current_adc, int req_adc)
{
    delay(300);
    Serial.println(current_adc-req_adc);
    Serial.println("stopped");
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void moveforward(int angle)
{
    
    for(int i=0; i<=angle/OPT; i++)
    {
        Serial.print("Going Forward 3 Degrees");
        Serial.print("       ");
        Serial.println(i);
        changeAngle1(1);
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
void setup()
{
    pinMode(ECHO1, INPUT);
    pinMode(TRIG1, OUTPUT); 
    pinMode(ECHO2, INPUT);
    pinMode(TRIG2, OUTPUT);
    //pinMode(LEDPIN, OUTPUT);
    Serial.begin(9600);
    Serial1.begin(9600);
    setPin(DIRPIN);
    setPin(DIRPIN1);
    noInterrupts();           // disable all interrupts 
    timInitPWM();
    tim3_Init();
    interrupts();             // enable all interrupts
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop()
{
    getPotAngle();
    int check = 0;
    while (!check)
    {
        check = go_to_angle(0);
        //Serial.print(" CURRENT ADC VALUE ");
        //Serial.println(analogRead(A0));
    }
    //check = 0;
  /*while(currentAngle > -20)
  {
    changeSAngle(1);
    Serial.print("\tCurrent Angle = "); Serial.println(currentAngle);  
  }
  Serial.print("\tCurrent Angle = "); Serial.println(currentAngle);  */	
      //Serial.print("NOW DETEDTING");
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
      Serial.print("SONAR-1 :"); Serial.print(dist[0]); Serial.print(" mm");
      Serial.print("\tSONAR-2 :"); Serial.print(dist[1]); Serial.print(" mm");
      Serial.print("\tCurrentAngle :"); Serial.println(currentAngle);
      PotTurn();
     Serial.print("current_adc = ");
     current_adc = analogRead(A0);
      Serial.println(current_adc);
    /*check = 0;
    while(!check)
    {
        check = go_to_angle(0);  // 20 means ccw from central_angle.. as in cartesian convention.. -20 means cw
    }
   // delay(50);
    Serial.print("code_stop_adc = ");
    Serial.println(analogRead(A0)); 
    Serial.print("current_adc = ");
    //delay(2000);*/
    //stop_motor();
}



