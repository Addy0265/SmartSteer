/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
int central_adc =511;              // need to be determined
int f_dir = 8;
int f_pulse = 11;
int x=0 , y=0 ;
int d , angle = 25;
void go_to_ang_step(int angle);
#include <math.h>
#include <Wire.h>
#include <avr/io.h>

#define ROT_ANGLE 180
#define PULSE_COUNT 5*ROT_ANGLE
#define PERIOD 67500
#define OPT 6

#define LEDPIN 13
#define PULPIN 11
#define DIRPIN 8
#define PULPIN1 12
#define DIRPIN1 48

#define ECHO1 2
#define TRIG1 6
#define ECHO2 3
#define TRIG2 7
volatile int currentAngle = 0;
volatile int count = 0;
volatile int flag = 0;
volatile int Pulse_Count = 0;
volatile int count1 = 0;
volatile int flag1 = 0;
volatile int Pulse_Count1 = 0;

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
void setICR(int value)
{
     ICR1 = value;
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void timInitPWM()
{
    pinMode(PULPIN, OUTPUT);
    pinMode(LEDPIN, OUTPUT);
    pinMode(DIRPIN, OUTPUT);
    pinMode(PULPIN1, OUTPUT);
    //pinMode(LEDPIN1, OUTPUT);
    pinMode(DIRPIN1, OUTPUT);
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    //TCCR5A = 0;
    //TCCR5B = 0;
    setOCR(0);     // compare match register 16MHz/256/2Hz           /// Motor Pulse given @ 10kHz
    setOCR1(0);
    setICR(PERIOD);
    //setICR1(PERIOD);
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11); //Fast PWM Mode with ICR1 as TOP, Clear on Compare Match
    TCCR1B |= (1<<WGM12)|(1<<WGM13); 
    TCCR1B |= (1 << CS10);            // 1 prescaler
    //TIMSK1 |= (1 << OCIE1A);          // enable timer compare interrupt
    TIMSK1 |= (1<<TOIE1);
    //TCCR5A |= (1<<COM5A1)|(1<<WGM51); //Fast PWM Mode with ICR5 as TOP, Clear on Compare Match
    //TCCR5B |= (1<<WGM52)|(1<<WGM53); 
    //TCCR5B |= (1 << CS50);            // 8 prescalar
    //TIMSK5 |= (1<<TOIE5);
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
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
        //setOCR1(0);
        count = 0;
        Pulse_Count = 0;
    }
    if(count1 < Pulse_Count1 && flag1 == 1)
    {   
        count1++;
    }
    else
    {
        flag1 = 0;
        setOCR1(0);
        //setOCR1(0);
        count1 = 0;
        Pulse_Count1 = 0;
    }
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*ISR(TIMER5_OVF_vect)          // timer compare interrupt service routine
{
  //digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);   // toggle LED pin
 
    if(count1 < Pulse_Count1 && flag1 == 1)
    { 
        count1++;
    }
    else
    {
        flag1 = 0;
        setOCR1(0);
        count1 = 0;
        Pulse_Count1 = 0;
    } 
}*/
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void gotoAngle(int g_Angle)  //Loaded Function should not be called at a high rate
{
     int deltaAngle = g_Angle - currentAngle;
     Serial.println("GOING FORWARD NOW"); 
     if(deltaAngle != 0)
     {
         if(deltaAngle < 0)
         {
             setPin(DIRPIN1);
             deltaAngle = -deltaAngle;
         }
         else
         {
             resetPin(DIRPIN1);
         }
         flag1 = 1;
         Pulse_Count1 = 5*deltaAngle;
         setOCR1(PERIOD/2);
         while(flag1);
         currentAngle = g_Angle;
         delay(g_Angle*2);            // delay for ensuring the motor reaches given angle.   
     }
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
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
void setup()
{
      pinMode(ECHO1, INPUT);
      pinMode(TRIG1, OUTPUT); 
      pinMode(ECHO2, INPUT);
      pinMode(TRIG2, OUTPUT);
      pinMode(LEDPIN, OUTPUT);
      Serial.begin(9600);
      Serial1.begin(9600);
      setPin(DIRPIN);
      setPin(DIRPIN1);
      noInterrupts();           // disable all interrupts 
      timInitPWM();
      //tim3_Init();
      interrupts();             // enable all interrupts
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop()
{
 /*   Serial.print("current_adc = ");
    Serial.println(analogRead(A0));
    */int check = 0;
    while(!check)
    {
        check = go_to_angle(0);  // 20 means ccw from central_angle.. as in cartesian convention.. -20 means cw
    }
   // delay(50);
    Serial.print("code_stop_adc = ");
    Serial.println(analogRead(A0)); 
    Serial.print("current_adc = ");
    //delay(2000);
    stop_motor();
}



