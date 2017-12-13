#include <math.h>
#include <Wire.h>
#include <avr/io.h>

#define ROT_ANGLE 180
#define PULSE_COUNT 5*ROT_ANGLE
#define PERIOD 65535
#define OPT 6

#define LEDPIN 13
#define PULPIN 11
#define DIRPIN 8
#define FEEDBACK_PIN 7

volatile int currentAngle = 0;
volatile int count = 0;
volatile int flag = 0;
volatile int Pulse_Count = 0; 

int j=0;
float DISTX,DISTY;       //DISTX and DISTY are integer distances in X direction and Y direction respectivey
char msg[75];             
char distx[4],disty[4],directx,directy;   //chalking of need values but in characters
int maxi,m,n,multi;
int front_pulse_pin , rear_pulse_pin=11 , front_direction_pin ,rear_direction_pin=9;


char temp,longi[11],lati[11];
int k;
float LATI,LONGI;
double d;

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
  //TIMSK1 |= (1 << OCIE1A);          // enable timer compare interrupt
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
     setOCR(PERIOD/100);
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
     setOCR(PERIOD/100);
     while(flag);
     //delay(1);  
}


void clear_rx_buffer(void)                                                  
{ 
  //function to clear recieving buffer 
    while (Serial2.available()>0)     // this function is used to clear rx_buffer
    {Serial2.read();}   
}

void modem_initialization(void)
{
  Serial2.begin(38400);
  delay(100);
  clear_rx_buffer();                                
  Serial2.println("AT\r");// Attention command to wake up GSM modem
  delay(200);
  clear_rx_buffer();
  Serial2.println("ATE0\r");
  clear_rx_buffer();
    Serial2.println("AT+CMGF=1\r");      // setting up GSM in text mode
  delay(500);
  clear_rx_buffer();
      Serial2.println("AT+CNMI=1,2,0,0,0\r");      // setting up GSM in text mode
  delay(500);
  clear_rx_buffer();
}

/*******************************************************************************************************************************/


void getdistance_bearing(float LATI, float LATF, float LONGI, float LONGF)
{
    unsigned long R = 6371000; // metre
    double p1 = LATI*(PI/180);
    double p2 = LATF*(PI/180);
    double delp = (LATF-LATI)*(PI/180);
    double dell = (LONGF-LONGI)*(PI/180);
    double a = sin(delp/2) * sin(delp/2) + cos(p1) * cos(p2) * sin(dell/2) * sin(dell/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
     d = R * c/2;
}    

 
void getlatilongi(void)
{
  
                                                                //==================== searching for "GG" ===================//    
            while ( !Serial1.available() );    
        while ( 'G' != Serial1.read() )
        {
           while(!Serial1.available());
        }        
      
        while(!Serial1.available());
     while ( 'G' != Serial1.read() )
     { 
       while ( !Serial1.available() );    
        while ( 'G' != Serial1.read() )
        {
           while(!Serial1.available());
        }        
      
        while(!Serial1.available());
     }
    

 /***************************************************************************************************************************/
                                                                          //============== seeking for north cordi
    
    
        while ( !Serial1.available() );                           
     while ( ',' != Serial1.read() )                        //this flushes all values between two commas
     {
         while ( !Serial1.available() );
     }
 
    while ( !Serial1.available() );                           
     while ( ',' != Serial1.read() )
     {
         while ( !Serial1.available() );
     }
 
/****************************************************************************************************************************/
    Serial.print(" N: ");
    
   
        while ( !Serial1.available() ); 
       
        for( k=0,temp=Serial1.read();',' !=temp;k++)                //storing all north latitude values
        {
          lati[k]=temp;
          while ( !Serial1.available() );
          temp=Serial1.read();
          
        }
        lati[k]='\0';
        float m=atof(lati);                                        //conversion of array to floating value that is RAW data of GPS and then converting it in degrees
        Serial.print(m,8);                          
        Serial.print("   ");
      LATI=(int)(m/100)+(m-((int)(m/100))*100)/60.0;
      Serial.print(LATI,8);
/********************************************************************************************************************************/
    
    
        while ( !Serial1.available() );                       // reading a character from the GPS
     while ( ',' != Serial1.read() )
     {
    while ( !Serial1.available() );   
     }

/***************************************************************************************************/

     Serial.print(" E: ");
    while ( !Serial1.available() ); 
       
        for( k=0,temp=Serial1.read();',' !=temp;k++)
        {
          longi[k]=temp;
          while ( !Serial1.available() );
          temp=Serial1.read();
          
        }
        longi[k]='\0';
        float n=atof(longi);                                              //conversion of array to floating value that is RAW data of GPS and then converting it in degrees
         Serial.print(m,8);
        Serial.print("   ");
      LONGI=(int)n/100+(n-((int)n/100)*100)/60.0;
      Serial.print(LONGI,8);
        
    Serial.println();
    
 
  
  
  
  
}
 


/***************************************************************************************************************************************/

void new_rec_sms(void)
{
while(1){
  while(Serial2.available()>0)
  {
 msg[j]=Serial2.read();
 //Serial.print(msg[i]);
 j=j+1;
 while(Serial2.available()==0);
 //delayMicroseconds(500);
 }
  maxi=j;
 /****************************************************************************************/ 
  if(maxi>0){   
  for(j=51,m=0;msg[j]!=' ';j++,m++)   //m also gives me the number of digits
 {
   distx[m]=msg[j];
   
 } 

 directx=msg[j+1];
  for(j=j+3,n=0;msg[j]!=' ';j++,n++)   //n also gives me the number of digits
 {
   disty[n]=msg[j];
   
 } 
 directy=msg[j+1];
 //code for converting necessary distance value
 /******************************************************************************************/
 DISTX=atof(distx);
 DISTY=atof(disty);
 
 /***********************************************************/
 
  Serial.println(DISTX);
  Serial.println(directx);
  Serial.println(DISTY);
  Serial.println(directy);
  /***************************************************************/
  // Serial.print(msg[i]);
    
  
break;  
  }
  
  maxi=0;j=0;
  
}
}
 
 
/*************************************************************************************************************************************************************************/


void setup()
{
  modem_initialization();
  new_rec_sms();
  
  setPin(DIRPIN);
  noInterrupts();           // disable all interrupts  
  timInitPWM();
  interrupts();             // enable all interrupts
  Serial.begin(9600); 
  Serial1.begin(9600);
  //Serial2.begin(38400);
    
 for( k=0;k<10;k++)                  //initializing total array zero string to avoid producing garbage values
{
  lati[k]='0';
  longi[k]='0';
  
}
 lati[10]='\0';
 longi[10]='\0';
  delay(100);
  
  while(1)
{
    if(LATI>0&&LONGI>0)
    {   
      break;
    }

}
 for(int h=0;h<5;h++)
  {
    getlatilongi(); 
  }

   new_rec_sms();
//getdistance_bearing(LATI,22.321384,LONGI,87.299889);
getdistance_bearing(LATI,DISTX,LONGI,DISTY);
}


void loop()
{
  getlatilongi();
  getdistance_bearing(LATI, DISTX, LONGI, DISTY);
  if(digitalRead(FEEDBACK_PIN) == LOW)
  {
    
    if(abs(d) > 5)
    {
      changeAngle(-1); //Main Code - Run Motor
    }
    else
    {
      setOCR(0);
    } 
  }
  else
  {
     // Obstacle Detected by both sonars - Stop Motor
     setOCR(0); 
  } 
}
