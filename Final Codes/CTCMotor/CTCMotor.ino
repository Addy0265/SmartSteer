#include <math.h>
#include <Wire.h>
#include <avr/io.h>

#define PULSE_COUNT 500
#define LEDPIN 13
#define PULPIN 11
#define DIRPIN 8

const int MAG = 0x1E;
int16_t MgX, MgY, MgZ;
int error;
float heading = 0;
float headingDegrees = 0;
int count = 0;

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

void timInitCTC()
{ 
  pinMode(PULPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
   
  setOCR(3000);     // compare match register 16MHz/256/2Hz
  
  TCCR1A |= (1<<COM1A0);//|(1<<COM1A0);
  TCCR1B |= (1<<WGM12);//|(1<<4);   // CTC mode
  TCCR1B |= (1 << CS11);//|(1<<CS10); // 8 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);   // toggle LED pin
  
  if(count < PULSE_COUNT)
  count++;
  else
  {
    count = 0;
    //setOCR(0); 
    TCCR1A &= ~(1<<COM1A0);
    resetPin(PULPIN);
  }  
}


void setup()
{
  setPin(DIRPIN);
  noInterrupts();           // disable all interrupts  
  timInitCTC();
  interrupts();             // enable all interrupts
  Serial.begin(9600);
  
  Wire.begin();
  Wire.beginTransmission(MAG);
  Wire.write(0x00);        // CONFIG_A Register
  Wire.write(0b01110000);  // 8 samples averaged per cycle, Data Output rate = 15Hz, Normal Measurement Mode No Bias.
  Wire.write(0b00100000);  // CONFIG_B Register 1090 Gain
  Wire.write(0x00);  // MODE Register
  error = Wire.endTransmission(true);
  Serial.println(error);
  
}

void loop()
{
  Wire.beginTransmission(MAG);
  Wire.write(0x03);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MAG,6,true);  // request a total of 14 registers
  MgX=Wire.read()<<8|Wire.read();  // 0x03 (MAG_XOUT_H) & 0x04 (MAG_XOUT_L)     
  MgZ=Wire.read()<<8|Wire.read();  // 0x05 (MAG_ZOUT_H) & 0x06 (MAG_ZOUT_L)
  MgY=Wire.read()<<8|Wire.read();  // 0x07 (MAG_YOUT_H) & 0x08 (MAG_YOUT_L)
  Wire.endTransmission(true);
  
  if(MgY != 0)
  headingDegrees =  atan2(MgY, MgX)*180/M_PI;
  else if(MgY == 0 && MgX < 0)
  headingDegrees = 180;
  else
  headingDegrees = 0;
  
  
  //  // Correct for when signs are reversed.
  // if(heading < 0)
  //   heading += 2*PI;
    
  // // Check for wrap due to addition of declination.
  // if(heading > 2*PI)
  //   heading -= 2*PI;
   
  // Convert radians to degrees for readability.
 // headingDegrees = heading * 180/M_PI; 
  // if (headingDegrees>180) 
  // {
  //   headingDegrees=360-headingDegrees;
  //   //front_flag = LOW;   
  // }
  // else 
  // {
  //   //front_flag=HIGH;
  // }
  
  Serial.print(" Mag_X = "); Serial.print(MgX);
  Serial.print("\tMag_Y = "); Serial.print(MgY);
    Serial.print("\tMag_Z = "); Serial.print(MgZ);
  Serial.print("\tHeading Degrees: "); Serial.println(headingDegrees);
  //delay(50);
}

