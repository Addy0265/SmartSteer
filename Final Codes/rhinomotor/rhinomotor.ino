
void setup()
{
  pinMode(8,OUTPUT);  //pin 8 for direction pin
  pinMode(9,OUTPUT);
  pinMode(11,OUTPUT); //pin 11 for pulse pin
  unsigned long anglespeed(unsigned long ,int );
  void stopmotor();
  void godistance(unsigned long,unsigned long);
  Serial.begin(9600);
}
void anglespeed(unsigned long noofrev, int rpm) //function to cover specified number of revolutions at specified rpm, both in int. rpm above 10 would not be accepted as max rpm is 10
{
  if(rpm>10)
  {
  rpm=10;
  } //to avoid any chance of error
  for(unsigned long i=0;i<(noofrev*1800);i++) 
  {
    digitalWrite(11,HIGH);
    delay(33-3*rpm); // 0.2 degrees per increment,  3.33 seconds delay for 10rpm
    digitalWrite(11,LOW);
    delayMicroseconds(5);
  }
}
void stalltest() //enclose in while(1) to stall
{
  digitalWrite(11,LOW);
  delayMicroseconds(5); //Don't use this function to stop bike. It puts unnecessary load on motor and can lead to breakage of shaft coupler
  digitalWrite(8,LOW);
  delayMicroseconds(5);
}
void stopmotor() //to really stop motor, enclose this function in while(1) 
{
    digitalWrite(11,LOW);
}
void startbike()
{
float x=0;
for(int i=0;i<1800;i++) //increases speed of bike gradually to 10rpm over 1 revolution from 1 rpm
{
  digitalWrite(11,HIGH);
  delay(33-x*0.0167); 
  digitalWrite(11,LOW);
  delayMicroseconds(5);
  x++;
}
}

void slowbike() //supply initial delay as argument
{
float x=0;
for(int i=0;i<1800;i++) //decreases speed of bike gradually from 10rpm to 1 rpm
{
  digitalWrite(11,HIGH);
  delay(3+x*0.0167);
  digitalWrite(11,LOW);
  delayMicroseconds(5);
  x++;
}
}
void godistance(int a, unsigned long distance) //function to make the motor move a distance, specified in centimeters forward or backward. Enter 1 as argument for backward and vice versa
{
  if(a==0)
  {
  for(double i=0;i<(distance*5);i++) //9.55 is the ratio between 1800 and 188.5 which is the value of 2pi*r(30cm)
  {
    digitalWrite(8,HIGH);
    digitalWrite(11,HIGH);
    delayMicroseconds(400); 
    digitalWrite(11,LOW);
    delayMicroseconds(5);
  }
  }
  
  else if(a==1)
  {
    for(double i=0;i<(distance*5);i++)
  {    
    digitalWrite(8,LOW);
    digitalWrite(11,HIGH);
    delayMicroseconds(400); 
    digitalWrite(11,LOW);
    delayMicroseconds(5);
 }
 }
}
 
  void loop()
{
/*  digitalWrite(11,HIGH);
  for(int i=0;i<9000;i++) //increases speed of bike gradually to 10rpm over 1 revolution from 1 rpm
{
  digitalWrite(9,HIGH);
  delay(3); 
  digitalWrite(9,LOW);
  delayMicroseconds(5);

Serial.print("def");
*/
/*
digitalWrite(11,HIGH);
  for(int m=0;m<5;m++)
  {
  digitalWrite(9,HIGH);
  delayMicroseconds(333);
  digitalWrite(9,LOW);
  delayMicroseconds(5);
  }
digitalWrite(11,HIGH);
  for(int k=10;k>0;k--)
{
  for(int m=0;m<10;m++)
  {
  digitalWrite(9,HIGH);
  delayMicroseconds(400*k);
  digitalWrite(9,LOW);
  delayMicroseconds(5);
  }
}
delay(1000);
  godistance(0,500);
  delay(3000);
  digitalWrite(11,HIGH);
for(int k=1;k<11;k++)
{
  for(int m=0;m<10;m++)
  {
  digitalWrite(9,HIGH);
  delayMicroseconds(400*k);
  digitalWrite(9,LOW);
  delayMicroseconds(5);
  }
}
*/

godistance(1, 20);
delay(1000);
godistance(0, 20);

while(1)
{
  stopmotor();
}

  //while(1)
 /* {
    for(int i=0;i<150;i++)
  {    
    digitalWrite(11,LOW);
    digitalWrite(9,HIGH);
    delay(3);
    digitalWrite(9,LOW);
    delayMicroseconds(5);
 }
 }*/
  }




