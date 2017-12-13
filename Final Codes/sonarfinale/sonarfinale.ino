/*
    Sonar Placement
    Sonar 1 : Facing towards the left of the bicycle
    Sonar 0 : Facing towards the center of the bicycle
    Sonar 2 : Facing towrds the right of the bicycle
    
    Turning algorithm :
    Detection by Sonar 0 ONLY : turn right by 28 degrees to get the obstacle out of range of vision
    Detection by Sonar 1 ONLY : don't do anything
    Detection by Sonar 2 ONLY : don't do anything
    Detection by Sonar 0 and Sonar 1 (object in front and left) : turn right by 35-37 degrees to get obstacle out of range of vision
    Detection by Sonar 0 and Sonar 2 (object in front and right) : turn left by 35-37 degrees to get obstacle out of range of vision
    Detection by Sonar 1 and Sonar 2 (object in right and left) : don't do anything
    Detection by all three Sonars : Abrupt turn of about 50 - 60 degrees on either side
*/

#define TICKS_PER_DEGREE (1800/360)
#define D_THRESH 220
#define D_MAX 800
#define ANGLE_LR 20
#define ANGLE_CLCR 40
#define ANGLE_C_CLR 0
 
#define L_SONAR 0
#define C_SONAR 1
#define R_SONAR 2

#define SA 30

int dist[3][SA] = {{0}, {0}, {0}};

int filter(int sample[], int siz)
{  int i;
   int error_range = 100;
   int flag = 0; 
   int prv = sample[0];
   int avg;
   if(sample[0] > 20)
   avg = sample[0];
   else
   {
     flag = 1;
     avg = 0; 
   }
   
   for(i=1; i<siz; i++)
   {
       if(abs(sample[i]-prv) < error_range)
       {
          avg += sample[i];
       }
       else
       {
          flag = 1;          
       }  
       prv = sample[i];
   }    
       if(flag == 0)
       return avg/siz;
       else 
       return avg/(siz-1);  
}
 
void setup() {
  pinMode(3, OUTPUT); //trig of sonar 0
  pinMode(5, OUTPUT); //trig of sonar 1
  pinMode(7, OUTPUT); //trig of sonar 2
  pinMode (2, INPUT); //echo of sonar 0
  pinMode (4, INPUT); //echo of sonar 1
  pinMode (6, INPUT); //echo of sonar 2
  pinMode(8,OUTPUT);  // DIR Pin
  pinMode(11,OUTPUT); // PULSE Pin
  Serial.begin(38400);
  void SonarSend();
  void gotoangle(int);
  void turn();
  long microsecondsToCentimeters(long);
}
long duration, distCM[3] = {D_MAX, D_MAX, D_MAX};
int currentAngle = 0;
int deltaAngle = 0;

void printSonarData()
{ 
   int j; 
   for(j=0; j<3; j++)
   {
       Serial.print("SONAR-");
       Serial.print(j+1);
       Serial.print(" :");
       Serial.print(distCM[j]); 
       Serial.print("\t");
   }
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  if (microseconds > 29000*0.8)
  {
      microseconds = 29000*0.8;
  }  
  return microseconds/58;
}

void SonarSend()
{
  int i=1;
  int j = 0;
  for(j=0; j<SA; j++)
  {
      for (i=1; i<4; i++)
      {
          digitalWrite(2*i+1, LOW);
          delayMicroseconds(1);
          digitalWrite(2*i+1, HIGH);
          delayMicroseconds(10);
          digitalWrite(2*i+1, LOW);
          delayMicroseconds(10);
          duration = pulseIn(2*i, HIGH);
          
          // convert the time into a distance in CM
          dist[i-1][j] = microsecondsToCentimeters(duration);
      }  
  }
  for(i=0; i<3; i++)
  {
     distCM[i] = filter(dist[i], SA); 
  }
  turn();
}

void gotoangle(int givenAngle)
{
  int j = 0;
  
  deltaAngle = givenAngle - currentAngle;
  
  digitalWrite(8,LOW);
  if(deltaAngle !=0)
  {
    if(deltaAngle < 0)
    {
          digitalWrite(8,HIGH);
          deltaAngle = -deltaAngle;
    }
    
    for(j=0; j<deltaAngle*TICKS_PER_DEGREE; j++)
    {
        digitalWrite(11,HIGH);
        delayMicroseconds(1500);
        digitalWrite(11,LOW);
        delayMicroseconds(1500);
    }
  }
  
  currentAngle = givenAngle;
  
  printSonarData();
  Serial.print("\tDELTA ANGLE:");
  Serial.print(deltaAngle);
  Serial.print("\tCURRENT ANGLE : ");
  Serial.println(currentAngle);
  Serial.println();
  //delay(20);
}

void turn()
{
     int angle;
     
    if(distCM[C_SONAR]<=D_THRESH && distCM[L_SONAR]<=D_THRESH && distCM[R_SONAR]>D_THRESH)  //left and centre object
    {
        angle = ANGLE_CLCR;
    }
    else if(distCM[C_SONAR]<=D_THRESH && distCM[R_SONAR]<=D_THRESH && distCM[L_SONAR]>D_THRESH)  //right and centre object
    {
        angle = -ANGLE_CLCR;
    }
    else if(distCM[R_SONAR]<=D_THRESH && distCM[L_SONAR]<=D_THRESH && distCM[0]>D_THRESH)     // left and right object
    {
        angle = 0;
    }
    else if(distCM[C_SONAR]<=D_THRESH && distCM[L_SONAR]>D_THRESH && distCM[R_SONAR]>D_THRESH)    //centre object
    {
        angle = ANGLE_C_CLR;
     }
    else if(distCM[L_SONAR]<=D_THRESH && distCM[C_SONAR]>D_THRESH && distCM[R_SONAR]>D_THRESH)     //left object
    {
        angle = ANGLE_LR;
    }
    else if(distCM[R_SONAR]<=D_THRESH && distCM[L_SONAR]>D_THRESH && distCM[C_SONAR]>D_THRESH)     //right object
    {
        angle = -ANGLE_LR;
    }
    else if(distCM[R_SONAR]<=D_THRESH && distCM[L_SONAR]<=D_THRESH && distCM[C_SONAR]<=D_THRESH)    //all direction object
    {
        angle = -ANGLE_C_CLR; 
    }
    else if(distCM[R_SONAR]>D_THRESH && distCM[L_SONAR]>D_THRESH && distCM[C_SONAR]>D_THRESH)         //no obstacle found
    {
         angle = 0;  // NO OBSTACLE DETECTED - RUN OTHER PERIPHERALS
    }
    else
    {
        angle = 0;
    }  
    
    //Serial.print("HERE\t");
    gotoangle(angle); 
}

void loop()
{
  SonarSend();
}


