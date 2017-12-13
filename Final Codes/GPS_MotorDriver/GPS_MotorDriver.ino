
char temp,longi[11],lati[11];

float LATI,LONGI,LATF,LONGF;
unsigned long dist;
double brng;
void getlatilongi(void)
{   
  //==================== searching for "GG" ===================/        
  int i;
            while ( !Serial.available() );    
        while ( 'G' != Serial.read() )
        {
           while(!Serial.available());
        }        
      
        while(!Serial.available());
     while ( 'G' != Serial.read() )
     { 
       while ( !Serial.available() );    
        while ( 'G' != Serial.read() )
        {
           while(!Serial.available());
        }        
      
        while(!Serial.available());
     }
    

 /***************************************************************************************************************************/
                                                                          //============== seeking for north cordi
    
    
        while ( !Serial.available() );                           
     while ( ',' != Serial.read() )                        //this flushes all values between two commas
     {
         while ( !Serial.available() );
     }
 
    while ( !Serial.available() );                           
     while ( ',' != Serial.read() )
     {
         while ( !Serial.available() );
     }
 
/****************************************************************************************************************************/
    Serial.print(" N: ");
    
        while ( !Serial.available() ); 
       
        for( i=0,temp=Serial.read();',' !=temp;i++)                //storing all north latitude values
        {
          lati[i]=temp;
          while ( !Serial.available() );
          temp=Serial.read();
          
        }
        lati[i]='\0';
        float m=atof(lati);                                        //conversion of array to floating value that is RAW data of GPS and then converting it in degrees
        Serial.print(m,8);                          
        Serial.print("   ");
      LATI=(int)(m/100)+(m-((int)(m/100))*100)/60.0;
      Serial.print(LATI,8);
/********************************************************************************************************************************/
    
    
        while ( !Serial.available() );                       // reading a character from the GPS
     while ( ',' != Serial.read() )
     {
    while ( !Serial.available() );   
     }

/***************************************************************************************************/

     Serial.print(" E: ");
    while ( !Serial.available() ); 
       
        for( i=0,temp=Serial.read();',' !=temp;i++)
        {
          longi[i]=temp;
          while ( !Serial.available() );
          temp=Serial.read();
          
        }
        longi[i]='\0';
        float n=atof(longi);                                              //conversion of array to floating value that is RAW data of GPS and then converting it in degrees
         Serial.print(m,8);
        Serial.print("   ");
      LONGI=(int)n/100+(n-((int)n/100)*100)/60.0;
      Serial.print(LONGI,8);
        
    Serial.println();  
}


void getdistance_bearing(float LATI, float LATF, float LONGI, float LONGF)
{
    unsigned long R = 6371000; // metre
    double p1 = LATI*(PI/180);
    double p2 = LATF*(PI/180);
    double delp = (LATF-LATI)*(PI/180);
    double dell = (LONGF-LONGI)*(PI/180);
    double a = sin(delp/2) * sin(delp/2) + cos(p1) * cos(p2) * sin(dell/2) * sin(dell/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;
    dist = ((int)d)*100;
    double y = sin(dell) * cos(p2);
    double x = cos(p1)*sin(p2) - sin(p1)*cos(p2)*cos(dell);
    brng = atan2(y, x)*(180/PI);
}

void getbearing()
{
    //get initial heading from HMC
    float headi; //feed data from HMC
}    

void setup()
{
     pinMode(8,OUTPUT);  //pin 8 for direction pin
    pinMode(11,OUTPUT); //pin 11 for pulse pin
    digitalWrite(8,LOW); 
    Serial.begin(9600);
    for(int i=0;i<10;i++)                  //initializing total array zero string to avoid producing garbage values
    {
        lati[i]='0';
        longi[i]='0';
    }
    lati[10]='\0';
    longi[10]='\0';
    delay(100);      
}
 
void loop ()
{
   getlatilongi();  
}
