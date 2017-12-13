//GPS on Serial-1

char temp,longi[11],lati[11];

float LATI,LONGI,LATF,LONGF;
unsigned long dist;
double brng;

void GPS_Init()
{
   int i;
   for(i=0;i<10;i++)                  //initializing total array zero string to avoid producing garbage values
    {
        lati[i]='0';
        longi[i]='0';
    }
    lati[10]='\0';
    longi[10]='\0';
    delay(100); 
}

void getGPSData(void)
{   
  //==================== searching for "GG" ===================/        
  int i;
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
       
        for( i=0,temp=Serial1.read();',' !=temp;i++)                //storing all north latitude values
        {
          lati[i]=temp;
          while ( !Serial1.available() );
          temp=Serial1.read();
          
        }
        lati[i]='\0';
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
       
        for( i=0,temp=Serial1.read();',' !=temp;i++)
        {
          longi[i]=temp;
          while ( !Serial1.available() );
          temp=Serial1.read();
          
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
