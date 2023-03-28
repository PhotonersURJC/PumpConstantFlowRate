/*---------------------------------------------------------------------------*\

The following code is used to work at constant flow rate in a photoreactor
where the pump is regulated through a variable-frequency drive (VFD). 

The sensors were directly connected to an Arduino Mega 2560 board integrated with a keypad, 
an LCD monitor, and an SD memory card to read and save the data instantaneously. Finally, 
a PID controller was implemented in the code to work at the desired constant flow rate 
during recirculation or in the presence of external variation.  

/*---------------------------------------------------------------------------*\
 * 
Code corresponding to the article entitled:
"Removal of diclofenac by UV-B and UV-C light-emitting diodes (LEDs) driven advanced 
oxidation processes (AOPs): wavelength dependence, kinetic modelling and energy consumption"
by:
A. Raffaella P. Pizzichetti, Cristina Pablos, Cintia Casado and Javier Marugán
from
    Department of Chemical and Environmental Technology, ESCET, Universidad Rey Juan Carlos, 
    C/Tulipán s/n, 28933 Móstoles, Madrid, Spain 
and 
A. Raffaella P. Pizzichetti, Ken Reynolds and Simon Stanley
from 
  ProPhotonix IRL LTD, 3020 Euro Business Park, Little Island, Cork, T45 X211, Ireland 
and 
A. Raffaella P. Pizzichetti, Eric Moore
from 
  Environmental Research Institute, University College Cork, Lee Road, Cork, Ireland T23 XE10

  
Tel. +34 91 488 46 19; E-mail: cristina.pablos@urjc.es

    
License
  This file is is free software: you can redistribute it and/or modify it under the terms 
  of the GNU General Public License as published by the Free Software Foundation, either 
  version 3 of the License, or (at your option) any later version. You should have received 
  a copy of the GNU General Public License along with the code. 
  If not, see <http://www.gnu.org/licenses/>.
    
\*---------------------------------------------------------------------------*/
 //include libraries
#include <Keypad.h>
#include <SoftwareSerial.h> 
#include <Wire.h> 
#include <SPI.h>
#include <SD.h>
#include "LiquidCrystal.h"
#include <DS3231.h>
#include "PulseSensor.h"

DS3231  rtc(SDA, SCL);

const byte rowsCount = 4;
const byte columsCount = 4;
 
char keys[rowsCount][columsCount] = {
   { '1','2','3', 'A' },
   { '4','5','6', 'B' },
   { '7','8','9', 'C' },
   { '0','.','E', 'D' }
};
 
const byte rowPins[rowsCount] = { 37, 39, 41, 43 };
const byte columnPins[columsCount] = { 36, 38, 40, 42};
 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, columnPins, rowsCount, columsCount);

// initialize the library by providing the nuber of pins to it
LiquidCrystal lcd(8,9,4,5,6,7);

#define REFRESH_INTERVAL  1000          // refresh time, 1 second
#define WRITE_INTERVAL 1000             // values send to serial port, 5 seconds (5 * 1000)
#define PULSE_PIN 2                    // see external interrupt pins available on your Arduino.
#define PULSES_2_LITERS 450             // coefficient conversion from pulses to liters
#define PULSES_SEC_2_LITERS_MINUTE 7.5  // (450/60) coefficient conversion pulses/second from to liters/minute
PulseSensor flowmeter;                  // instance to collect data
//variables to process and send values
float flowRate;
float acumFlow;
float lastAcumFlow;
boolean firstData;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

int counter = -1;  //variable to count keypresses
String d;
String t;
char filename[12];
char FileName [12];
int str_len;
char pad;
char padText[5];
char padPosition;
File myFile;
int line=1;
float SensorP1 = 0;
//float SensorF1 = 0;
int inputPinP1 = A0;
float Voltage = 0;
const float  OffSet = 0.1;
float Pressure = 0;
float Pressuretot = 0;
float FlowrateTarget;
float Pressure_final=0;
int k = 0;

const int PinVFD = 12; // Analog output pin
byte outputVFD = 0;    // value of PWM

//flow meter average
float flow_average = 0;
int index_c = 0;
float data[3] = {-1,-1,-1};
float newData=0;

double elapsedTime=0;
double error=0;
double lastError=0;
double cumError=0;
double rateError=0;

//PID constants
double kp = 10;
double ki = 0.15;
double kd = 2;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  rtc.begin();
  Serial.println("Sensors measurements:");
  lcd.begin(16,4);
  // set cursor position to start of first line on the LCD
  lcd.setCursor(0,0);
  lcd.print("Initialization");
    while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(53)) {
      Serial.println("initialization failed!");
      lcd.clear();
      lcd.setCursor(0,0);
      //text to print
      lcd.print("Card failed!");
      lcd.clear();
      return;
  }
  Serial.println("initialization done.");
  lcd.clear();
  d=rtc.getDateStr();
  d.replace(".", "");
  String a = d.substring(0, 4);
  String b = d.substring(6, 8);
  a.concat(b);
  a.concat("0.csv");
  Serial.print(a);
  int str_len = a.length() + 1; 
  a.toCharArray(filename, str_len);
  Serial.print("\n");
  Serial.print("filename: ");
  Serial.print(filename);
  Serial.print("\n");
      
      for (int i = 0; i < 10; i++) 
        {
          filename[6] = i + '0';
          Serial.print("\n");
          Serial.print(i);
          Serial.print("\n");
          Serial.print(filename);
          if (! SD.exists(filename)) 
          {
            break;
          } 
        }

  flowmeter.begin(PULSE_PIN, REFRESH_INTERVAL, PULSES_SEC_2_LITERS_MINUTE, PULSES_2_LITERS); //flow-meter
  Serial.println("time(s), average flowrate(l/min), volume(liters)");
  
  firstData = false;
  flowRate = 0;
  acumFlow = 0;
      
  myFile = SD.open(filename, FILE_WRITE);
    
  if (myFile) {
      Serial.print("\nWriting to document.csv...\n");
      Serial.print(rtc.getDOWStr());
      Serial.print(" ");
      Serial.print(rtc.getDateStr());
      Serial.print(" -- ");
      Serial.println(rtc.getTimeStr());
      
      myFile.print("Date;");   
      myFile.print("Time;");
      myFile.print("Time(s);");
      myFile.print("k;");
      myFile.print("VoltagePT(V);");  
      myFile.print("Pressure(bar);");   
      myFile.print("Pressurefinal(bar);");   
      myFile.print("FlowrateTarget(bar);");   
      myFile.print("Flowrate(L/min);");
      myFile.print("Flowrate_averaged(L/min);");
      myFile.print("Volume(liters);");
      myFile.print("OutputVFD;");
       
      myFile.close(); // close the file
  }
  else 
  {
      Serial.println("error opening filename.csv");
  }
  
 padPosition = -1;
 outputVFD = 0; 
  
}

void loop()
  {  
    if (!SD.begin(53)) 
    {
    Serial.println("initialization failed!");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Card failed!");
    return;
    }
  
  char current_key = keypad.getKey();  //get keypad state

  if (current_key) 
  {
    pad= current_key;
    padPosition=padPosition+1;
  }
  
  if (pad=='D')
  {
    if (padPosition>-1)
      {
        FlowrateTarget = (float)strtod(padText,NULL);
        if(FlowrateTarget > 7.5) //To have a max
        FlowrateTarget=7.5;
        for(int i = 0; i < 5; i++)
        padText[i]=0;
        padPosition=-1;
        lcd.clear();
      }
  }
  
  else if(pad=='B')
  {
      for(int i = 0; i < 5; i++)
      padText[i]=0;
      padPosition=-1;
      lcd.clear();
  }
  
  else if(padPosition < 4)
  {
  padText[padPosition]=pad; //to keep writing
  }

  //reading the pulses flow meter
  
  sei();  //Enables interrupts
  currentMillis = millis();
  
  //reading the pressure transmitter
  
  SensorP1 = analogRead(inputPinP1);
  SensorP1 = (SensorP1/204.6); //from Arduino readings to voltage
  SensorP1= (SensorP1-(SensorP1*0.00826));//Arduino voltage if coming from the 9 V power supply
 
   if (SensorP1<0.956) 
    {
    Pressure=0;
    }
   else 
    {
    Pressure = 2.6151*SensorP1 - 2.5; //transforming the current signal into bar
    Pressuretot = Pressuretot + Pressure; //to create an average value
    }
    
  
     if (currentMillis - previousMillis >= WRITE_INTERVAL) 
        
    {
        if (flowmeter.available()) 
        {
        newData = flowmeter.read();
        flowRate=flowmeter.read(); 

        
        if(data[0] == -1 && data[1] == -1 && data[2]== -1)
        {
          data[index_c] = newData;
          flow_average = newData;
          index_c = index_c + 1;
        }
        else if(data[0] != -1 && data[1] == -1 && data[2]== -1)
        {
          data[index_c] = newData;
          flow_average = ((data[0] + data[1])/2);
          index_c = index_c + 1 ;
          
        }
        else if (data[0] != -1 && data[1] != -1 && data[2]== -1)
        {
          data[index_c] = newData;
          flow_average = (data[0] + data[1] + data[2])/3;
        }
        else 
        {
          for (int z = 0; z < 2; z++)
          {
          data[z] = data[z + 1];
          }
          data[index_c] = newData;
          flow_average = 0;
          flow_average = (data[0] + data[1] + data[2])/3;
       
        }

        acumFlow = flowmeter.readAcum(); //volume
        }

        Pressuretot=Pressuretot/(k+1);
        Pressure_final=Pressuretot;
        Pressuretot=0;

        elapsedTime=(currentMillis-previousMillis)/1000;
           if (FlowrateTarget==0)
        {
          outputVFD=0;
          error=0;
          lastError=0; 
          cumError=0;
          rateError=0;
        }
        else {
        error=FlowrateTarget-flow_average;
        cumError += error*elapsedTime;
        rateError = (error-lastError)/elapsedTime;
        
        outputVFD = outputVFD + kp*error + ki*cumError + kd*rateError;
        lastError=error;
        }
        
        previousMillis = currentMillis; 
        
        firstData = true;

        Serial.print("\n");
        Serial.print(currentMillis / 1000);
        Serial.print(",");
        Serial.print(flow_average);
        Serial.print(",");
        Serial.println(acumFlow);
        Serial.print(rtc.getDOWStr());
        Serial.print(" ");
        // Send date
        Serial.print(rtc.getDateStr());
        Serial.print(" -- ");
        // Send time
        Serial.println(rtc.getTimeStr());
        Serial.print(Pressure_final);
        Serial.print(",");
        Serial.print(Pressure);
        Serial.print(",");

    analogWrite(PinVFD, outputVFD);
       
       Serial.print("OutputVFD:    ");
       Serial.print(outputVFD);
       Serial.print("\n");
       Serial.print("error:    ");
       Serial.print(error);
       Serial.print("\n Last error:");
       Serial.print(lastError);
       Serial.print("\n");
       Serial.print("elapsedTime:    ");
       Serial.print(elapsedTime);
        Serial.print("\n");
       Serial.print("cumError:    ");
       Serial.print(cumError);
       Serial.print("\n rateError:");
       Serial.print(rateError);

       Serial.print("\n \n");

       myFile = SD.open(filename, FILE_WRITE); 
       if (myFile) 
       {  
         myFile.print("\n");
         myFile.print(rtc.getDateStr());   
         myFile.print(";");
         myFile.print (rtc.getTimeStr());
         myFile.print(";");  
         myFile.print(currentMillis / 1000);
         myFile.print(";");
         myFile.print(k);
         myFile.print(";");
         myFile.print(SensorP1);   
         myFile.print(";");
         myFile.print(Pressure);   
         myFile.print(";");
         myFile.print(Pressure_final);   
         myFile.print(";");
         myFile.print(FlowrateTarget);   
         myFile.print(";"); 
         myFile.print(flowRate);
         myFile.print(";");
         myFile.print(flow_average);
         myFile.print(";");
         myFile.print(acumFlow);
         myFile.print(";");
         myFile.print(outputVFD);   
         myFile.print(";");
         myFile.close();  
       }
      
        lcd.setCursor(0,0);
        lcd.print("P (bar):");
        lcd.setCursor(10,0);
        lcd.print(Pressure_final);
        lcd.setCursor(0,1);
        lcd.print("F (L/min)");
        lcd.setCursor(10,1);
        lcd.print(flow_average);
        lcd.setCursor(0,2);
        lcd.print("F_t(L/min)");
        lcd.setCursor(11,2);
        lcd.print(FlowrateTarget);
        lcd.setCursor(0,3);
        lcd.print(padText); //display key
        k=-1;
    
  }

 k=k+1;

}
