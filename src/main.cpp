#include <Arduino.h>
#include <DNSServer.h>
#include <ESPUI.h>
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <SoftwareSerial.h>

#include "MedianFilterLib.h"


#define SENSOR_BAUD_RATE 9600
#define SENSOR_RX_PIN 13
#define PUMP_PIN 4

#define FILTER_SIZE 50

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;

SoftwareSerial swSer;
char SensorString[10];
MedianFilter<int> medianFilter(FILTER_SIZE);

enum {
  eSTATE_AUTOMATIC,
  eSTATE_MANUAL
};

int running = eSTATE_AUTOMATIC;
int status1,status2,lowerID,upperID,distID,upSPID,lowSPID;

bool pumping = false;

int startPumpingSP=100;
int stopPumpingSP=30;
int currentDist=50;
int currentReading=0;
int readValid=0;

void sensorRead()
{
	static int state = 0;
	static int i;
	int r;

	 while (swSer.available() > 0) {
		 r = swSer.read();
		 switch (state)
		 {
		 case 0:
			 if (r==0xff)
			 {
				 SensorString[i++]=r;
				 state++;
			 }
			 break;
		 case 1:
			 SensorString[i++]=r;
			 if (i>3)
			 {
				 //end of string calc Csum
				 int c;
				 uint8_t sum=1;
				 //Serial.printf("Calc CSum\n");
				 uint16_t dist;
				 for (c=0;c<3;c++)
				 {
					 sum+=SensorString[c];
				 }
				 //Serial.printf("sum %d %x %x\n",sum,sum & 0xff, SensorString[3]);
				 if ((sum & 0xff) == SensorString[3])
				 {
					 //Serial.printf("Csum pass\n");
					 dist = (SensorString[1]<<8) | SensorString[2];
					 //Serial.printf("Dist %d\n",dist);
           if ((dist > 200) && (dist < 8000))
           {
             currentDist = medianFilter.AddValue(dist/10);
             Serial.printf("currentDist %d\n",currentDist);
             ESPUI.updateControlValue(distID,String(currentDist));
             if (currentReading < FILTER_SIZE)
               currentReading++;
             else
               readValid = true;

             //volatileSettings.distance -= volatileSettings.distance / 10;
             //volatileSettings.distance += dist / 10;
           }
         }
				 state=0;
				 i=0;
			 }
			 break;
		 }
	 }

}

void runPump()
{
  if (running == eSTATE_MANUAL)
  {

  }
  else if (readValid)
  {
      if (pumping)
      {
        if (currentDist < stopPumpingSP)
        {
          ESPUI.updateControlValue(status2, "Off");  
          digitalWrite(PUMP_PIN, 0);
          pumping = false;
        }
        else
          digitalWrite(PUMP_PIN, 1);
       
      }
      else
      {
        if (currentDist > startPumpingSP) {
          ESPUI.updateControlValue(status2, "Pumping");
          digitalWrite(PUMP_PIN, 1);
          pumping = true;
        }
        else
          digitalWrite(PUMP_PIN, 0);
      }
  }
}


void buttonCallback(Control *sender, int type) {
  switch (type) {
  case S_INACTIVE:
    //if (running == eSTATE_AUTOMATIC)
    {
      ESPUI.updateControlValue(status1, "Manual");
      ESPUI.getControl(status1)->color = ControlColor::Carrot;
      ESPUI.updateControl(status1);

      ESPUI.updateControlValue(status2, "Pumping");
      digitalWrite(PUMP_PIN, 1);

      running = eSTATE_MANUAL;
    } 
    break;
  case S_ACTIVE:

      ESPUI.updateControlValue(status1, "Automatic");
      ESPUI.getControl(status1)->color = ControlColor::Peterriver;
      ESPUI.updateControl(status1);      
      ESPUI.updateControlValue(status2, "Off");
      digitalWrite(PUMP_PIN, 0);      
      running = eSTATE_AUTOMATIC;    
    break;
  }
}


void upperslider(Control *sender, int type) {
  stopPumpingSP = sender->value.toInt();
  Serial.printf("Upper setpoint %d\n",stopPumpingSP);
  
  ESPUI.updateControlValue(upSPID,String(stopPumpingSP));
  if (stopPumpingSP>startPumpingSP) {
    startPumpingSP = stopPumpingSP;
    ESPUI.updateControlValue(lowSPID,String(startPumpingSP));
  }  
}

void lowerslider(Control *sender, int type) {
  startPumpingSP = sender->value.toInt();
  Serial.printf("Lower setpoint %d\n",startPumpingSP);
  ESPUI.updateControlValue(lowSPID,String(startPumpingSP));
  if (startPumpingSP<stopPumpingSP) {
    stopPumpingSP = startPumpingSP;
    ESPUI.updateControlValue(upSPID,String(stopPumpingSP));
  }
}

void setupUI()
{
  status1 = ESPUI.label("Status:", ControlColor::Peterriver, "Automatic");
  status2 = ESPUI.label("Pump Status:", ControlColor::Peterriver, "Off");
  distID = ESPUI.label("Level", ControlColor::Peterriver, "0");
  ESPUI.switcher("Operation", &buttonCallback, ControlColor::Emerald, "Automatic / Manual");
  upperID=ESPUI.slider("Stop filling", &upperslider, ControlColor::Turquoise, 30,30,140);
  lowerID=ESPUI.slider("Start filling", &lowerslider, ControlColor::Turquoise, 100,30,140);
  ESPUI.sliderContinuous = true;

  upSPID = ESPUI.label("Upper SP", ControlColor::Peterriver, "30");
  lowSPID = ESPUI.label("Lower SP", ControlColor::Peterriver, "100");
  
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PUMP_PIN,OUTPUT);
  digitalWrite(PUMP_PIN,0);

  swSer.begin(SENSOR_BAUD_RATE,SWSERIAL_8N1,SENSOR_RX_PIN);

  WiFi.hostname("TankControl");
  setupUI();
  //ESPUI.setVerbosity(Verbosity::VerboseJSON);
  ESPUI.begin("Water Tank Controller");

  AsyncWiFiManager wifiManager(ESPUI.server,&dnsServer);
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  //wifiManager.setAPCallback(configModeCallback);
  //wifiManager.resetSettings();
  wifiManager.autoConnect("TankControl");
}

void loop() {
  // put your main code here, to run repeatedly:
  dnsServer.processNextRequest();
  sensorRead();
  runPump();

}