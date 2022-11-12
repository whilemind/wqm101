/**
 * This code is to communicate with LoRaWAN. 
 * It has been developed under the IoTLab Federation University Australia, Churchill Campus.
 * 
 * Temperature sensor:
 *    1. VCC will connect with 5v
 *    2. data pin will connect with digital pin 5
 * 
 * TDS sensor:
 *    1. VCC will connect with 5v
 *    2. data ping will connect with Analog pin A1
 *    
 * DOWNLINK Command:   
 *    1. To turn the onboard builtin LED on, you need to send the following bytes:
 *      > In base64 format is: AQNn
 *      > In bytes: 01 03 67
 *        01 - downlink payload
 *        03 - builtin led
 *        67 - turn it on.
 *    2. To turn the onboard builtin LED off, you need to send the following bytes:
 *      > In base64 foramt is: AQNo
 *      > In bytes: 01 03 68
 *        01 - downlink payload
 *        03 - builtin led
 *        68 - turn it off.
 *    3. Reset sleep time.    
 *      > In base64 format is: AQQC
 *      > 01 - downlink payload
 *      > 04 - reset sleep time.
 *      > n  - time in minute. It is 2 mins set in the base64 above.
 *      
 * @author: Mohammad Mahabub Alam
 */

#include <MKRWAN.h>
#include <OneWire.h>
#include <ArduinoLowPower.h>
#include "secret.h"

#define PRODUCTION false

#define DS18S20_DIGITAL_PIN 5 // Temperature sensor digital pin 5
#define DEFAULT_LED_PIN 6
#define MSG_SIZE 8 // Maximum will be 11 bytes.

// This is for the TDS.
#define TdsSensorPin A1
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT  30  // sum of sample point


LoRaModem modem;
OneWire dsHandler(DS18S20_DIGITAL_PIN);

// Enter your sensitive data in the secret tab or arduino_secrets.h
String devEui = SECRET_DEV_EUI;
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

int gIsNetworkJoined = 0;
int gSleepTime = 300000; // it is 5 mins in milli second

byte msg[MSG_SIZE];

struct TempData {
  byte lsb;
  byte msb;
};

int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float temperature = 25;
static unsigned long startTimeTik;

void setup() {
  // Initialize serial port at 9600 bauds 
  Serial.begin(115200);
//  while (!Serial);
  
  // Initialize LoRa module with the AS923 region parameters
  if (!modem.begin(AS923)) {
    #if !PRODUCTION    
    Serial.println("Failed to start module");
    #endif    
    while (1) {}
  };

  // Device module version and EUI
  delay(1000);
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  pinMode(TdsSensorPin,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  modem.setADR(false);
  gIsNetworkJoined = join();
  modem.minPollInterval(30);
}

void loop(){
  TempData tdata;
  float tdsData;
    
  if(gIsNetworkJoined > 0) {
    tdata = getTemp();
    tdsData = getTDS();

    // Temperature data
    msg[0] = 0x01; // channel 1
    msg[1] = 0x67; // temperature code by Cayenne LLP.
    msg[2] = tdata.lsb;
    msg[3] = tdata.msb;
    Serial.println("Temperature MSB: " + String(tdata.msb) + " and the LSB: " + String(tdata.lsb));

    // This is the TDS sensor data.
    msg[4] = 0x02; // channel 2
    msg[5] = 0x68; // temperature code by Cayenne LLP.
    
    if(tdsData > -100) {
      int tmp = round(tdsData);
      Serial.println("Tds in int format is " + String(tmp));
      msg[6] = (tmp >> 8) & 0xFF;
      msg[7] = ((tmp << 8) >> 8) & 0xFF;
    } else {
      msg[6] = 0x0;
      msg[7] = 0x0;
    }
    
    Serial.println();
    Serial.print("Sending msg: ");
    for (unsigned int i = 0; i < MSG_SIZE; i++) {
      Serial.print(msg[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
      
    send_data();
    receive_data(10000);  
  } else {
    Serial.println("Not connected with LoRaWAN...");
  }
    
//  LowPower.deepSleep(300000); // unit is in milliseconds
  delay(gSleepTime);
}

int join() {
  int connected = 0;

  Serial.println("Trying to join...");
  
  connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {
    Serial.println("Something went wrong, retrying...");
    delay(5000);
    connected = join();
  } else {
    Serial.println("Joined.");
  }

  return connected;
}

int send_data() {
  int error_flag = 0;
  
  modem.beginPacket();

  for (unsigned int i = 0; i < MSG_SIZE; i++) {
    modem.write(msg[i]);
    Serial.print(msg[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  error_flag = modem.endPacket(true);

  if(error_flag > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message");
  }

  return error_flag;
}

void receive_data(int delayTime) {
  byte rcv[64];
  int i = 0, j;
  startTimeTik = millis();

  Serial.println("In receiving mode:");

  while((millis() - startTimeTik) < delayTime) {
    while (modem.available()) {

      rcv[i++] = modem.read();
      
      if(i >= 64) { // can't access invalid memory
        Serial.println("Can't receive more than 64 bytes of data.");
        break;
      }
    }
    
    if(i > 0) {
      Serial.print("Received: ");
      for (j = 0; j < i; j++) {
        Serial.print(rcv[j], HEX);
        Serial.print(" ");     
      }
      Serial.println();  

      if(i >= 3) {
        if(rcv[0] == 0x01) {
          if(rcv[1] == 0x03 && rcv[2] == 0x67) {
            Serial.println("Turning Buildin LED on");
            digitalWrite(LED_BUILTIN, HIGH);
          } 

          if(rcv[1] == 0x03 && rcv[2] == 0x68) {
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println("Turning Buildin LED off");
          }

          if(rcv[1] == 0x04) {
            Serial.println("Resetting the sleep time to " + String(rcv[2]) + " mins.");
            gSleepTime = (rcv[2] * 60000);
          }
          
        }
      }
      
      break;       
    }
  }

  if(i == 0) {
    Serial.println("Nothing received.");
  }
}

float getTDS() {
  float averageVoltage = 0, tdsValue = -100;
  static unsigned long analogSampleTimepoint, printTimepoint;
  
  analogSampleTimepoint = millis();
  printTimepoint = millis();

  // put your main code here, to run repeatedly:
  while(tdsValue == -100) {
    
    //every 40 milliseconds,read the analog value from the ADC
    if(millis() - analogSampleTimepoint > 40U) {     

     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
    }
       
    if(millis() - printTimepoint > 800U) {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
    
      Serial.print("voltage:");
      Serial.print(averageVoltage,2);
      Serial.print("V   ");
      
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
    }
//    delay(1);
  }

   return tdsValue;
}

int getMedianNum(int bArray[], int iFilterLen) {
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

TempData getTemp(){
  
  //returns the temperature from one DS18S20 in DEG Celsius
  TempData tdata = {0xFF, 0xFF};
  
  byte data[12];
  byte addr[8];

  if ( !dsHandler.search(addr)) {
      //no more sensors on chain, reset search
      dsHandler.reset_search();
      return tdata;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return tdata;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return tdata;
  }

  dsHandler.reset();
  dsHandler.select(addr);
  dsHandler.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = dsHandler.reset();
  dsHandler.select(addr);    
  dsHandler.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = dsHandler.read();
  }
  
  dsHandler.reset_search();
  
  byte LSB = data[0];
  byte MSB = data[1];
 
  
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;  
  Serial.print("Temperature is: ");
  Serial.println(TemperatureSum);

  tdata.lsb = data[0];
  tdata.msb = data[1];
  
  return tdata;
}
