/* LimiTTer sketch for the Arduino UNO/Pro-Mini.
   It scans the Freestyle Libre Sensor every 5 minutes
   and sends the data to the xDrip Android app. You can
   see the data in the serial monitor of Arduino IDE, too.
   If you want another scan interval, simply change the
   SLEEP_TIME value.

   This sketch is based on a sample sketch for the BM019 module
   from Solutions Cubed.

   Wiring for UNO / Pro-Mini:

   Arduino          BM019           BLE-HM11
   IRQ: Pin 9       DIN: pin 2
   SS: pin 10       SS: pin 3
   MOSI: pin 11     MOSI: pin 5
   MISO: pin 12     MISO: pin4
   SCK: pin 13      SCK: pin 6
   I/O: pin 2                   BLE_CHK: pin 15
   I/O: pin 3                       VCC: pin 9
   I/O: pin 5                        TX: pin 2
   I/O: pin 6                        RX: pin 4
*/
#include "LowPower.h"
#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define MIN_V 3150 // battery empty level
#define MAX_V 3850 // battery full level
#define MAX_BLE_WAITx8 6 // Maximum bluetooth re-connect time 4x8=32+ seconds
#define SLEEP_TIME 37 // SleepTime in 37x8 seconds = 4 min 56 seconds
#define MAX_NFC_READTRIES 2 // Amount of tries for every nfc block-scan

const int SSPin = 10;  // Slave Select pin
const int IRQPin = 9;  // Sends wake-up pulse for BM019
const int MOSIPin = 11;
const int MISOPin = 12;
const int SCKPin = 13;
const int NFCPin1 = 7; // Power pin BM019
const int NFCPin2 = 8; // Power pin BM019
const int NFCPin3 = 4; // Power pin BM019

const int BLEPin = 3; // BLE power pin.
const int BLEState = 2; // BLE connection state pin
const int BLE_RX =5; //!!!!!!!!!!!!!!!!! 5
const int BLE_TX = 6;//!!!!!!!!!!!!!!!!!! 6

// search replace Serial.p Serial.b without //
bool startHM17() ;//schaltet pins, wartet auf connection
bool endHM17();//pin power off
bool FirstRunHM17 = true; // config HM-17 NAME and BLEState
bool endBM019(); //pin power off.

void goToSleep( int ); // goToSleep for , int x 8 seconds
//void wakeUp(); // depricate; solved via separate startHM17 and startBM019
String xdripPacket;
bool RC;
bool HM17= false;
bool BM019 =false;
int ERROR_READS; //How many error tries occured, higher longer sleep
String sendAT(String, char ); // function to send AT commands to BLE chip
int ERRORCODE = 2;// the 2 of fe "216" xdrippacket

//NFC Antenna http://www.antenna-theory.com/definitions/nfc-antenna.php

byte RXBuffer[24];
byte NFCReady = 0;  // used to track NFC state
byte FirstRun = 1;
byte batteryLow;
int batteryPcnt;
long batteryMv;
int noDiffCount = 0;
int sensorMinutesElapse;
float lastGlucose;
float trend[16];


SoftwareSerial ble_Seril(BLE_RX, BLE_TX); // RX | TX


String sendAT(String packet, char BLE_Command[])
{

  int i_counter=0;

  String string_Answer="";
 Serial.println("XXX" + string_Answer + "XXX");

  delay(100);
  ble_Seril.listen();
  if(packet=="") ble_Seril.write(BLE_Command);
  if(packet!="") ble_Seril.print(packet);


  delayMicroseconds(100);

  while(ble_Seril.available() <= 0 ){

//LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      if (i_counter > (20000)) //LL18 war 5000
      {
       Serial.println(String(BLE_Command) + ": " + i_counter);
            return String(i_counter);
      }
      delayMicroseconds(100); //LL18 500
      i_counter = i_counter +1;

  }
int i;
  for(i = 0; i < 100 ; i++)
  {
    delayMicroseconds(10);
ble_Seril.listen();
      while(ble_Seril.available()>0)
      {char c_char = ble_Seril.read();
        string_Answer += c_char;
      }
    }

Serial.println(String(BLE_Command) + ": " + string_Answer);
return string_Answer;
}

void setup() {

   Serial.begin(9600);
    delay(100);
    endHM17();
    endHM17();
    endBM019();

}

bool startBM019(){
 Serial.println("startBM019");
  pinMode(IRQPin, OUTPUT);
  digitalWrite(IRQPin, HIGH);
  pinMode(SSPin, OUTPUT);
  digitalWrite(SSPin, HIGH);
  pinMode(MOSIPin, OUTPUT);
  pinMode(MISOPin, INPUT);
  pinMode(SCKPin, OUTPUT);

  pinMode(NFCPin1, OUTPUT);
  digitalWrite(NFCPin1, HIGH);
  pinMode(NFCPin2, OUTPUT);
  digitalWrite(NFCPin2, HIGH);
  pinMode(NFCPin3, OUTPUT);
  digitalWrite(NFCPin3, HIGH);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  delay(10);                      // send a wake up
  digitalWrite(IRQPin, LOW);      // pulse to put the
  delayMicroseconds(100);         // BM019 into SPI
  digitalWrite(IRQPin, HIGH);     // mode
  delay(10);
  digitalWrite(IRQPin, LOW);
  delay(100);
  return true;
}

bool endBM019(){
SPI.end();
  digitalWrite(SSPin, LOW);
  digitalWrite(MOSIPin, LOW);
   digitalWrite(MISOPin, LOW);
  digitalWrite(SCKPin, LOW);
  digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
  digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
  digitalWrite(NFCPin3, LOW);

   pinMode(SSPin, INPUT); //LL could only switch off the BM019 power LED, if set to input
   pinMode(MOSIPin, INPUT);
   pinMode(MISOPin, INPUT);
   pinMode(SCKPin, INPUT);
   pinMode(IRQPin, INPUT);

  digitalWrite(IRQPin, LOW);
  return true;
}
void restartBLE() {
  pinMode(BLEState, INPUT);
  pinMode(BLEPin, OUTPUT);
    digitalWrite(BLEPin, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    delay(500);
    ble_Seril.write("AT+RESET");
    delay(500);
    pinMode(BLEState, INPUT);
}
bool testhm17=false;
void setupHM17(){
  sendAT("","AT+VERS?");
  sendAT("","AT+VERR?");
  //String HM_Name = sendAT("","AT+NAME?");
  // HM11 !HM_Name.startsWith("OK+NAME:Leonhard")
  //HM17!HM_Name.startsWith("OK+GET:Leonhard")
  //if(!HM_Name.startsWith("OK+Get:Leonhard")) {
          sendAT("","AT+RENEW");
          sendAT("","AT+RESET");
          delay(500);
          digitalWrite(BLEPin, LOW);
          delay(500);
          digitalWrite(BLEPin, HIGH);
          delay(500);

          sendAT("","AT+NAMELeonhard");
          sendAT("","AT+VERS?");
          sendAT("","AT+VERR?");
          sendAT("","AT+NAME?");
          sendAT("","AT+NAMELeonhard");
sendAT("","AT+PIO1?");
                sendAT("","AT+PIO11"); // on connection BLEState durable light not blinking


                // 45. Query/Set Module Power
                // Send Receive Parameter
                // AT+POWE? OK+Get:<P1> None
                // AT+POWE <P1> OK+Set:<P1> Para: 0 ~ 7
                // 0: -18dbm
                // 1: -12dbm
                // 2: -6dbm
                // 3: -3dbm
                // 4: -2dbm
                // 5: -1dbm
                // 6: 0dbm
                // 7: 3dbm
                // Default: 6
                sendAT("","AT+POWE?");
                sendAT("","AT+RELI?");
                sendAT("","AT+RELI0");
                sendAT("","AT+PASS?");
sendAT("","AT+PASS000000");
                sendAT("","AT+PASS?");

                // 46. Query/Set reliable advertising mode
                // Send Receive Parameter
                // AT+RELI? OK+ Get:<P1>
                // AT+RELI<P1> OK+ Set:<P1>
                // Para1: 0, 1
                // 0: Normal advertising
                // 1: Reliable advertising
                // Default: 0
                //       52. Query/Set BLE talk method
          // Send Receive Parameter
          // AT+RESP? OK+Get:<P1> None
          // AT+RESP<P1> OK+Set:<P1> Para1: 0, 1, 2
          // 0: Writewithoutresponse
          // 1: Writewithresponse
          // 2: Both 0 and 1
          // Default: 0
          sendAT("","AT+RESP?");
          sendAT("","AT+RESP1");//LL35
          // 53. Query/Set PIO0 function (System KEY)
          // Send Receive Parameter
          // AT+SYSK? OK+Get:[P]
          // AT+SYSK[P] OK+Set:[P]
          // Para1: 0, 1
          // 0: Only cancel operate,
          // 1: When module is
          // standby, restore factory
          // setting.
          // Default: 1
          sendAT("", "AT+PWRM?");
          sendAT("","AT+PASS?");

//}
sendAT("","AT+RESET");
delay(500);
digitalWrite(BLEPin, LOW);
delay(500);
digitalWrite(BLEPin, HIGH);
delay(100);
sendAT("","AT+RESET");
}
bool startHM17() {
 Serial.println("startHM17");
 endHM17();
  restartBLE();
  //return true;
  //LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);

if(testhm17==true)setupHM17();
ble_Seril.begin(9600);
sendAT("","AT+RESET");

  String ble_NameResponse = sendAT("","AT+NAME?");
  String ble_ConnectedResponse = sendAT("","AT");
//seems wrong, only the first AT gives sometimes OK, after connection no AT commands. means = 20001
  if (ble_NameResponse== "OK+Get:Leonhard"&& ble_ConnectedResponse == "OK" && digitalRead(BLEState) == HIGH) return true; // Already setup AND responsive
//maybe
//  if (digitalRead(BLEState) == HIGH && ble_NameResponse== "20001"&& ble_ConnectedResponse == "20001"  ) return true; // Already setup AND responsive

bool BLEState_current=false;
                  for (int i_wakeup=0; (i_wakeup < 90) ; i_wakeup++)
        {
          Serial.println("Waiting for BLE OK ...");
          delay(50);
          if(digitalRead(BLEState) == LOW)   BLEState_current = false;
          //if(digitalRead(BLEState) == HIGH)  BLEState_current = true;
          if(BLEState_current == false)   digitalWrite(BLEPin, LOW);
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
          if(BLEState_current == false)    digitalWrite(BLEPin, HIGH);
              LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF); //LL48  delay(500);
            if(BLEState_current == false)      sendAT("","AT+RESET");

     sendAT("","AT+RESET");
          ble_NameResponse = sendAT("","AT+NAME?");
          ble_ConnectedResponse = sendAT("","AT");
          ble_ConnectedResponse = sendAT("","OK");

         if(digitalRead(BLEState) == HIGH) BLEState_current = true;

         Serial.print("BLEState_current " + String( BLEState_current));
         delay(50);
         if ( BLEState_current == true) return true; // Already setup AND responsive
         //HM17 HM11
         if( !ble_NameResponse.startsWith( "OK+Get:Leonhard") && !ble_NameResponse.startsWith( "OK+NAME:Leonhard")){  setupHM17();          }
        }

      return false;
}

bool endHM17(){
ble_Seril.end();
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(BLEPin, LOW);
  digitalWrite(BLEState, LOW);
  delay(50);
  return true;
}

bool SetProtocol_Command() {

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);

  for (long i; (RXBuffer[0] != 8 ) && (i < 100000001L);i++)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
if (i==100000000L) return false;
  }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))  // is response code good?
    {
   Serial.println("Protocol Set Command OK");
    NFCReady = 1; // NFC is ready
    return  true; // NFC is ready
    }
  else
    {
   Serial.println("Protocol Set Command FAIL");
    NFCReady = 0; // NFC not ready
    return  false; // NFC not ready
    }
}

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 8.5);
}

bool Inventory_Command() {

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);

     for (long i; (RXBuffer[0] != 8 ) && (i < 100000001L);i++)
       {
       RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
       RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
if (i==100000000L) return false;
     }

  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (byte i=0;i<RXBuffer[1];i++)
      RXBuffer[i+2]=SPI.transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  delay(1);

  if (RXBuffer[0] == 128)  // is response code good?
    {
   Serial.println("Sensor in range ... OK");
    NFCReady = 2;
    return  true;
    }
  else
    {
   Serial.println("Sensor out of range");
      NFCReady = 1;
    return  false;
    }
 }

float Read_Memory() {

 byte oneBlock[8];
 String hexPointer = "";
 String trendValues = "";
 String hexMinutes = "";
 String elapsedMinutes = "";
 float trendOneGlucose;
 float trendTwoGlucose;
 float currentGlucose;
 float shownGlucose;
 float averageGlucose = 0;
 int glucosePointer;
 int validTrendCounter = 0;
 float validTrend[16];
 byte readError = 0;
 int readTry;

 for ( int b = 3; b < 16; b++) {
  readTry = 0;
  do {
  readError = 0;
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(b);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  for (long i; (RXBuffer[0] != 8 ) && (i < 100000001L);i++)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
if (i==100000000L) return false;
  }

  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
 for (byte i=0;i<RXBuffer[1];i++)
   RXBuffer[i+2]=SPI.transfer(0);  // data
 if (RXBuffer[0] != 128)
     readError = 1;
  digitalWrite(SSPin, HIGH);
  delay(1);

 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];

  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
  if (!readError)       // is response code good?
  {
   Serial.println(str);
    trendValues += str;
  }
  readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );

 }
  readTry = 0;
  do {
  readError = 0;
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(39);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  for (long i; (RXBuffer[0] != 8 ) && (i < 100000001L);i++)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if (i==100000000L) return false;
  }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
 for (byte i=0;i<RXBuffer[1];i++)
   RXBuffer[i+2]=SPI.transfer(0);  // data
 if (RXBuffer[0] != 128)
     readError = 1;
  digitalWrite(SSPin, HIGH);
  delay(1);

 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];

  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
  if (!readError)
    elapsedMinutes += str;
  readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );

  if (!readError)
    {
      hexMinutes = elapsedMinutes.substring(10,12) + elapsedMinutes.substring(8,10);
      hexPointer = trendValues.substring(4,6);
      sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
      glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);

     Serial.println("");
     Serial.print("Glucose pointer: ");
     Serial.print(glucosePointer);
     Serial.println("");

      int ii = 0;
      for (int i=8; i<=200; i+=12) {
        if (glucosePointer == ii)
        {
          if (glucosePointer == 0)
          {
            String trendNow = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendOne = trendValues.substring(178,180) + trendValues.substring(176,178);
            String trendTwo = trendValues.substring(166,168) + trendValues.substring(164,166);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));

            if (FirstRun == 1)
               lastGlucose = currentGlucose;

            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else if (glucosePointer == 1)
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendTwo = trendValues.substring(178,180) + trendValues.substring(176,178);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));

            if (FirstRun == 1)
               lastGlucose = currentGlucose;

            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(i-22,i-20) + trendValues.substring(i-24,i-22);
            String trendTwo = trendValues.substring(i-34,i-32) + trendValues.substring(i-36,i-34);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));


            if (FirstRun == 1)
               lastGlucose = currentGlucose;

            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
        }

        ii++;
      }

     for (int i=8, j=0; i<200; i+=12,j++) {
          String t = trendValues.substring(i+2,i+4) + trendValues.substring(i,i+2);
          trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL ,16));
       }

    for (int i=0; i<16; i++)
    {
      if (((lastGlucose - trend[i]) > 50) || ((trend[i] - lastGlucose) > 50)) // invalid trend check
         continue;
      else
      {
         validTrend[validTrendCounter] = trend[i];
         validTrendCounter++;
      }
    }

    if (validTrendCounter > 0)
    {
      for (int i=0; i < validTrendCounter; i++)
         averageGlucose += validTrend[i];

      averageGlucose = averageGlucose / validTrendCounter;

      if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
         shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      else
         shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value

    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;

    if (currentGlucose != 0)
      lastGlucose = currentGlucose;


    NFCReady = 2;
    FirstRun = 0;

    if (noDiffCount > 3){
      ERRORCODE = 5; // Expired Sensor
      return 0;
    }// LL original was 5, means if over 5*5min value was the same then error, my feeling goes with 3

    else
      return shownGlucose;

    }
  else
    {
   Serial.print("Read Memory Block Command FAIL");
    NFCReady = 0;
    readError = 0;
    ERRORCODE = 6; // Read Memory Block Command FAIL
    }
    return 0;
 }



String Build_Packet(float glucose) {

// Let's build a String which xDrip accepts as a BTWixel packet

      unsigned long raw = glucose*1000; // raw_value
      String packet = "";
      packet = String(raw);
      packet += ' ';
packet += ERRORCODE;
      if (noDiffCount > 0) packet += "05"; //205 warning for expird sensor. Dont understand the xdrip code 205,206,209 all result in low transmitter battery. But at least a message.
      if (noDiffCount == 0) packet += "16"; // 216    sensor_battery_level = 216; //no message, just system status "OK"

      packet += ' ';
//if (noDiffCount > 0) batteryPcnt = batteryPcnt*100; //warning for expird sensor
      packet += String(batteryPcnt);

      packet += ' ';
      packet += String(sensorMinutesElapse);
     Serial.println("");
     Serial.print("Glucose level: ");
     Serial.print(glucose);
     Serial.println("");
     Serial.print("15 minutes-trend: ");
     Serial.println("");
      for (int i=0; i<16; i++)
      {
       Serial.print(trend[i]);
       Serial.println("");
      }
     Serial.print("Battery level: ");
     Serial.print(batteryPcnt);
     Serial.print("%");
     Serial.println("");
     Serial.print("Battery mVolts: ");
     Serial.print(batteryMv);
     Serial.print("mV");
     Serial.println("");
     Serial.print("Sensor lifetime: ");
     Serial.print(sensorMinutesElapse);
     Serial.print(" minutes elapsed");
     Serial.println("");
      return packet;
}

bool Send_Packet(String packet) { //LL47 is only invoked  if (digitalRead(BLEState) == HIGH)
   if ((packet.substring(0,1) != "0"))
    {
   ble_Seril.print(packet += String(" ") ); //+= packet += String(" ") one packet should be ok.
      Serial.print("SendPacket");
      Serial.println("");
      Serial.print("xDrip packet: ");
      Serial.print(packet);
      Serial.println("");
    }
    else{
      packet = String("ERR 916 ") + String(batteryPcnt) + ' ' + String("30000") + String(" ");
     //+= packet += String(" ") one packet should be ok.

      ble_Seril.print(packet);
        Serial.print("SendPacket");
         Serial.println("");
         Serial.print("xDrip packet: ");
         Serial.print(packet);
         Serial.println("");
    }


    if (digitalRead(BLEState) == HIGH)  {LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); // ble up, atmel not needed -> down.
      //LL47 if the BLEState Pin is up at the beginnening,after the ble print and after 1second time, it should have succeded
   if (digitalRead(BLEState) == HIGH)return true;}
      return false;

  }

int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  batteryMv = (high<<8) | low;

  batteryMv = 1125300L / batteryMv; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  int batteryLevel = min(map(batteryMv, MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
  return batteryLevel;
}

void goToSleep( int time) {
 Serial.println("goToSleep!" + String(time));
  delay(100); //LL direct powerdown sleep after serial data, hangs Atom serial software?

 for (int i=0; i<time; i++) {
   LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
 }
}


void lowBatterySleep() {

 SPI.end();
 digitalWrite(MOSIPin, LOW);
 digitalWrite(SSPin, LOW);
 digitalWrite(MOSIPin, LOW);
 digitalWrite(SCKPin, LOW);
 digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
 digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
 digitalWrite(NFCPin3, LOW);
 // digitalWrite(NFCPin4, LOW);
 // digitalWrite(NFCPin5, LOW);
 digitalWrite(IRQPin, LOW);
 digitalWrite(5, LOW);
 digitalWrite(6, LOW);
 digitalWrite(BLEPin, LOW);

Serial.print("Battery low! LEVEL: ");
Serial.print(batteryPcnt);
Serial.print("%");
Serial.println("");
 delay(100);

 // Switch LED on and then off shortly
    for (int i=0; i<10; i++) {
      digitalWrite(SCKPin, HIGH);
      delay(50);
      digitalWrite(SCKPin, LOW);
      delay(100);
    }

 MCUSR = 0;
 WDTCSR |= 0b00011000;
 WDTCSR =  0b01000000 | 0b100001;
 set_sleep_mode (SLEEP_MODE_PWR_DOWN);
 sleep_enable();
 sleep_cpu();
 sleep_disable();
 power_all_enable();
 wdt_reset();
}

bool runBM019(){
  startBM019();
  startBM019();
  delay(200);

 Serial.println("RC==" + String(RC));
  if(RC==true){Serial.println("SetProtocol_Command");
  RC = SetProtocol_Command();
if(RC==false){ERRORCODE = 3;}} // Error in SetProtocol_Command

  if(RC==true){
        for(int i=0; i<3;i++) {
           Serial.println("Inventory_Command");
            delay(100);
              RC = Inventory_Command();
              if (RC == true) i = 4;
                      }
        if(RC==false){ERRORCODE = 4;} //Error in Inventory_Command
  }


  if(RC==true)  {
         Serial.println("xdripPacket");
          xdripPacket = Build_Packet(Read_Memory());
          return true;
  }
  xdripPacket = Build_Packet(0);
    return false;
}

void loop() {

  batteryPcnt = readVcc();
 Serial.println(batteryPcnt);
  if (batteryPcnt < 1) {
    batteryLow = 1;
    endBM019();
  endHM17();
}

  while (batteryLow == 1)
  {
    goToSleep ( 1);
    batteryPcnt = readVcc();
    if (batteryPcnt > 10)
    {
      batteryLow = 0;
      //delay(100);
    }
  }
RC=true;
NFCReady = 0;
ERRORCODE = 2;

// if last try for BLE connection didnt succed, try BLE first, only if ok then run NFC reading first.

if(HM17==false ){ //LL34 false
    HM17 = startHM17();
        if(HM17==true){
            BM019 = runBM019();
            endBM019();
            endBM019();
            if(BM019 == true) RC = Send_Packet(xdripPacket);
            endHM17();
            endHM17();
        }
        else{  RC = false;}
}
// if last BLE connection was ok, but NFC not -> try NFC reading first bevor BLE connection.
else{ // HM17==true
      BM019 = runBM019();
      endBM019();
      endBM019();
      //try ble connectioneven if nfc not ok do deliver error codes
      //if(BM019==true){
              HM17 = startHM17();
              if(HM17==true) RC = Send_Packet(xdripPacket);
              endHM17();
              endHM17();
      //}
  //else {RC = false;}
  if(!BM019) RC = false;
  if(!HM17) RC = false;
}



    //if(batteryPcnt<=80) digitalWrite(14, LOW);
//delay(4000);
if(RC==true) { // if current try was ok, to to sleep for normal ca 5min
      goToSleep( SLEEP_TIME);
    ERROR_READS=0;
    }
if(RC==false) { // if current try didnt succed, then sleep for 24,48,72,96,120,144,168, 192,216
      ERROR_READS++;

       if(ERROR_READS <= 9) goToSleep( 2*ERROR_READS);
       if(ERROR_READS > 9) goToSleep( SLEEP_TIME);

    }


    delay(1);
}
