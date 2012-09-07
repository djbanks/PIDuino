/*******************************************************************************
* The osPID Kit comes with swappable IO cards which are supported by different
* device drivers & libraries. For the osPID firmware to correctly communicate with
* your configuration, you must uncomment the appropriate "define" statements below.
* Please take note that only 1 input card and 1 output card can be used at a time. 
* List of available IO cards:
*
* Input Cards
* ===========
* 1. TEMP_INPUT_V110:
*    Temperature Basic V1.10 with 1 thermistor & 1 type-K thermocouple (MAX6675)
*    interface.
* 2. TEMP_INPUT_V120:
*    Temperature Basic V1.20 with 1 thermistor & 1 type-K thermocouple 
*    (MAX31855KASA) interface.
* 3. PROTOTYPE_INPUT:
*    Generic prototype card with input specified by user. Please add necessary
*    input processing in the section below.
*
* Output Cards
* ============
* 1. DIGITAL_OUTPUT_V120: 
*    Output card with 1 SSR & 2 relay output.
* 2. DIGITAL_OUTPUT_V150: 
*    Output card with 1 SSR & 2 relay output. Similar to V1.20 except LED mount
*    orientation.
* 3. PROTOTYPE_OUTPUT:
*    Generic prototype card with output specified by user. Please add necessary
*    output processing in the section below.
*
* This file is licensed under Creative Commons Attribution-ShareAlike 3.0 
* Unported License.
*
*******************************************************************************/

// ***** INPUT CARD *****
//#define TEMP_INPUT_V110
//#define TEMP_INPUT_V120
#define PROTOTYPE_INPUT

// ***** OUTPUT CARD *****
//#define DIGITAL_OUTPUT_V120
#define SSR_OUTPUT
//#define PROTOTYPE_OUTPUT

union {                // This Data structure lets
  byte asBytes[32];    // us take the byte array
  float asFloat[8];    // sent from processing and
}                      // easily convert it to a
serialXfer;            // float array
byte b1,b2;


#ifdef PROTOTYPE_INPUT
 /*Include any libraries and/or global variables here*/

#include "RTDModule_local.h"
RTDModule rtd;

float flt1_i=0, flt2_i=0, flt3_i=0, flt4_i=0;
byte bt1_i=0, bt2_i=0, bt3_i=0, bt4_i=0;

void EEPROMBackupInputParams(int offset)
{
  /*EEPROM_writeAnything(offset, bt1_i);
  EEPROM_writeAnything(offset+1, bt2_i);
  EEPROM_writeAnything(offset+2, bt3_i);
  EEPROM_writeAnything(offset+3, bt4_i);  
  EEPROM_writeAnything(offset+4,flt1_i);
  EEPROM_writeAnything(offset+8,flt2_i);
  EEPROM_writeAnything(offset+12,flt3_i);
  EEPROM_writeAnything(offset+16,flt4_i);*/
}

void EEPROMRestoreInputParams(int offset)
{
  /*EEPROM_readAnything(offset, bt1_i);
  EEPROM_readAnything(offset+1, bt2_i);
  EEPROM_readAnything(offset+2, bt3_i);
  EEPROM_readAnything(offset+3, bt4_i);  
  EEPROM_readAnything(offset+4,flt1_i);
  EEPROM_readAnything(offset+8,flt2_i);
  EEPROM_readAnything(offset+12,flt3_i);
  EEPROM_readAnything(offset+16,flt4_i);*/
}

void InitializeInputCard()
{
  rtd.calibration(0.148741418, -7.288329519);
  analogReference(INTERNAL);
}

void InputSerialReceiveStart()
{
}

void InputSerialReceiveDuring(byte val, byte index)
{
  if(index==1) bt1_i = val;
  else if(index==2) bt2_i = val;
  else if(index==3) bt3_i = val;
  else if(index==4) bt4_i = val;
  else if(index<22) serialXfer.asBytes[index-5] = val; 
}

void InputSerialReceiveAfter(int eepromOffset)
{
  flt1_i = serialXfer.asFloat[0];
  flt2_i = serialXfer.asFloat[1];
  flt3_i = serialXfer.asFloat[2];
  flt4_i = serialXfer.asFloat[3];  

  EEPROMBackupInputParams(eepromOffset);
}

void InputSerialSend()
{
  Serial.print(int(bt1_i)); 
  Serial.print(" "); 
  Serial.print(int(bt2_i)); 
  Serial.print(" "); 
  Serial.print(int(bt3_i)); 
  Serial.print(" "); 
  Serial.print(int(bt4_i)); 
  Serial.print(" ");   
  Serial.print(flt1_i); 
  Serial.print(" ");  
  Serial.print(flt2_i); 
  Serial.print(" ");  
  Serial.print(flt3_i);   
  Serial.print(" ");  
  Serial.println(flt4_i);   
}

void InputSerialID()
{
  Serial.print(" IID0"); 
}

double ReadInputFromCard()
{
  return rtd.getTemperature();
}
#endif /*PROTOTYPE_INPUT*/


#ifdef SSR_OUTPUT
const byte SSRPin = A1;
//unsigned long windowStartTime;
double outWindowSec = 5.0;
unsigned long WindowSize = 5000;

void setOutputWindow(double val)
{
  unsigned long temp = (unsigned long)(val*1000);
  if(temp<500)temp = 500;
  outWindowSec = (double)temp/1000;
  if(temp!=WindowSize)
  {
    WindowSize = temp;
  } 
}

void InitializeOutputCard()
{
  pinMode(SSRPin, OUTPUT);
}

void OutputSerialReceiveStart()
{
}

void OutputSerialReceiveDuring(byte val, byte index)
{
  if(index==1) b1 = val;
  else if(index<6) serialXfer.asBytes[index-2] = val; 
}

void OutputSerialReceiveAfter(int eepromOffset)
{
  digitalWrite(SSRPin, LOW);
  outWindowSec =  serialXfer.asFloat[0];
  setOutputWindow(outWindowSec);
}

void OutputSerialID()
{
  Serial.print(" OID1"); 
}

void WriteToOutputCard(double value)
{
  unsigned long wind = millis() % WindowSize;
  unsigned long oVal = (unsigned long)(value*(double)WindowSize/ 100.0);
  digitalWrite(SSRPin ,(oVal>wind) ? HIGH : LOW);
}

// Serial send & receive
void OutputSerialSend()
{
  Serial.print((int)1); 
  Serial.print(" ");  
  Serial.println(outWindowSec); 
}
#endif /*SSR_OUTPUT*/
