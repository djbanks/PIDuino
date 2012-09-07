#include <EEPROM.h>
#include "PID_v1_local.h"
#include "EEPROMAnything.h"
#include "io.h"
#include <RTDModule_local.h>
#include <Wire.h>

// Rot Enc
#include <PinChangeInt.h> // necessary otherwise we get undefined reference errors.
#include <AdaEncoder.h>
#define a_PINA 2
#define a_PINB 3
int8_t clicks=0;
char id=0;

//RTC
int timeHours, timeMins;
float startTime;

// OLED Display
#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(10, 8);
int dWidth = 128;
int dHeight = 64;

const byte EEPROM_ID = 2;


unsigned long now, lcdTime, buttonTime,ioTime, serialTime;
boolean sendInfo=true, sendDash=true, sendTune=true, sendInputConfig=true, sendOutputConfig=true;

bool editing=false;
bool tuning = false;

double oldSetPoint=250,setpoint=250,input=250,output=50, pidInput=250;

double kp = 2, ki = 0.5, kd = 2;
byte ctrlDirection = 0;
byte modeIndex = 0;

PID myPID(&pidInput, &output, &setpoint,kp,ki,kd, DIRECT);

double aTuneStep = 20, aTuneNoise = 1;
unsigned int aTuneLookBack = 10;
byte ATuneModeRemember = 0;
//PID_ATune aTune(&pidInput, &output);

byte curProfStep=0;
byte curType=0;
float curVal=0;
float helperVal=0;
unsigned long helperTime=0;
boolean helperflag=false;
unsigned long curTime=0;


void setup()
{
  Serial.begin(9600);
  lcdTime=10;
  buttonTime=1;
  ioTime=5;
  serialTime=6;
  //windowStartTime=2;
  
  initRTC();

  delay(1000);

  initializeEEPROM();


  InitializeInputCard();
  InitializeOutputCard();
  
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, 100);
  myPID.SetTunings(kp, ki, kd);
  myPID.SetControllerDirection(ctrlDirection);
  myPID.SetMode(modeIndex);
  
  
  // buttons
  //pinMode(2, INPUT);
  //pinMode(3, INPUT);
  pinMode(4, INPUT);
  //digitalWrite(2, HIGH);
  //digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  
  AdaEncoder::addEncoder('a', a_PINA, a_PINB);
}

void loop()
{
  now = millis();

  bool doIO = now >= ioTime;
  //read in the input
  if(doIO)
  { 
    ioTime+=250;
    
    input =  ReadInputFromCard();
    if(!isnan(input))pidInput = input;
  }

  //allow the pid to compute if necessary
  myPID.Compute();

  if(doIO)
  {
    //send to output card
    WriteToOutputCard(output);
  }

  if(now>lcdTime)
  {
    u8g.firstPage();  
    do {
      draw();
    } while( u8g.nextPage() );
    
    readRTC();
    
    lcdTime+=250; 
  }
  if(millis() > serialTime)
  {
    //if(receivingProfile && (now-profReceiveStart)>profReceiveTimeout) receivingProfile = false;
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
  
  if(now>buttonTime)
  {
    checkButtons();
    
    
    buttonTime += 50;
  }
  
  
}

void draw(void) {
  //black on white
  u8g.setColorIndex(1);
  u8g.drawFrame(0,0,dWidth,dHeight);

// Time
  u8g.setFont(u8g_font_6x10);
  u8g.setPrintPos(20, 11);
  u8g.print(timeHours);
  u8g.print(":");
  u8g.print(timeMins);
  
// Params
  u8g.setFont(u8g_font_04b_03r);
  u8g.setPrintPos(70, 7);
  u8g.print("p:"); u8g.print(kp);
  u8g.setPrintPos(99, 7);
  u8g.print(" i:"); u8g.print(ki);
  u8g.setPrintPos(70, 14);
  u8g.print("d:"); u8g.print(kd);
  u8g.setPrintPos(97, 14);
  u8g.print(" o:"); u8g.print((int)output); u8g.print("%");
  
  
  u8g.drawBox(0,15,dWidth,dHeight - 15 - 15);
  // white on black
  u8g.setColorIndex(0);
  
  u8g.setFont(u8g_font_04b_03r);
  u8g.setPrintPos(3, 47);
  u8g.print("TIMER:");

  u8g.drawStr270( 75, 46, "SET");
  u8g.setFont(u8g_font_fub25n);

  u8g.setPrintPos(28, 45); 
  int elapsedTime = getTimerSeconds();
  if(elapsedTime<10)
    u8g.print("0");
  u8g.print(elapsedTime);
  
// temperature
  u8g.setFont(u8g_font_fub11);
  u8g.setPrintPos(74, 29);
  char tmp1[10];
  dtostrf(input,2,1,tmp1);
  u8g.print(tmp1);
  u8g.setPrintPos(74, 36+10);
  dtostrf(setpoint,2,1,tmp1);
  u8g.print(tmp1);
// temp symbols
  u8g.setFont(u8g_font_m2icon_5);
  u8g.setPrintPos(117, 23);
  u8g.print("Q");
  u8g.setPrintPos(117, 30+10);
  u8g.print("Q");
  u8g.setFont(u8g_font_04b_03r);
  u8g.setPrintPos(121, 29);
  u8g.print("C");
  u8g.setPrintPos(121, 36+10);
  u8g.print("C");
  
  //black on white
  u8g.setColorIndex(1);
  u8g.drawVLine(dWidth/2, dHeight - 15, 15);
  
// buttons
  u8g.setFont(u8g_font_6x10);
  u8g.setPrintPos(23, 60);
  u8g.print("STOP");
  u8g.setPrintPos(80, 60);
  u8g.print("CANCEL");
  
}






const int eepromTuningOffset = 1; //13 bytes
const int eepromDashOffset = 14; //9 bytes
//const int eepromATuneOffset = 23; //12 bytes
//const int eepromProfileOffset = 35; //136 bytes
const int eepromInputOffset = 172; //? bytes (depends on the card)
const int eepromOutputOffset = 300; //? bytes (depends on the card)
void initializeEEPROM()
{
  //read in eeprom values
  byte firstTime = EEPROM.read(0);
  if(firstTime!=EEPROM_ID)
  {//the only time this won't be 1 is the first time the program is run after a reset or firmware update
    //clear the EEPROM and initialize with default values
    for(int i=1;i<1024;i++) EEPROM.write(i,0);
    EEPROMBackupTunings();
    EEPROMBackupDash();
    //EEPROMBackupATune();
    EEPROMBackupInputParams(eepromInputOffset);
    //EEPROMBackupOutputParams(eepromOutputOffset);
    //EEPROMBackupProfile();
    EEPROM.write(0,EEPROM_ID); //so that first time will never be true again (future firmware updates notwithstanding)
  }
  else
  {
    EEPROMRestoreTunings();
    EEPROMRestoreDash();
    //EEPROMRestoreATune();
    EEPROMRestoreInputParams(eepromInputOffset);
    //EEPROMRestoreOutputParams(eepromOutputOffset);
    //EEPROMRestoreProfile();    
  }
}
void EEPROMreset()
{
  EEPROM.write(0,0);
}
void EEPROMBackupTunings()
{
  EEPROM.write(eepromTuningOffset,ctrlDirection);
  EEPROM_writeAnything(eepromTuningOffset+1,kp);
  EEPROM_writeAnything(eepromTuningOffset+5,ki);
  EEPROM_writeAnything(eepromTuningOffset+9,kd);
}

void EEPROMRestoreTunings()
{
  ctrlDirection = EEPROM.read(eepromTuningOffset);
  EEPROM_readAnything(eepromTuningOffset+1,kp);
  EEPROM_readAnything(eepromTuningOffset+5,ki);
  EEPROM_readAnything(eepromTuningOffset+9,kd);
}

void EEPROMBackupDash()
{
  EEPROM.write(eepromDashOffset, (byte)myPID.GetMode());
  EEPROM_writeAnything(eepromDashOffset+1,setpoint);
  EEPROM_writeAnything(eepromDashOffset+5,output);
}

void EEPROMRestoreDash()
{
  modeIndex = EEPROM.read(eepromDashOffset);
  EEPROM_readAnything(eepromDashOffset+1,setpoint);
  EEPROM_readAnything(eepromDashOffset+5,output);
}

/*void EEPROMBackupATune()
{
  EEPROM_writeAnything(eepromATuneOffset,aTuneStep);
  EEPROM_writeAnything(eepromATuneOffset+4,aTuneNoise);
  EEPROM_writeAnything(eepromATuneOffset+8,aTuneLookBack);
}

void EEPROMRestoreATune()
{
  EEPROM_readAnything(eepromATuneOffset,aTuneStep);
  EEPROM_readAnything(eepromATuneOffset+4,aTuneNoise);
  EEPROM_readAnything(eepromATuneOffset+8,aTuneLookBack);
}*/




/*void changeAutoTune()
{
  if(!tuning)
  {
    //initiate autotune
    AutoTuneHelper(true);
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{

  if(start)
  {
    ATuneModeRemember = myPID.GetMode();
    myPID.SetMode(MANUAL);
  }
  else
  {
    modeIndex = ATuneModeRemember;
    myPID.SetMode(modeIndex);
  } 
}
*/




/********************************************
 * Serial Communication functions / helpers
 ********************************************/

boolean ackDash = false, ackTune = false;
union {                // This Data structure lets
  byte asBytes[32];    // us take the byte array
  float asFloat[8];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats
void SerialReceive()
{

  // read the bytes sent from Processing
  byte index=0;
  byte identifier=0;
  byte b1=255,b2=255;
  boolean boolhelp=false;

  while(Serial.available())
  {
    byte val = Serial.read();
    if(index==0){ 
      identifier = val;
      Serial.println(int(val));
    }
    else 
    {
      switch(identifier)
      {
      case 0: //information request 
        if(index==1) b1=val; //which info type
        else if(index==2)boolhelp = (val==1); //on or off
        break;
      case 1: //dasboard
      case 2: //tunings
      case 3: //autotune
        if(index==1) b1 = val;
        else if(index<14)foo.asBytes[index-2] = val; 
        break;
      case 4: //EEPROM reset
        if(index==1) b1 = val; 
        break;
      case 5: //input configuration
        if (index==1)InputSerialReceiveStart();
        InputSerialReceiveDuring(val, index);
        break;
      case 6: //output configuration
        if (index==1)OutputSerialReceiveStart();
        OutputSerialReceiveDuring(val, index);
        break;
      case 7:  //receiving profile
        //if(index==1) b1=val;
        //else if(b1>=nProfSteps) profname[index-2] = char(val); 
        //else if(index==2) proftypes[b1] = val;
        //else foo.asBytes[index-3] = val;

        break;
      case 8: //profile command
        //if(index==1) b2=val;
        break;
      default:
        break;
      }
    }
    index++;
  }

  //we've received the information, time to act
  switch(identifier)
  {
  case 0: //information request
    switch(b1)
    {
    case 0:
      sendInfo = true; 
      sendInputConfig=true;
      sendOutputConfig=true;
      break;
    case 1: 
      sendDash = boolhelp;
      break;
    case 2: 
      sendTune = boolhelp;
      break;
    case 3: 
      sendInputConfig = boolhelp;
      break;
    case 4: 
      sendOutputConfig = boolhelp;
      break;
    default: 
      break;
    }
    break;
  case 1: //dashboard
    if(index==14  && b1<2)
    {
      setpoint=double(foo.asFloat[0]);
      //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
      //   value of "Input"  in most cases (as 
      //   in this one) this is not needed.
      if(b1==0)                       // * only change the output if we are in 
      {                                     //   manual mode.  otherwise we'll get an
        output=double(foo.asFloat[2]);      //   output blip, then the controller will 
      }                                     //   overwrite.

      if(b1==0) myPID.SetMode(MANUAL);// * set the controller mode
      else myPID.SetMode(AUTOMATIC);             //
      EEPROMBackupDash();
      ackDash=true;
    }
    break;
  case 2: //Tune
    if(index==14 && (b1<=1))
    {
      // * read in and set the controller tunings
      kp = double(foo.asFloat[0]);           //
      ki = double(foo.asFloat[1]);           //
      kd = double(foo.asFloat[2]);           //
      ctrlDirection = b1;
      myPID.SetTunings(kp, ki, kd);            //    
      if(b1==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
      else myPID.SetControllerDirection(REVERSE);          //
      EEPROMBackupTunings();
      ackTune = true;
    }
    break;
  case 3: //ATune
/*    if(index==14 && (b1<=1))
    {

      aTuneStep = foo.asFloat[0];
      aTuneNoise = foo.asFloat[1];    
      aTuneLookBack = (unsigned int)foo.asFloat[2];
      if((!tuning && b1==1)||(tuning && b1==0))
      { //toggle autotune state
        changeAutoTune();
      }
      EEPROMBackupATune();
      ackTune = true;   
    }*/
    break;
  case 4: //EEPROM reset
    if(index==2 && b1<2) EEPROM.write(0,0); //eeprom will re-write on next restart
    break;
  case 5: //input configuration
    InputSerialReceiveAfter(eepromInputOffset);
    sendInputConfig=true;
    break;
  case 6: //ouput configuration
    OutputSerialReceiveAfter(eepromOutputOffset);
    sendOutputConfig=true;
    break;
  case 7: //receiving profile

    /*if((index==11 || (b1>=nProfSteps && index==9) ))
    {
      if(!receivingProfile && b1!=0)
      { //there was a timeout issue.  reset this transfer
        receivingProfile=false;
        Serial.println("ProfError");
        EEPROMRestoreProfile();
      }
      else if(receivingProfile || b1==0)
      {
        if(runningProfile)
        { //stop the current profile execution
          StopProfile();
        }
          
        if(b1==0)
        {
          receivingProfile = true;
          profReceiveStart = millis();
        }

        if(b1>=nProfSteps)
        { //getting the name is the last step
          receivingProfile=false; //last profile step
          Serial.print("ProfDone ");
          Serial.println(profname);
          EEPROMBackupProfile();
          Serial.println("Archived");
        }
        else
        {
          profvals[b1] = foo.asFloat[0];
          proftimes[b1] = (unsigned long)(foo.asFloat[1] * 1000);
          Serial.print("ProfAck ");
          Serial.print(b1);           
          Serial.print(" ");
          Serial.print(proftypes[b1]);           
          Serial.print(" ");
          Serial.print(profvals[b1]);           
          Serial.print(" ");
          Serial.println(proftimes[b1]);           
        }
      }
    }*/
    break;
  case 8:
    /*if(index==2 && b2<2)
    {
      if(b2==1) StartProfile();
      else StopProfile();

    }*/
    break;
  default: 
    break;
  }
}


// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  if(sendInfo)
  {//just send out the stock identifier
    Serial.print("\nosPID v1.50");
    InputSerialID();
    OutputSerialID();
    Serial.println("");
    sendInfo = false; //only need to send this info once per request
  }
  if(sendDash)
  {
    Serial.print("DASH ");
    Serial.print(setpoint); 
    Serial.print(" ");
    if(isnan(input)) Serial.print("Error");
    else Serial.print(input); 
    Serial.print(" ");
    Serial.print(output); 
    Serial.print(" ");
    Serial.print(myPID.GetMode());
    Serial.print(" ");
    Serial.println(ackDash?1:0);
    if(ackDash)ackDash=false;
  }
  if(sendTune)
  {
    Serial.print("TUNE ");
    Serial.print(myPID.GetKp()); 
    Serial.print(" ");
    Serial.print(myPID.GetKi()); 
    Serial.print(" ");
    Serial.print(myPID.GetKd()); 
    Serial.print(" ");
    Serial.print(myPID.GetDirection()); 
    Serial.print(" ");
    Serial.print(tuning?1:0);
    Serial.print(" ");
    Serial.print(aTuneStep); 
    Serial.print(" ");
    Serial.print(aTuneNoise); 
    Serial.print(" ");
    Serial.print(aTuneLookBack); 
    Serial.print(" ");
    Serial.println(ackTune?1:0);
    if(ackTune)ackTune=false;
  }
  if(sendInputConfig)
  {
    Serial.print("IPT ");
    InputSerialSend();
    sendInputConfig=false;
  }
  if(sendOutputConfig)
  {
    Serial.print("OPT ");
    OutputSerialSend();
    sendOutputConfig=false;
  }
  /*if(runningProfile)
  {
    Serial.print("PROF ");
    Serial.print(int(curProfStep));
    Serial.print(" ");
    Serial.print(int(curType));
    Serial.print(" ");
    switch(curType)
    {
      case 1: //ramp
        Serial.println((helperTime-now)); //time remaining
         
      break;
      case 2: //wait
        Serial.print(abs(input-setpoint));
        Serial.print(" ");
        Serial.println(curVal==0? -1 : float(now-helperTime));
      break;  
      case 3: //step
        Serial.println(curTime-(now-helperTime));
      break;
      default: 
      break;
      
    }

  }*/
  
}

void readRTC()
{
  Wire.beginTransmission(0x6f);   // transmit to device address 111 (0x6F)
  Wire.write(0);                                    // point to internal register “0″
  Wire.endTransmission();              // send bytes and stop transmission
  Wire.requestFrom(0x6f,6);       // reads bytes from sequential addresses
  byte Seconds = Wire.read();     // receive a byte
  byte Minutes = Wire.read();     // receive a byte
  byte Hours = Wire.read();       // receive a byte
  byte Day = Wire.read();         // receive a byte
  byte Date = Wire.read();        // receive a byte
  byte Month = Wire.read();       // receive a byte
  byte Year = Wire.read();        // receive a byte
  //Convert BCD seconds to total seconds and send to serial monitor
  timeHours=(((Hours&0x70)>>4)*10)+(Hours&0x0f);
  timeMins=(((Minutes&0x70)>>4)*10)+(Minutes&0x0f);
}

void initRTC()
{
  Wire.begin();
  Wire.beginTransmission(0x6f);
  Wire.write(0);
  Wire.write(0x80);
  Wire.endTransmission(0);
  delay(5);
  Wire.beginTransmission(0x6f);   // transmit to device address 111 (0x6F)
  Wire.write(7);                                     // point to internal register “7″
  Wire.write(0x40);                             // make MFP pin an output w/ 1HZ square wave
  Wire.endTransmission(0);            // send bytes and stop transmission
  delay(5);
}

void startTimer()
{
  startTime = millis();
}
int getTimerSeconds()
{
  int tSecs = (int)((millis() - startTime)/1000);
  if(tSecs > 60)
  {
    startTimer();
    tSecs = 0;
  }
  return tSecs;
}

void checkButtons()
{
  encoder *thisEncoder;
  thisEncoder=AdaEncoder::genie(&clicks, &id);
  if (thisEncoder != NULL)
  {
    if(clicks > 0)
    {
       setpoint = 5; 
    }
    else if(clicks < 0)
    {
      setpoint = 10;
    }
    else
    {
      setpoint = 55;
    }
    //setpoint += ((float)clicks)/2;
  }
  
  if(digitalRead(4)== LOW)
  {
    if(setpoint < 130)
    {
      oldSetPoint = setpoint;
      setpoint = 140;
    }
    else
    {
      setpoint = oldSetPoint;
    }
  }
}
