#include <EEPROM.h>
#include <SPI.h>
#include <avr/wdt.h>

/***
 * arduino-stim firmware
 ***/

enum OperationMode {
  NORMAL = 1,
  EMERGENCY_SHUTDOWN = 2,
};

const boolean DEBUG = true; // If true, the device will regularly send all internal parameters to the computer
const int BAUD_RATE = 9600;

// Message types
enum Parameter {
  actual_current = 1,
  target_current = 2,
  safety_limit = 3,
  absolute_limit = 4,
  scaling = 5,
  max_power_for_channel = 6,
  current_override = 7,
  polarity_value = 8,

  playback_speed = 10,
  playback_power = 11,

  ramping_speed = 20,
  ramping_limit = 21,

  tap_value = 30,
  voltage_value = 31,

  start = 100,
  stop = 101,
  start_playback = 102,
  stop_playback = 103,

  bad_packet = 255,
};

// For storing data to EEPROM
enum EEPROM_ADDRESSES {
  ADDR_SCALE_FACTOR = 2, // 4 bytes
  ADDR_CALIBRATE = 6,    // 1 byte
  ADDR_PROTOCOL_OK = 9,  // 1 byte
  ADDR_SAFETY_LIMIT = 10,// 4 bytes
  ADDR_MAX_POWER = 14,   // 4 bytes
  ADDR_MAX_RAMP = 18,    // 4 bytes
  ADDR_POWER_DELTA = 22, // 4 bytes
  ADDR_RUNNING_TIME = 26,// 4 bytes
};

float getValue(Parameter p);
void sendParams(int count, const Parameter* params);
void sendInternalState();
float getAvgCurrent();
void handleCommand(char* input, int bytecount);
void processSerialMessage();

/***********************************************/
/** ADJUST THE FOLLOWING RESISTANCE VALUE !!! **/
/***********************************************/
const float ABSOLUTE_MAX_LIMIT_CURRENT = 0.01; // 10 mA - WARNING: do not change this value, this is a hard limit that will trigger safety shutdown

const float RESISTANCE = 22; // Value in Ohms (so this is a small resistor)
const int INTERVAL_IN_MS = 250 * 4;
const int MAX_OVERCURRENT_ALERT_TIMEOUT = 2000; // In ms, after this amount of time in overcurrent, shutdown everything in emergency
const int FAST_CYCLE_PERIOD_USEC = 2000; // One fast cycle every 2 ms
const float MAX_RESISTANCE = 255/0.5; // System will assume that a usable connection is present as long as we can get at 0.2 mA at full power
const float SHAM_RAMPDOWN_RATE=0.006; // How fast current will ramp down during a sham session

const Parameter INTERNAL_STATE[7] = { 
  actual_current, target_current, safety_limit, scaling, tap_value, voltage_value, polarity_value };
const Parameter STATE_UPDATE[1] = { 
  actual_current };

float proportionalFactor = 0.1;
volatile float scale_factor = 1;

// Status
OperationMode operationMode; // NORMAL or EMERGENCY_SHUTDOWN
volatile float tapValue = 0; // Imput value of the digital potentiometer (from 0 to 255)
volatile long lastTransmit; // Timestamp (in milliseconds) of the last transmit time (last regular packet sent to the computer)
volatile boolean polarity = true; // true if positive, false otherwise
byte fastCycleCounter = 0; //count how many times the fast loop runs(for controlling sham purposes)

boolean gotSerial = false; // true is serial connection is available, false otherwise (playback mode)
boolean readEeprom = true; // Flag that indicates whether the EEPROM needs to be read (true) of has already been read (false)
boolean calibFlag = true;  // true if the device has not been calibrated

boolean shamMode = false;
boolean playback = false; // if true, the data is played back from file (simulation only, for test purposes)
int playbackSpeed = 0;
float playbackPower[100];
byte playbackEndPosition = 0;
byte playbackPosition = 0;
long lastPlaybackItemTime = millis();
volatile boolean ledStatus = false;

// Following are volatile because they are accessed via timer function
volatile float current; // in A (0.002 = 2mA)
volatile float connectionQualityIndex;
volatile float voltage;
volatile float raw4;
volatile float raw5;
volatile int a4out;
volatile int a5out;
volatile int displaySamples=0;
volatile float displayCurrent=0;
volatile float powerDelta = 0;

volatile float targetCurrent = 0;
volatile float outputCurrent = 0;

volatile float shamRampdown = 0;

// Output parameters (either set by controller or read from EEPROM
volatile float safetyLimit = ABSOLUTE_MAX_LIMIT_CURRENT; //maximum current permited through the system before a shutdown is initiated
volatile float shutdownCounter = 0; // counter incremented each time a current read is above the safetyLimit

volatile float overrideCurrent = 0; // variable to manually set channel 5 for diagnostics
volatile float originalTarget; // Target current when (before) entering sham mode

float maxRamp = 1; //maximum power the device will reach when ramping up if it never recieves a stop signal
float maxPower = ABSOLUTE_MAX_LIMIT_CURRENT; //maximum power values for each channel

byte tapOverride = -1;
boolean overrideTapOn = false;

const int COMPUTER_TIMEOUT = 30000; //how long to wait after the device is turned on before it decides that there is no computer connected and executes the protocol in EEPROM
float runningTimeSec = 0; // How long should the stimulation be run for in SECONDS

byte channel=0; //channel specifier
int currentCycle=0; 
int lastPacketTime=0;

/* For float <-> byte conversion */
typedef union {
  float f;
  char b[4];
} 
u;

float bytes2Float(char*data) {
  u _u;
  _u.b[3] = data[3];
  _u.b[2] = data[2];
  _u.b[1] = data[1];
  _u.b[0] = data[0];
  return _u.f;
}

void writeFloat(float value, char* data) {  
  u _u;
  _u.f = value;
  memcpy(data, _u.b, 4);
}

/* PIN DECLARATION */
const int LED_PIN = 3;
const int REF_PIN = 2;
const int ELECTRODE_PIN_1 = 8;

const int READ_1_PIN = 4;
const int READ_2_PIN = 5;

/* SPI pins  to control the digital potentiometer */
#if defined (__AVR_ATmega2560__) || defined (__AVR_ATmega1280__)
const int CS_PIN = 53;
const int SDI_PIN = 51;
const int CLK_PIN = 52;
#else
const int CS_PIN = 10;
const int SDI_PIN = 11;
const int CLK_PIN = 13;
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

void setup(void)
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = 31199 ; // set compare match register to desired timer count: 2ms
  TCCR1B |= (1 << WGM12); // turn on CTC mode:   
  TCCR1B |= (1 << CS10); // runes at clock frequency (multiplier = 1)
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt: 
  sei(); // enable global interrupts: 

  randomSeed(analogRead(0));
  lastTransmit=millis();

  // 2 ends of the electrode circuit
  pinMode(REF_PIN,OUTPUT);
  pinMode(ELECTRODE_PIN_1,OUTPUT);
  digitalWrite(REF_PIN,LOW);
  digitalWrite(ELECTRODE_PIN_1,LOW);

  // The LED output
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

  // Begin SPI communication with digital potentiometer
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  setTap(0); //Sets the pot to 0 - this means max resistance between A and W according to http://www.analog.com/static/imported-files/data_sheets/AD5204_5206.pdf 
  operationMode=NORMAL;

  // Read scale factor from EEPROM
  // scale_factor = eepromReadFloat(ADDR_SCALE_FACTOR);
  if (scale_factor < 0.2 || scale_factor > 2) {
    scale_factor = 1;
  }

  /*
  //initialize the playback array
   for (int i=0; i<100;i++) {
   playbackPower[i]=0;
   }
   for (int i=0; i< 50; i++) {
   wdt_reset(); //feed the dog
   delay(100);
   }
   
   if (EEPROM.read(ADDR_CALIBRATE) == 123) {
   calibFlag=false;
   }
   */
  Serial.begin(BAUD_RATE);

  wdt_enable(WDTO_8S); // Watchdog if more than 8s of inactivity
}

// Run every 2 ms (very fast)
ISR(TIMER1_COMPA_vect)
{
  if (overrideTapOn) { 
    tapValue = tapOverride;
    polarity = true;
    pinMode(REF_PIN,OUTPUT);
    pinMode(ELECTRODE_PIN_1,OUTPUT);
    digitalWrite(REF_PIN,HIGH);
    digitalWrite(ELECTRODE_PIN_1,LOW);
    digitalWrite(LED_PIN,HIGH);
  }

  readCurrent();

  if (operationMode == NORMAL) {
    setTap(int(tapValue));

    //overwrite the target value with the data from the playback sequence, if neccesary
    if (playback) {
      proportionalFactor=1; //optimize the control loop for fast response
      handlePlaybackSequence(); //keep the output updated
      //targetCurrent=abs(playbackPower[playbackPosition]); //set the power appropriately
      proportionalFactor=0.3; //optimize the control loop for accuracy
      if (playbackPower[playbackPosition] > 0) { //set the polarity of the signal
        polarity=true;
      }
      else {
        polarity=false;
      }
    }

    if (overrideTapOn) return;

    outputCurrent=targetCurrent;

    //if the device should be running in sham mode, ramp down the current until it is 0
    if (shamMode) processShamMode();

    int ledStatus = HIGH;
    if (abs(targetCurrent) == 0 && !shamMode) 
      ledStatus = LOW;
    digitalWrite(LED_PIN, ledStatus);

    if (abs(outputCurrent) > 0) { 
      setPowerDistribution(); // Figure out how to set the pins to get correct polarity

      if (connectionQualityIndex <= MAX_RESISTANCE || outputCurrent < 0.5) {
        float dtap = 1;
        if (abs(current) > 0.000001) {
          dtap = 5 * abs(current - outputCurrent) / (current * outputCurrent) / (1000000 / 255) * proportionalFactor;
        }
        if (abs(current) > abs(outputCurrent)) {
          tapValue -= dtap;
        }
        else if (abs(current) < abs(outputCurrent)) {
          tapValue += dtap;
        }
        // Compute tapValue accordingly
        /*
        if (abs(current) > abs(outputCurrent)) {
          tapValue = tapValue - (abs(current) - abs(outputCurrent)) * proportionalFactor;
        }
        else if (abs(current) < abs(outputCurrent)) {
          tapValue=tapValue + (abs(outputCurrent) - abs(current)) * proportionalFactor;
        }
        */
        if (tapValue > 255)
        {
          tapValue = 255;
        }
        if (tapValue < 0) {
          tapValue=0;
        }
      }
    }
    else {  //turn the power source off completely if 0 current is commanded.
      powerOffAll();
      tapValue=0;
    }
  }
}

void processShamMode() {
  if (shamRampdown <= originalTarget) {
    if (targetCurrent >= 0) {
      outputCurrent = targetCurrent - shamRampdown;
      if (outputCurrent < 0) {
        outputCurrent = 0;
      }
    }
    else {
      outputCurrent = targetCurrent + shamRampdown;
      if (outputCurrent > 0) {
        outputCurrent = 0;
      }
    }
  }
  if (abs(outputCurrent) < 0.3) {
    outputCurrent=0;
  }
  fastCycleCounter++;
  if (fastCycleCounter > INTERVAL_IN_MS / (FAST_CYCLE_PERIOD_USEC / 1000) ) { //only reduce the power if 250 milliseconds have elapsed since the last time. 125 cycles=250 ms.
    fastCycleCounter=0;      
    shamRampdown = shamRampdown + SHAM_RAMPDOWN_RATE;
  }
}

void readCurrent() {  
  a4out = analogRead(READ_1_PIN); // between 0 and 1023
  a5out = analogRead(READ_2_PIN);

  raw4 = (5.0 / 1024.0 * a4out); // actual voltage in V
  raw5 = (5.0 / 1024.0 * a5out);

  if (!polarity) {
    voltage = raw5 - raw4;
  }
  else {
    voltage = raw4 - raw5;
  }
  current = voltage /* * scale_factor */ / (float) RESISTANCE; //figure out the current and do the correction factor
  connectionQualityIndex = tapValue / current   + 0.00000001; //figure out the connection quality while preventing divide-by-zero errors

  //do averaging of current
  if (!playback) {//don't do averaging if we're playing back a custom waveform, it makes it hard to see what's going on 
    displayCurrent = displayCurrent + current;
    displaySamples ++;
  }
  else {
    // Playback
    displayCurrent = current;
    displaySamples = 1;
  }
}

void setPowerDistribution() { //set the channels so that we are only sending power through the currently active one
  pinMode(REF_PIN,OUTPUT);
  pinMode(ELECTRODE_PIN_1,OUTPUT);
  if (polarity) {
    // first, set the polarity of the reference electrode
    // current flows from 2 (anode) to 8 (cathode)
    digitalWrite(REF_PIN,HIGH);
    digitalWrite(ELECTRODE_PIN_1,LOW);
  }
  else {
    // current flows from 8 (cathode) to 2 (anode)
    digitalWrite(ELECTRODE_PIN_1,HIGH);
    digitalWrite(REF_PIN,LOW);
  }
}

void powerOffAll() { //kill power, used when the system is in resting mode
  digitalWrite(ELECTRODE_PIN_1,LOW);
  digitalWrite(REF_PIN,LOW);

  for (int i=2; i< 9; i++) { //shut everyething down
    pinMode(i,OUTPUT);
    digitalWrite(i,LOW);
  }
}

// Set the potentiometer value (only one - channel 5 - is in use)
int setTap( int value) {
  // take the SS pin low to select the chip:
  digitalWrite(CS_PIN,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(5);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(CS_PIN,HIGH); 
}

/* Main loop */
void loop(void)
{
  wdt_reset(); //feed the dog    
  if (operationMode == EMERGENCY_SHUTDOWN) { //emergency shutdown mode 
    powerOffAll();
    // Blink fast
    pinMode(LED_PIN,OUTPUT);
    delay(100);
    digitalWrite(LED_PIN,HIGH);
    delay(100);
    digitalWrite(LED_PIN,LOW);

    return; // Just to make sure that nothig else gets executed...
  }

  //normal operation
  unsigned long timestamp = millis();
  processSerialMessage(); // If any

  /*
    if (gotSerial==false && timestamp >= COMPUTER_TIMEOUT && EEPROM.read(ADDR_PROTOCOL_OK) == 123 ) { //the EEPROM stored protocol can be executed
   if (readEeprom) {
   readEeprom = false;
   safetyLimit = max(0, min( eepromReadFloat(ADDR_SAFETY_LIMIT) , ABSOLUTE_MAX_LIMIT_CURRENT));
   maxPower = eepromReadFloat(ADDR_MAX_POWER);
   maxRamp = eepromReadFloat(ADDR_MAX_RAMP);
   powerDelta = eepromReadFloat(ADDR_POWER_DELTA);
   runningTimeSec = eepromReadFloat(ADDR_RUNNING_TIME);
   }    
   if ((timestamp - COMPUTER_TIMEOUT) > (runningTimeSec * 1000)) {
   Serial.println("Ramping down");
   float oldIncrement = eepromReadFloat(ADDR_POWER_DELTA);
   if (oldIncrement > 0) { 
   powerDelta = 0 - oldIncrement;
   }
   else {
   powerDelta = abs(oldIncrement);
   }
   }
   }
   */

  if (overrideCurrent != 0) { //handle override stuff
    targetCurrent = overrideCurrent;
  }    

  if (timestamp >= lastTransmit + INTERVAL_IN_MS) { //250 ms have elpased since we last updated the computer, so send new data
    //handle on chip ramping
    digitalWrite(LED_PIN, ledStatus ? HIGH : LOW);
    ledStatus = !ledStatus;

    targetCurrent = targetCurrent + (powerDelta * INTERVAL_IN_MS / 1000);

    if (abs(targetCurrent) > abs(maxPower)) {
      targetCurrent = maxPower;
    }

    //handle zero-crossing problems
    if ((maxPower > 0 && targetCurrent < 0) || (maxPower < 0 && targetCurrent > 0)) {
      targetCurrent = 0;
    }

    //the control loop should have equilibrated by now, so store whatever tap value we're at
    currentCycle++;
    //targetCurrent = targets[activeChannel];
    int displayTap = tapValue;
    lastTransmit = timestamp;

    if (true || timestamp + 200 > lastPacketTime) { //make sure that we are clear to send
      if (calibFlag) {
        //Serial.print("CALIBRATEv");
      }

      sendParams(1, STATE_UPDATE);

      if (DEBUG)
        sendInternalState();
    }
    displayCurrent = displaySamples = 0;

    //safety check
    if (abs(getAvgCurrent()) > abs(safetyLimit)) {//uh oh
      shutdownCounter++;
    }
    else {
      shutdownCounter=0;
    }

    if (shutdownCounter > (MAX_OVERCURRENT_ALERT_TIMEOUT / INTERVAL_IN_MS) ){ //the system has been in overcurrent mode for at least MAX_OVERCURRENT_ALERT_TIMEOUT
      operationMode=EMERGENCY_SHUTDOWN; //go to emergency shutdown state
    }
  }
}

void sendFloatParamValue(const byte p, float value, byte& chk) {
  Serial.write(p);
  chk ^= p;
  char data[4];
  writeFloat(value, data);
  Serial.write(data, 4);
  for (int i = 0; i<4; ++i)
    chk ^= data[i];
}

void sendParams(int count, const Parameter* params) {
  byte chk = 0;
  Serial.write(0);
  chk ^= 0;
  byte length = 5*count + 1;
  Serial.write(length);
  chk ^= length;

  for (int i = 0; i<count; ++i)
    sendFloatParamValue(params[i], getValue(params[i]), chk);
  Serial.write(chk);
}

void sendInternalState() {
  sendParams(7, INTERNAL_STATE);
}

float getValue(Parameter p) {
  switch (p) {
  case tap_value:
    return tapValue;
  case voltage_value:
    return voltage;
  case target_current:
    return targetCurrent;
  case actual_current:
    return polarity ? getAvgCurrent() : -getAvgCurrent();
  case safety_limit:
    return safetyLimit;
  case polarity_value:
    if (polarity) return 1;
    return -1;
  default:
    return NAN;
  }
}
void processSerialMessage() {
  if (Serial.available() > 2) {
    byte chk = 0;
    byte header = Serial.read(); // header
    chk ^= header;
    byte length = Serial.read(); 
    chk ^= length;
    char buffer[30];
    int readB = Serial.readBytes(buffer, length);
    for (int i = 0; i<length; i++)
      chk ^= buffer[i];
    byte expected = buffer[length-1];
    byte computed = chk;

    if (chk != 0) {
      // Bad packet

      // Empty incoming buffer
      while (Serial.available())
        Serial.read();

      // Send bad a bad packet notification
      chk = 0;
      Serial.write(0);
      chk ^= 0;
      Serial.write(length + 7);
      chk ^= length + 6;
      Serial.write(bad_packet);
      chk ^= bad_packet;
      Serial.write(readB);
      chk ^= readB;
      Serial.write(header);
      chk ^= header;
      Serial.write(length);
      chk ^= length;
      for (int i = 0; i<length; i++) {
        chk ^= buffer[i];
        Serial.write(buffer[i]);
      }
      Serial.write(expected);
      chk ^= expected;
      Serial.write(computed);
      chk ^= computed;
      Serial.write(chk);
    } 
    else {
      handleCommand(buffer, length);
    }			
  }
}

void handleCommand(char* input, int bytecount) {
  char command = (byte) input[0];
  char* data = input + 1;
  switch (command) {
  case target_current:
    {
      //set to normal operation
      float signedTarget = bytes2Float(data);
      targetCurrent = abs(signedTarget);
      polarity = signedTarget >= 0;
      operationMode = NORMAL;
      powerDelta=0;
      overrideTapOn=false;
      break;
    }
  case safety_limit:
    { //set  the safety limit
      safetyLimit = max(0, min(bytes2Float(data), ABSOLUTE_MAX_LIMIT_CURRENT));
      overrideTapOn = false;
      break;  
    } 
  case scaling: 
    { //flash a new scaling value to the EEPROM 
      //eepromWriteFloat(ADDR_SCALE_FACTOR, bytes2Float(data));
      scale_factor=bytes2Float(data);
      powerDelta=0;
      overrideTapOn = false;
      break;
    }
  case max_power_for_channel:
    { //set the maximum power for this channel
      maxPower = bytes2Float(data);
      overrideTapOn = false;
      break;  
    }
  case current_override:
    { //override everything and force the system to output this current on channel 5 (used for diagnostics)
      delay(500);
      overrideCurrent = bytes2Float(data);
      overrideTapOn = false;
      break;  
    }
  case playback_speed: 
    {
      overrideTapOn = false;
      //set the custom waveform playback speed
      playbackSpeed=bytes2Float(data);
      targetCurrent=0;
      powerDelta=0;
      break;
    }
  case ramping_speed:
    { //set the speed for on-chip current ramping
      overrideTapOn = false;
      powerDelta=bytes2Float(data);
      break;  
    }
  case ramping_limit:
    { //set the upper limit for on-chip ramping
      overrideTapOn = false;
      maxRamp=bytes2Float(data);
      break;  
    }
  case stop:
    { //set to emergency off
      operationMode = EMERGENCY_SHUTDOWN;
      targetCurrent = 0;
      powerDelta = 0;
      overrideTapOn=false;
      break;
    }
  case start:
    { //set to emergency off
      operationMode = NORMAL;
      powerDelta=0;
      overrideTapOn=false;
      break;
    }
    /*
		case OverrideTap: 
     {
     tapOverride = data[0];
     overrideTapOn = true;
     break;
     }
     case SetSham: 
     { //toggle the sham on and start simulated sham rampdown
     powerDelta=0;
     overrideTapOn = false;
     
     if (!shamMode) {
     originalTarget = targetCurrent;
     shamMode=true;
     }
     else  {
     shamMode=false;
     targetCurrent=0;
     }
     break;
     }
     case StartPlayback: 
     { //start custom waveform playback
     overrideTapOn = false;
     playback=true;
     playbackPosition=0;
     powerDelta=0;
     break;  
     }
     case StopPlayback:
     { //stop custom playback
     overrideTapOn = false;
     playback=false;
     powerDelta=0;
     break;  
     }
     case UploadPoint:
     { //upload a custom waveform point--NOPE!
     overrideTapOn = false;
     powerDelta=0;
     if (playbackPosition < 100) {  //make sure that there's space left in the playback array
     playbackPower[playbackPosition]=incomingData;
     targetCurrent=0;
     playbackPosition++;
     playbackEndPosition++;
     }
     break;  
     }
     case GetPlaybackPower:
     {
     overrideTapOn = false;
     for (int i=0; i<100; i++) {
     //Serial.print(playbackPower[i]);
     // Serial.println("");
     }
     break;  
     }
     }
     
     case SaveSimulationParams:
     { //save stimulation parameters to the EEPROM
     overrideTapOn = false;
     float totalStimTime = incomingData;
     conservativeWrite(ADDR_PROTOCOL_OK, 123); //Set EEPROM address 9 to tell the device that battery operation is configured
     eepromWriteFloat(ADDR_SAFETY_LIMIT, safetyLimit); //store the safety limit
     eepromWriteFloat(ADDR_MAX_POWER, maxPower); //store the target power
     eepromWriteFloat(ADDR_MAX_RAMP, maxRamp); //store the value to ramp to
     eepromWriteFloat(ADDR_POWER_DELTA, powerDelta); //store the value to ramp to
     eepromWriteFloat(ADDR_RUNNING_TIME, totalStimTime); //store the value to ramp to
     break;  
     }
     case DisableCalibration:
     { //disable the calibration flag temporarily(if value is zero), or permanenetly(if value is one)
     calibFlag=false;
     if (incomingData == 1) {
     conservativeWrite(ADDR_CALIBRATE,123);
     }
     break;  
     }	
     		*/
  default:
    break;
  }
}

float getAvgCurrent() {
  return displayCurrent / displaySamples;
}

void handlePlaybackSequence() {
  if (millis() > lastPlaybackItemTime + playbackSpeed) { //time to start playing back the next item in the array
    playbackPosition++;
    if (playbackPosition >= playbackEndPosition) {
      playbackPosition=0;
    }
    lastPlaybackItemTime = millis();
  }
}

/******************************/
/** EEPROM related functions **/
/******************************/

float eepromReadFloat(int address){
  union u_tag {
    byte b[4];
    float fval;
  } 
  u;   
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address+1);
  u.b[2] = EEPROM.read(address+2);
  u.b[3] = EEPROM.read(address+3);
  return u.fval;
}

void eepromWriteFloat(int address, float value){
  union u_tag {
    byte b[4];
    float fval;
  } 
  u;
  u.fval=value;

  conservativeWrite(address,u.b[0]);
  conservativeWrite(address+1,u.b[1]);
  conservativeWrite(address+2,u.b[2]);
  conservativeWrite(address+3,u.b[3]);
}

// @TODO: We should use some wear levelling technique to ensure maximum durability
void conservativeWrite(int address, byte value) { //reduce wear on the EEPROM by only writing to it if data needs to be updated
  if (EEPROM.read(address) != value) {
    EEPROM.write(address, value);
  }
}






