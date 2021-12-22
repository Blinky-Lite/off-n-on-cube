#include "ModbusRtu.h"
#define BAUD_RATE  19200
#define MODBUSBUFSIZE  7
#define powerOnLedPin 4
#define powerOnPin    21
#define currentMonPin A0
#define commLEDPin    13

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(1,Serial1,0); // this is slave @1 and RS-232 or USB-FTDI

union ModbusUnion
{
  struct
  {
    uint16_t initCube;
    uint16_t avgAdc;
    uint16_t rmsAdc;
    uint16_t sampleRate;
    uint16_t nsamples;
    uint16_t nfilter;
    uint16_t powerOn;
  };
  uint16_t modbusBuffer[MODBUSBUFSIZE];
} mb;
boolean commLED = false;
uint16_t msgCnt = 0;

float avgAdc = 0.0;
float nsamplesCnt = 1.0;
float avgRms = 0.0;
float adcValue;
float rmsAdc;
float filteredAdc = 0.0;
float nfilterCnt = 1.0;
unsigned long tstart;
unsigned long tnow;
uint16_t sampleCount = 0;;

void setup()
{
  pinMode(powerOnLedPin, OUTPUT);
  pinMode(powerOnPin,    OUTPUT);
  pinMode(commLEDPin,    OUTPUT);
  pinMode(currentMonPin, INPUT);
 
  digitalWrite(powerOnLedPin, LOW);    
  digitalWrite(powerOnPin,    LOW);    
  digitalWrite(commLEDPin,    LOW);    

  mb.initCube               = 1;
  mb.avgAdc                 = 0;
  mb.rmsAdc                 = 0;
  mb.nsamples               = 5000;
  mb.nfilter                = 5;
  mb.sampleRate             = 0;
  mb.powerOn                = 0;

//  Serial.begin(9600);
  Serial1.begin(BAUD_RATE);
  slave.start();
  delay(1000);
//  for (int ii = 0; ii < 10; ++ii) Serial.println(mb.modbusBuffer[ii]);  
  tstart = micros();
  tnow = tstart;
   
}

void loop()
{
  float frms = 0.0;
  slave.poll( mb.modbusBuffer, MODBUSBUFSIZE );
  checkComm();

  adcValue = (float) analogRead(currentMonPin);
  filteredAdc = filteredAdc + (adcValue - filteredAdc) / nfilterCnt;
  avgAdc = avgAdc + (adcValue - avgAdc) / nsamplesCnt;
  rmsAdc = (filteredAdc - avgAdc);
  rmsAdc = rmsAdc * rmsAdc;
  avgRms = avgRms + (rmsAdc - avgRms) / nsamplesCnt;
  if (nsamplesCnt < ((float) mb.nsamples)) nsamplesCnt = nsamplesCnt + 1.0;
  if (nfilterCnt  < ((float) mb.nfilter)) nfilterCnt = nfilterCnt + 1.0;
 
  frms = 32.0 * sqrt(avgRms);
  mb.avgAdc = (uint16_t) (32.0 * avgAdc);
  mb.rmsAdc = (uint16_t) frms;

  if (mb.powerOn == 0)
  {
    digitalWrite(powerOnLedPin, LOW);    
    digitalWrite(powerOnPin,    LOW);    
  }
  else
  {
    digitalWrite(powerOnLedPin, HIGH);    
    digitalWrite(powerOnPin,    HIGH);    
  }
  sampleCount = sampleCount + 1;
  tnow = micros();
  if ((tnow - tstart) > 1000000)
  {
    mb.sampleRate = sampleCount;
    sampleCount = 0;
    tstart = tnow;
  }

}
void checkComm()
{
  uint16_t numMessages;
  numMessages = slave.getInCnt();
  if (numMessages != msgCnt)
  {
    msgCnt = numMessages;
    commLED = !commLED;
    digitalWrite(commLEDPin, commLED);    
  }
  
}
