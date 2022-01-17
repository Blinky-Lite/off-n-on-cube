#include "BlinkyBus.h"
#define BAUD_RATE  19200
#define commLEDPin    5

#define powerOnLedPin 4
#define powerOnPin    21
#define currentMonPin A0

#define BLINKYBUSBUFSIZE  7
union BlinkyBusUnion
{
  struct
  {
    int16_t state;
    int16_t avgAdc;
    int16_t rmsAdc;
    int16_t sampleRate;
    int16_t nsamples;
    int16_t nfilter;
    int16_t powerOn;
  };
  int16_t buffer[BLINKYBUSBUFSIZE];
} bb;
BlinkyBus blinkyBus(bb.buffer, BLINKYBUSBUFSIZE, Serial1, commLEDPin);


float avgAdc = 0.0;
float nsamplesCnt = 1.0;
float avgRms = 0.0;
float adcValue;
float rmsAdc;
float filteredAdc = 0.0;
float nfilterCnt = 1.0;
unsigned long tstart;
unsigned long tnow;
int16_t sampleCount = 0;;

void setup()
{
  pinMode(powerOnLedPin, OUTPUT);
  pinMode(powerOnPin,    OUTPUT);
  pinMode(commLEDPin,    OUTPUT);
  pinMode(currentMonPin, INPUT);
 
  digitalWrite(powerOnLedPin, LOW);    
  digitalWrite(powerOnPin,    LOW);    

  bb.state                  = 1;
  bb.avgAdc                 = 0;
  bb.rmsAdc                 = 0;
  bb.nsamples               = 2000;
  bb.nfilter                = 5;
  bb.sampleRate             = 0;
  bb.powerOn                = 0;

  Serial1.begin(BAUD_RATE);
  blinkyBus.start();
  tstart = micros();
  tnow = tstart;
}

void loop()
{
  float frms = 0.0;
  int16_t oldPowerOn;

  oldPowerOn = bb.powerOn;
  blinkyBus.poll();
  if (oldPowerOn != bb.powerOn)
  {
    if (bb.powerOn == 0)
    {
      digitalWrite(powerOnLedPin, LOW);    
      digitalWrite(powerOnPin,    LOW);    
    }
    else
    {
      digitalWrite(powerOnLedPin, HIGH);    
      digitalWrite(powerOnPin,    HIGH);    
    }
    nsamplesCnt = 1.0;
    nfilterCnt  = 1.0;
    delay(500);
  }

  if (nsamplesCnt > ((float) bb.nsamples) ) nsamplesCnt = 1;
  if (nfilterCnt  > ((float) bb.nfilter) )  nfilterCnt = 1;

  adcValue = (float) analogRead(currentMonPin);
  filteredAdc = filteredAdc + (adcValue - filteredAdc) / nfilterCnt;
  avgAdc = avgAdc + (adcValue - avgAdc) / nsamplesCnt;
  rmsAdc = (filteredAdc - avgAdc);
  rmsAdc = rmsAdc * rmsAdc;
  avgRms = avgRms + (rmsAdc - avgRms) / nsamplesCnt;
  if (nsamplesCnt < ((float) bb.nsamples)) nsamplesCnt = nsamplesCnt + 1.0;
  if (nfilterCnt  < ((float) bb.nfilter)) nfilterCnt = nfilterCnt + 1.0;
 
  frms = 32.0 * sqrt(avgRms);
  bb.avgAdc = (int16_t) (32.0 * avgAdc);
  bb.rmsAdc = (int16_t) frms;

  sampleCount = sampleCount + 1;
  tnow = micros();
  if ((tnow - tstart) > 1000000)
  {
    bb.sampleRate = sampleCount;
    sampleCount = 0;
    tstart = tnow;
  }

}
