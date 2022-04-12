#include <Arduino.h>
#include "ADS131M04.h"
#include <SPI.h>
#include "CRC16.h"
#include "CRC.h"

static uint8_t numberofsamples= 2000;//2000
static uint8_t readyword[5];
char message;
CRC16 crc;
ADS131M04 adc;
adcOutput res;
volatile uint8_t readflag =0;
uint8_t registerbytes[2];
uint8_t buff[12000]; //numberofsamples*6 12000
uint8_t smallsendbuff[64];//60+2 for packet counter and +2 for crc
uint16_t register_contents = 0;
uint32_t counter=0;
uint32_t bigcounter=0;
uint32_t o,p,x =0;


void hw_wdt_disable(){
    *((volatile uint32_t*) 0x60000900) &= ~(1); // Hardware WDT OFF
}

void ICACHE_RAM_ATTR ISR() {
    //Serial.print(counter);
    //delayClocks( 750 );
    readflag=1;

}

void setup() {
  pinMode(D8, OUTPUT);
  digitalWrite(D8, LOW);
  delay(100);
  digitalWrite(D8, HIGH);
  delay(100);
  pinMode(D2, INPUT); //dataready
  hw_wdt_disable();
  ESP.wdtDisable();
  registerbytes[1] = 0xFF;//0xFF
  registerbytes[0] = 0xFF;
  readyword[0]=0x72;
  readyword[1]=0x65;
  readyword[2]=0x61;
  readyword[3]=0x64;
  readyword[4]=0x79;
  Serial.begin(115200);

  adc.begin(14, 12, 13, 5);// cs, dataready
  adc.setOsr(1);
  //adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  //adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  attachInterrupt(digitalPinToInterrupt(D2), ISR, FALLING);
  }

void loop() {
  if (readflag){
    readflag=0;
    res = adc.readADC();
    buff[counter*6] = res.ch0x;
    buff[(counter*6) +1] = res.ch0x2;
    buff[(counter*6) +2] = res.ch0x3;
    buff[(counter*6) +3] = res.ch1x;
    buff[(counter*6) +4] = res.ch1x2;
    buff[(counter*6) +5] = res.ch1x3;
    counter+=1;
    if (counter==sizeof(buff)/6){
        detachInterrupt(digitalPinToInterrupt(D2));
      Serial.write(readyword,5);
      while (Serial.available() > 0){
        message = Serial.read();
        if(message==0x7A){
          break;
          }
        }
      for(o=0;o<sizeof(buff)/60;++o){//sizeof(buff)/60
        for (p=0;p<60;++p){
           smallsendbuff[p] = buff[(o*60) +p];
        }
        crc.reset();
        crc.setPolynome(0x8408);
        crc.add((uint8_t*)smallsendbuff, 62);
        smallsendbuff[60] = bigcounter & 255;
        smallsendbuff[61] = (bigcounter >> 8) & 255;
        smallsendbuff[62] = crc.getCRC() & 255;
        smallsendbuff[63] = (crc.getCRC() >> 8) & 255;//crc.getCRC()
        Serial.write(smallsendbuff,sizeof(smallsendbuff));
        bigcounter++;
      }
      counter= 0;
      bigcounter=0;
      for(x=0;x<4;++x) {
          register_contents = adc.readRegister(x);
          registerbytes[1] = register_contents & 255;//0xFF
          registerbytes[0] = (register_contents >> 8) & 255;
          Serial.write(registerbytes, sizeof(registerbytes));
      }

      attachInterrupt(digitalPinToInterrupt(D2), ISR, FALLING);
    }
  }
}