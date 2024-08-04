#include "ESP32_PWM.h"
#include "esp32-hal-ledc.h"
#include "Arduino.h"
#include <driver/ledc.h> //defines LEDC_TIMER_BIT_MAX

PWM *PWM::channels[] = {0};

PWM::PWM(int pin, int freq, float min_us, float max_us)
{     
  this->req_freq = freq;
  this->pin = pin; 
  this->min_us = min_us;
  this->max_us = max_us;
  this->ch = -1;
  this->bits = 0;
  this->act_freq = 0;
  this->max_duty = 0;
  this->inv_duty_resolution_us = 1;

  //exit if no free channel
  int ch = findFreeChannel(freq);
  if(ch<0) return;

  //find maximum number of bits, or exit if less than 7
  int act_freq;
  int bits = LEDC_TIMER_BIT_MAX;
  while(1) {
    act_freq = ledcSetup(ch, freq, bits);
    if(act_freq > 0) break;
    bits--;
    if(bits < 7) return;
  }

  ledcWrite(ch, 0); //start with no output
  ledcAttachPin(pin, ch);      

  this->ch = ch;
  this->bits = bits;
  this->act_freq = act_freq;
  this->max_duty =  (1<<bits) - 1;
  this->inv_duty_resolution_us = 1.0e-6 * act_freq * (max_duty+1);
  channels[ch] = this;
}

void PWM::writeMicroseconds(float us) 
{
  if(us < min_us) us = min_us;
  if(us > max_us) us = max_us;
  int duty = us * inv_duty_resolution_us;
  if(duty < 0) duty = 0;
  if(duty > max_duty) duty = max_duty;
  ledcWrite(ch, us * inv_duty_resolution_us);
}

//two channels share the same timer - have also the same freq 
//try first to find a free channel with matching freq
//if not found, then find first free timer
int PWM::findFreeChannel(int freq)
{
  //find free channel with other channel in same group with same req_freq
  for(int i=0;i<PWM_MAX;i++){
    if(!channels[i]) {
      int other_i = (i % 2 == 0 ? i+1 : i-1);
      PWM *other_ch = channels[other_i];
      if(other_ch && other_ch->req_freq == freq) {
        return i;
      }
    }
  }
  //no free channel with matching freq found -> find first free group
  for(int i=0;i<PWM_MAX;i++){
    if(!channels[i]) {
      int other_i = (i % 2 == 0 ? i+1 : i-1);
      PWM *other_ch = channels[other_i];
      if(!other_ch) {
        return i;
      }
    }
  }      
  return -1;
} 