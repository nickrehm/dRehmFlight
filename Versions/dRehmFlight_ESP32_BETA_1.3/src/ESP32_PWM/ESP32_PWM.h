/*=============================================================================
Minimal Servo / PWM library for ESP32

Example
-------
#include "ESP32_PWM.h"

PWM motor(12,2000,125,250); //pin 12: Oneshot motor ESC 2000Hz pulse 125-250 us
PWM servo(22,50,1000,2000); //pin 22: regular servo 50Hz pulse 1000-2000 us

float motor_pwm = 125;
float servo_pwm = 1000;

void setup() {
 
}

void loop() {
  motor_pwm++;
  if(motor_pwm > motor.get_max_us()) motor_pwm = motor.get_min_us();
  motor.writeMicroseconds(motor_pwm);

  servo_pwm++;
  if(servo_pwm > servo.get_max_us()) servo_pwm = servo.get_min_us();
  servo.writeMicroseconds(servo_pwm);

  delay(3);
}

=============================================================================*/

//Maximum number of PWM outputs - NOTE: some ESP32 chips have less than this
#define PWM_MAX 16

class PWM
{
  public:
    PWM(int pin, int freq, float min_us, float max_us);
    void writeMicroseconds(float us);
    int get_ch() {return ch;}
    float get_min_us() {return min_us;}
    float get_max_us() {return max_us;}
    float get_duty_resolution_us() {return 1/inv_duty_resolution_us;}
    int get_req_freq() {return req_freq;}
    int get_act_freq() {return act_freq;}

  private:
    static PWM *channels[PWM_MAX];
    int pin;
    int ch;
    int bits;
    int max_duty;
    float min_us;
    float max_us;     
    int req_freq; //requested frequency
    int act_freq; //actual frequency
    float inv_duty_resolution_us;
    static int findFreeChannel(int freq);
};