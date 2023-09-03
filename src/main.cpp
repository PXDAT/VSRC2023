#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define SERVO 2
#define SERVO2 3
#define SERVO3 4
#define SERVO4 5
#define SERVO_FREQ_MIN 200
#define SERVO_FREQ_MAX 550
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180
#define ENA
#define ENB 
#define pressures false
#define rumble false
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;
bool mode_drive = false, collector_mode = false, servo1 = false, servo2 = false, mode_linh = false, mode_vinh = false;
int servo3 = 0, servo4 = 0, shooter_mode = 0;
void setup() 
{
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);

  pwm.setPWM(SERVO,0,0);

  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.print("Ket noi voi tay cam PS2)");

  int error = -1;
  for (int i = 0; i < 10; i++) 
  {
    delay(200);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }
  switch (error) 
  {
    case (0):
      Serial.println(" Ket noi tay cam PS2 thanh cong");
      break;
    case (1):
      Serial.println(" LOI) Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
      break;
    case (2):
      Serial.println(" LOI) khong gui duoc lenh");
      break;
    case (3):
      Serial.println(" LOI) Khong vao duoc Pressures mode ");
      break;
  }

}
// servo setup
void servo_clockwise(uint16_t Servo)
{
  pwm.setPWM(Servo, 0, 180);
}
void servo_anticlockwise(uint16_t Servo1)
{
  pwm.setPWM(Servo1, 90, 580);
}
void servo_clockwise180(uint16_t Servo3)
{
  pwm.setPWM(Servo3, 0, 512);
}
void servo_anticlockwise180(uint16_t Servo4)
{
  pwm.setPWM(Servo4, 0, 205);
}
void stop_servo(uint8_t Servo2)
{
  pwm.setPWM(Servo2, 0, 0);
}
// motor setup
void rightdc(uint16_t right, uint16_t right2)
{
  pwm.setPWM(10,0,right);
  pwm.setPWM(11,0,right2);
}
void leftdc(uint16_t left0, uint16_t left1)
{
  pwm.setPWM(12,0,left0);
  pwm.setPWM(13,0,left1);
}
// shooter and collector setup
void liftDC()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,3000);
}
void reverse_liftDC()
{
  pwm.setPWM(8,0,4000);
  pwm.setPWM(9,0,0);
}
void liftDC_stop()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,0);
}
void collector()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,3000);
}
void reverse_collector()
{
  pwm.setPWM(14,0,4000);
  pwm.setPWM(15,0,0);
}
void reverse_collector_stop()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,0);
}
void collector_stop()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,0);
}
// control setup
void ps2Control() 
{
  ps2x.read_gamepad(false, false);
  int joyleft = ps2x.Analog(PSS_LY);
  int joyright = ps2x.Analog(PSS_RY);
  joyright = map(joyright,0, 255, -4095, 4095);
  joyleft = map(joyleft, 0, 255, -4095, 4095);
  if(joyleft > -17)
  {
    leftdc(0,0+joyleft);
    Serial.println(joyleft);
  }
  else if(joyleft < -17)
  {
    leftdc(0-joyleft,0);
    Serial.println(joyleft);
  }
  else
  {
    leftdc(0,0);
  }
  if(joyright > -17)
  {
    rightdc(0,0+joyright);
    Serial.println(joyright);
  }
  else if(joyright < -17)
  {
    rightdc(0-joyright, 0);
    Serial.println(joyright);
  }
  else 
  {
    rightdc(0,0);
  }
  delay(50);
}
void loop() {ps2Control();}
