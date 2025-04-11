#include <Arduino.h>
#include <vector>

#include "kinematics.h"
#include "motor.h"
#include <encoder.h>
#include "pid.h"

#define LED_PIN 38

#define HEAD        0xFF
#define HOST_ID     0x00
#define DEVICE_ID   0x01
#define FUNC_MOTION 0x02

#define PWM_M1      15
#define DIR_M1      16
#define PWM_M2      4
#define DIR_M2      5

#define H1A         48
#define H1B         47
#define H2A         6
#define H2B         7

#define BASE DIFFERENTIAL_DRIVE  
#define MOTOR_MAX_RPM           67 
#define MAX_RPM_RATIO           0.85
#define WHEEL_DIAMETER          0.126            
#define LR_WHEELS_DISTANCE      0.235
#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12 
#define ENC_PULSE_PER_REV       6114

#define PWM_NEG_M1 -120
#define PWM_POS_M1  120
#define PWM_NEG_M2 -120
#define PWM_POS_M2  120

const int freq = 21000;
const int PWM_CH_MOTOR_LEFT = 0;
const int PWM_CH_MOTOR_RIGHT = 0;
const int resolution = 8;
int dutyCycle = 130;

#define PWM_MAX pow(2, resolution)-1
#define PWM_MIN -PWM_MAX
#define K_P 4.0
#define K_I 2.0
#define K_D 0

Motor motor1(PWM_M1, DIR_M1, 0, freq, resolution);
Motor motor2(PWM_M2, DIR_M2, 1, freq, resolution);

Encoder* Encoder::instance0_ ;
Encoder* Encoder::instance1_ ;
Encoder* Encoder::instance2_ ;
Encoder* Encoder::instance3_ ;

Encoder encoder1(0, H1A, H1B, ENC_PULSE_PER_REV);
Encoder encoder2(1, H2A, H2B, ENC_PULSE_PER_REV);

Kinematics kinematics(
    Kinematics::BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

Kinematics::velocities cmd_vel;

PID pid_motor1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID pid_motor2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

uint8_t header;
uint8_t device_id;
uint8_t len;
uint8_t func;
uint8_t data_len;
uint8_t value;
uint8_t rx_check_num;
uint8_t check_sum;  
std::vector<uint8_t> data;

unsigned long prev_cmd_time = 0;

void setup() {

  encoder1.begin();
  encoder2.begin();

  pinMode(LED_PIN, OUTPUT); // Set LED_PIN as output

  Serial0.begin(115200); // Start Serial0 communication

  pid_motor1.reset();
  pid_motor2.reset();

  pid_motor1.set_deadband_comp( PWM_POS_M1, PWM_NEG_M1 );
  pid_motor2.set_deadband_comp( PWM_POS_M2, PWM_NEG_M2 );

  Serial0.flush();

  delay(100);
}

void loop() {

  if (Serial0.available() > 0) {
    header = Serial0.read();

    if (header == HEAD) {
    
        device_id = Serial0.read();

        if (device_id == DEVICE_ID) {

            len = Serial0.read();
            func = Serial0.read();

            check_sum = header + device_id + len + func;
            data_len = len - 4;
            data = {}; 

            while (data.size() < data_len) {  

                value = Serial0.read();
                data.push_back(value);
                check_sum += value;

                if (data.size() == data_len) {
                    rx_check_num = Serial0.read();                

                    if ((check_sum & 0xFF) == rx_check_num){
                  
                        delay(1);

                        if (func == FUNC_MOTION){

                          int16_t packed_v_x = (data[1] << 8) | data[0];
                          int16_t packed_v_y = (data[3] << 8) | data[2];
                          int16_t packed_w_z = (data[5] << 8) | data[4];
                  
                          cmd_vel.linear_x  = packed_v_x/1000.0;
                          cmd_vel.linear_y  = packed_v_y/1000.0;
                          cmd_vel.angular_z = packed_w_z/1000.0;
                  
                          prev_cmd_time = millis();  
                        
                        }
                    }
                    else {
                        return;
                    } 
                }                         
            }
        }
    }
  }

  if (millis() - prev_cmd_time > 100){
    cmd_vel.linear_x  = 0.0;
    cmd_vel.linear_y  = 0.0;
    cmd_vel.angular_z = 0.0;
  }

  Kinematics::rpm req_rpm = kinematics.getRPM(
  cmd_vel.linear_x, 
  cmd_vel.linear_y, 
  cmd_vel.angular_z);    

  float rpm_motor1 = encoder1.getRPM();
  float rpm_motor2 = encoder2.getRPM();

  motor1.drive((int)pid_motor1.compute(req_rpm.motor1, rpm_motor1));
  motor2.drive((int)pid_motor2.compute(req_rpm.motor2, rpm_motor2));

  delay(20);
}