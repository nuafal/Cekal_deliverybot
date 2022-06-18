//----------------------------------------------------------------------------------//
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <CytronMotorDriver.h>
#include "Config.h"
#include "Kinematics.h"
#include "Motor.h"

Motor LH_motor(LH_D1, LH_D2, LH_ENA, LH_ENB);
Motor RH_motor(RH_D1, RH_D2, RH_ENA, RH_ENB);
CytronMD motor1(PWM_DIR, 10, A0);  // PWM 1 = Pin 10, DIR 1 = Pin A0.
CytronMD motor2(PWM_DIR, 9, A1); // PWM 2 = Pin 9, DIR 2 = Pin A1.
Kinematics Robot(LH_motor, RH_motor);
void LH_ISRA();
void LH_ISRB();
void RH_ISRA();
void RH_ISRB();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void cmd_vel_callback(const geometry_msgs::Twist& twist);
std_msgs::Int16 lwheel_ticks_msg;
std_msgs::Int16 rwheel_ticks_msg;
std_msgs::Int16 lwheel_pwm_msg;
std_msgs::Int16 rwheel_pwm_msg;
ros::NodeHandle nh;
ros::Publisher lwheel_ticks_pub("left_ticks", &lwheel_ticks_msg);
ros::Publisher rwheel_ticks_pub("right_ticks", &rwheel_ticks_msg);
ros::Publisher lwheel_pwm_pub("left_pwm", &lwheel_pwm_msg);
ros::Publisher rwheel_pwm_pub("right_pwm", &rwheel_pwm_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);
unsigned long lastCmdVelReceived = 0;
float linearX_vel = 0, angularZ_vel = 0;
const float MIN_VELOCITY = 0.0;
const float MAX_VELOCITY = (2*PI*WHEEL_RADIUS*MAX_RPM)/(60*GEAR_REDUCTION);//0.728485253 m/s
//----------------------------------------------------------------------------------//

void setup(){
  pinMode(LH_ENA, INPUT_PULLUP);
  pinMode(LH_ENB, INPUT_PULLUP);
  pinMode(RH_ENA, INPUT_PULLUP);
  pinMode(RH_ENB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LH_ENA), LH_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_ENB), LH_ISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENA), RH_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENB), RH_ISRB, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ESTOP), EMG_STOP, HIGH);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(lwheel_ticks_pub);
  nh.advertise(rwheel_ticks_pub);
  nh.advertise(lwheel_pwm_pub);
  nh.advertise(rwheel_pwm_pub);
}

void loop(){
  nh.spinOnce();
  Robot.isRosConnected = nh.connected();

  //Stop the robot if not connected to ROS or there are no velocity command after some time
  if(!Robot.isRosConnected || (millis() - lastCmdVelReceived > CMD_VEL_TIMEOUT)){
    linearX_vel = 0;
    angularZ_vel = 0;
    Robot.Move(MIN_PWM, MIN_PWM);//stop motors
  }

  // Convert Linear X and Angular Z Velocity to PWM
  // Calculate left and right wheel velocity
  float  left_vel = linearX_vel - angularZ_vel*(WHEEL_SEPARATION/2);
  float right_vel = linearX_vel + angularZ_vel*(WHEEL_SEPARATION/2);

  // Determine left and right direction using sign
  int  left_dir = (left_vel >0)? 1 : -1;
  int right_dir = (right_vel>0)? 1 : -1;

  // Make sure calculated velocity is in range of min and max velocity before mapping to PWM
  if(fabs(left_vel)>MAX_VELOCITY){
    left_vel = left_dir*MAX_VELOCITY;
  }
  else if(fabs(left_vel)<MIN_VELOCITY){
    left_vel = left_dir*MIN_VELOCITY;
  }
  if(fabs(right_vel)>MAX_VELOCITY){
    right_vel = right_dir*MAX_VELOCITY;
  }
  else if(fabs(right_vel)<MIN_VELOCITY){
    right_vel = right_dir*MIN_VELOCITY;
  }

  // Map wheel velocity to PWM
  int  left_pwm = round(mapFloat(fabs(left_vel ), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));
  int right_pwm = round(mapFloat(fabs(right_vel), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));

  // Actuate the motors
  lwheel_pwm_msg.data = left_dir*left_pwm;
  rwheel_pwm_msg.data = right_dir*right_pwm;
  Robot.Move(left_dir*left_pwm, right_dir*right_pwm);


  //Publishing data to ROS
  lwheel_ticks_pub.publish(&lwheel_ticks_msg);
  rwheel_ticks_pub.publish(&rwheel_ticks_msg);
  lwheel_pwm_pub.publish(&lwheel_pwm_msg);
  rwheel_pwm_pub.publish(&rwheel_pwm_msg);
  delay(200); //5Hz
}

////////////FUNCTION DEFINITIONS////////////////

void LH_ISRA(){
  lwheel_ticks_msg.data = LH_motor.doEncoderA();
}

void LH_ISRB(){
  lwheel_ticks_msg.data = LH_motor.doEncoderB();
}

void RH_ISRA(){
  rwheel_ticks_msg.data = RH_motor.doEncoderA();
}

void RH_ISRB(){
  rwheel_ticks_msg.data = RH_motor.doEncoderB();
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cmd_vel_callback( const geometry_msgs::Twist& twist){
  linearX_vel = twist.linear.x;
  angularZ_vel = twist.angular.z;
  lastCmdVelReceived = millis();
}
