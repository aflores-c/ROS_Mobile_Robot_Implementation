#define USE_USBCON

#include <ros.h> //Include ROS Library
#include <PID_v1.h> // PID Library
#include <Encoder.h> // Encoder Library

#include <std_msgs/Int16MultiArray.h> //to receive the set point values
#include <std_msgs/Int64MultiArray.h> //to publish vel_wheels
#include <std_msgs/Float32MultiArray.h> //to receive KP, KI, KD

#define LOOPTIME                      100     //Looptime in millisecond
unsigned long lastMilli = 0;
double count_enc1 = 0;
double count_enc2 = 0;
double count_enc3 = 0;
double count_enc4 = 0;
bool stop_robot;

// Initialize PID paramaters
double Setpoint_lf, Input_lf, Output_lf;
double Setpoint_lb, Input_lb, Output_lb;
double Setpoint_rf, Input_rf, Output_rf;
double Setpoint_rb, Input_rb, Output_rb;
double aggKp=0.0, aggKi= 0.0, aggKd=0.0;

PID myPID_lf(&Input_lf, &Output_lf, &Setpoint_lf, aggKp, aggKi, aggKd, DIRECT);
PID myPID_lb(&Input_lb, &Output_lb, &Setpoint_lb, aggKp, aggKi, aggKd, DIRECT);
PID myPID_rf(&Input_rf, &Output_rf, &Setpoint_rf, aggKp, aggKi, aggKd, DIRECT);
PID myPID_rb(&Input_rb, &Output_rb, &Setpoint_rb, aggKp, aggKi, aggKd, DIRECT);


//This pins come from the motor encoders
//Left front encoder
Encoder encoder_lfront(32, 33);
//Left back encoder
Encoder encoder_lback(34, 35);
//Right front encoder
Encoder encoder_rfront(37, 36);
//Right back encoder
Encoder encoder_rback(39, 38);

//PWM pins to control the speed of the motors
//This pins go to the H-bridge drivers
const uint8_t LF_PWM = 2; //H-Bridge one
const uint8_t LB_PWM = 3; //H-Bridge one
const uint8_t RF_PWM = 5; //H-Bridge two
const uint8_t RB_PWM = 4; //H-Bridge two

//Digital pins to move forward or backward
//This pins go to the H-bridge drivers
//Left front motor - H-Bridge one
const uint8_t LF_BACK = 22; 
const uint8_t LF_FORW = 23;
//Left back motor - H-Bridge one
const uint8_t LB_BACK = 24;
const uint8_t LB_FORW = 25;
//Right front motor - H-Bridge two
const uint8_t RF_BACK = 29;
const uint8_t RF_FORW = 28;
//Right back motor - H-Bridge two
const uint8_t RB_BACK = 27;
const uint8_t RB_FORW = 26;


ros::NodeHandle nh; //ROS node
std_msgs::Int64MultiArray vel_wheels;
ros::Publisher vel_pub("velocity_wheels", &vel_wheels);


void pid_callback(const std_msgs::Float32MultiArray& cmd_msg)
{
    int aggKp = cmd_msg.data[0];
    int aggKi = cmd_msg.data[1];
    int aggKd = cmd_msg.data[2];
    myPID_lf.SetTunings(aggKp, aggKi, aggKd);
    myPID_lb.SetTunings(aggKp, aggKi, aggKd);
    myPID_rf.SetTunings(aggKp, aggKi, aggKd);
    myPID_rb.SetTunings(aggKp, aggKi, aggKd);
}

// Sets the setpoints of the pid for each wheel
void set_point_callback(const std_msgs::Int16MultiArray& msg)
{

  float left_speed = msg.data[0];
  float right_speed = msg.data[1];
  Setpoint_lf = left_speed;
  Setpoint_rf = right_speed;
  Setpoint_lb = left_speed;
  Setpoint_rb = right_speed;
  
  if(left_speed==0 || right_speed==0){
    stop_robot=true;
  }
  else{
    stop_robot = false;
  }

}

//ROS Suscribers
ros::Subscriber<std_msgs::Int16MultiArray> sp_sub("set_vel", &set_point_callback);
ros::Subscriber<std_msgs::Float32MultiArray> pid_sub("pid_set", &pid_callback);


// Move any motor function with speed_pwm value and pin numbers
void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t back,const uint8_t forw)
{
  if(speed_pwm >= 0)
  {
    digitalWrite(back, HIGH);
    digitalWrite(forw, LOW);
    analogWrite(pwm, abs(speed_pwm));
  }
  else if(speed_pwm < 0)
  {
    digitalWrite(back, LOW);
    digitalWrite(forw, HIGH);
    analogWrite(pwm, abs(speed_pwm));
  }
}

void reset_pid_Ki()
{
  myPID_lf.SetMode(MANUAL);
  myPID_rf.SetMode(MANUAL);
  myPID_lb.SetMode(MANUAL);
  myPID_rb.SetMode(MANUAL);
  
  Output_lf=0;
  Output_rf=0;
  Output_lb=0;
  Output_rb=0;

  myPID_lf.SetMode(AUTOMATIC);
  myPID_rf.SetMode(AUTOMATIC);
  myPID_lb.SetMode(AUTOMATIC);
  myPID_rb.SetMode(AUTOMATIC);

}


// Initialize pins for forward movement

void setpins()
{
  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  pinMode(LB_PWM,OUTPUT);
  pinMode(RB_PWM,OUTPUT);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
}


void setup() {


  // Pid setup
  myPID_lf.SetOutputLimits(-255, 255);
  myPID_lb.SetOutputLimits(-255, 255);
  myPID_rf.SetOutputLimits(-255, 255);
  myPID_rb.SetOutputLimits(-255, 255);

  myPID_lf.SetMode(AUTOMATIC);
  myPID_lb.SetMode(AUTOMATIC);
  myPID_rf.SetMode(AUTOMATIC);
  myPID_rb.SetMode(AUTOMATIC);

  myPID_lf.SetSampleTime(20);
  myPID_lb.SetSampleTime(20);
  myPID_rf.SetSampleTime(20);
  myPID_rb.SetSampleTime(20);


  // setup pins and fix encoders
  setpins();


//  //encoder ticks array initialiazation
  char dim0_label[] = "vel_wheels";
  vel_wheels.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  vel_wheels.layout.dim[0].label = dim0_label;
  vel_wheels.layout.dim[0].size = 4;
  vel_wheels.layout.dim[0].stride = 1*4;
  vel_wheels.data = (long long int *)malloc(sizeof(long long int)*4);
  vel_wheels.layout.dim_length = 0;
  vel_wheels.data_length = 4;

  
  //ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(vel_pub);  //vel publisher
  nh.subscribe(pid_sub); //pid subscriber
  nh.subscribe(sp_sub); //setpoint subscriber
}

// Initialize starting loop paramaters for calculating velocity and time

unsigned long prev = 0;
long old_ct1=0;
long old_ct2=0;
long old_ct3=0;
long old_ct4=0; 

float ticks_per_revolution = 7200.0;

void loop() {

   nh.spinOnce();

   if((millis()-lastMilli) >= LOOPTIME)   
  { // enter timed loop
    lastMilli = millis();

  // count encoder ticks
  long ct1 = encoder_lfront.read();
  long ct2 = encoder_rfront.read();
  long ct3 = encoder_lback.read();
  long ct4 = encoder_rback.read();


// Publish encoder ticks to calculate odom on Jetson Nano side
  vel_wheels.data[0]=Input_lf;
  vel_wheels.data[1]=Input_rf;
  vel_wheels.data[2]=Input_lb;
  vel_wheels.data[3]=Input_rb;
  vel_pub.publish(&vel_wheels);

  count_enc1 = ct1 - old_ct1;
  count_enc2 = ct2 - old_ct2;
  count_enc3 = ct3 - old_ct3;
  count_enc4 = ct4 - old_ct4;
  
  if (abs(count_enc1) < 20) count_enc1= 0;
  else 
  {
      Input_lf = (360)*(float(count_enc1) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 
  if (abs(count_enc2) < 20) count_enc2= 0;
  else 
  {
      Input_rf = (360)*(float(count_enc2) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 
  if (abs(count_enc3) < 20) count_enc3= 0;
  else 
  {
      Input_lb = (360)*(float(count_enc3) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 
   if (abs(count_enc4) < 20) count_enc4= 0;
  else 
  {
      Input_rb = (360)*(float(count_enc4) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 

// PID Control
  myPID_lf.Compute();
  myPID_lb.Compute();
  myPID_rf.Compute();
  myPID_rb.Compute();

  //Move_motor(Output_lf,LF_PWM,LF_BACK, LF_FORW);
  //Move_motor(Output_lb,LB_PWM,LB_BACK,LB_FORW);
  Move_motor(Output_rf,RF_PWM,RF_BACK,RF_FORW);
  //Move_motor(Output_rb,RB_PWM,RB_BACK,RB_FORW);

  if(stop_robot) reset_pid_Ki();

  // take the old encoder ticks
  old_ct1 = ct1;
  old_ct2 = ct2;
  old_ct3 = ct3;
  old_ct4 = ct4;
  
    
  }
  

}
