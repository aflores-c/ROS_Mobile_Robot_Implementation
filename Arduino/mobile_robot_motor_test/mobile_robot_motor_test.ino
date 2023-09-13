#define USE_USBCON
#include <ros.h> //Include ROS Library
#include <std_msgs/Int32.h> //Include message type to send encoder ticks
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h> // Include message to send velocity
#include <geometry_msgs/Twist.h> // Include message type to receive vel commands
#include <geometry_msgs/Vector3Stamped.h>

//Libraries for control and sensors
#include <PID_v1.h> // PID Library
#include <Encoder.h> // Encoder Library

#define LOOPTIME                      100     //Looptime in millisecond
unsigned long lastMilli = 0;
const double radius = 0.06;                   //Wheel radius, in m
const double wheelbase = 0.4;               //Wheelbase, in m
double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s
double speed_req_left = 0;
double speed_req_right = 0;
double count_enc1 = 0;
double count_enc2 = 0;
double count_enc3 = 0;
double count_enc4 = 0;


// Initialize PID paramaters
double Setpoint_lf, Input_lf, Output_lf;
double Setpoint_lb, Input_lb, Output_lb;
double Setpoint_rf, Input_rf, Output_rf;
double Setpoint_rb, Input_rb, Output_rb;
//double aggKp=0.705, aggKi= 0.6, aggKd=0;
double aggKp=2.0, aggKi= 1.0, aggKd=0.0;

PID myPID_lf(&Input_lf, &Output_lf, &Setpoint_lf, aggKp, aggKi, aggKd, DIRECT);
PID myPID_lb(&Input_lb, &Output_lb, &Setpoint_lb, aggKp, aggKi, aggKd, DIRECT);
PID myPID_rf(&Input_rf, &Output_rf, &Setpoint_rf, aggKp, aggKi, aggKd, DIRECT);
PID myPID_rb(&Input_rb, &Output_rb, &Setpoint_rb, aggKp, aggKi, aggKd, DIRECT);
float demandx = 0;
float demandz = 0;

// Initialize pin numbers
// Encoder pins
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
bool wtf;




// Initialize ROS parameters
ros::NodeHandle nh; //ROS node
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
//std_msgs::Int64MultiArray enc_ticks; //Variable to send encoder ticks
//ros::Publisher enc_ticks_pub("encoder_ticks", &enc_ticks); // Publisher to send enc ticks
//ros::Publisher vel_pub("velocity_wheels", &vel_wheels);
//std_msgs::Float64MultiArray vel_wheels;

/*
void onPid_cb(const std_msgs::Int16MultiArray& cmd_msg)
{
    int aggKp = cmd_msg.data[0];
    int aggKi = cmd_msg.data[1];
    int aggKd = cmd_msg.data[2];
    myPID_lf.SetTunings(aggKp, aggKi, aggKd);
    myPID_lb.SetTunings(aggKp, aggKi, aggKd);
    myPID_rf.SetTunings(aggKp, aggKi, aggKd);
    myPID_rb.SetTunings(aggKp, aggKi, aggKd);
}*/


// Cmd_vel Callback
// Sets the setpoints of the pid for each wheel
//void onTwist(const std_msgs::Float32MultiArray& msg)
//{
//
//  float left_speed = msg.data[0];
//  float right_speed = msg.data[1];
//  Setpoint_lf = left_speed;
//  Setpoint_rf = right_speed;
//  Setpoint_lb = left_speed;
//  Setpoint_rb = right_speed;
//
//}

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  //noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  
  Setpoint_lf = (speed_req_left/radius)*(180/PI);
  Setpoint_rf = (speed_req_right/radius)*(180/PI);
  Setpoint_lb = (speed_req_left/radius)*(180/PI);
  Setpoint_rb = (speed_req_right/radius)*(180/PI);

}

//Callback for velocities--just for testing
//void cmd_vel_cb(const geometry_msgs::Twist& twist){
//
//  demandx = twist.linear.x;
//  demandz = twist.linear.z;
//  //demandx = demandx*1000;
////  Setpoint_lf = demandx*1000;
//
//}
//

//ROS Suscribers
//ros::Subscriber<std_msgs::Float32MultiArray> cmd_sub("set_vel", &onTwist);
//ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
//ros::Subscriber<std_msgs::Int16MultiArray> pid_sub("pid_set", &onPid_cb);
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)



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

//void reset Integral error when we stop
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

void setup() {


  //Debugging
  //Serial.begin(115200);

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
//  char dim0_label[] = "encoder_ticks";
//  enc_ticks.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
//  enc_ticks.layout.dim[0].label = dim0_label;
//  enc_ticks.layout.dim[0].size = 4;
//  enc_ticks.layout.dim[0].stride = 1*4;
//  enc_ticks.data = (long long int *)malloc(sizeof(long long int)*4);
//  enc_ticks.layout.dim_length = 0;
//  enc_ticks.data_length = 4;

  
  //ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  //nh.advertise(enc_ticks_pub);
  //nh.advertise(vel_pub);
  //nh.subscribe(cmd_sub);
//  nh.subscribe(pid_sub);
//  nh.subscribe(sub);
}

// Initialize starting loop paramaters for calculating velocity and time

unsigned long prev = 0;
long old_ct1=0;
long old_ct2=0;
long old_ct3=0;
long old_ct4=0; 

//float ticks_per_meter = 16700.0;
float ticks_per_revolution = 7200.0;

double angle_lf = 0.0;
double angle_lb = 0.0;
double angle_rf = 0.0;
double angle_rb = 0.0;

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

  
//  enc_ticks.data[0]=ct1;
//  enc_ticks.data[1]=ct2;
//  enc_ticks.data[2]=ct3;
//  enc_ticks.data[3]=ct4;
  

  // Publish encoder ticks to calculate odom on Jetson Nano side
//  enc_ticks_pub.publish(&enc_ticks);
  count_enc1 = ct1 - old_ct1;
  count_enc2 = ct2 - old_ct2;
  count_enc3 = ct3 - old_ct3;
  count_enc4 = ct4 - old_ct4;
  
  if (abs(count_enc1) < 20) count_enc1= 0;
  else 
  {
      Input_lf = 360*(float(count_enc1) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 
  if (abs(count_enc2) < 20) count_enc2= 0;
  else 
  {
      Input_rf = 360*(float(count_enc2) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 
  if (abs(count_enc3) < 20) count_enc3= 0;
  else 
  {
      Input_lb = 360*(float(count_enc3) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 
   if (abs(count_enc4) < 20) count_enc4= 0;
  else 
  {
      Input_rb = 360*(float(count_enc4) / ticks_per_revolution) / ((LOOPTIME) / 1000.0);
  } 

// PID Control
  myPID_lf.Compute();
  myPID_lb.Compute();
  myPID_rf.Compute();
  myPID_rb.Compute();

  Move_motor(Output_lf,LF_PWM,LF_BACK, LF_FORW);
  Move_motor(Output_lb,LB_PWM,LB_BACK,LB_FORW);
  Move_motor(Output_rf,RF_PWM,RF_BACK,RF_FORW);
  Move_motor(Output_rb,RB_PWM,RB_BACK,RB_FORW);


//  // take the old encoder ticks and time for calculating velocity
  old_ct1 = ct1;
  old_ct2 = ct2;
  old_ct3 = ct3;
  old_ct4 = ct4;

  //prev = now;
  
  //delay(25);
  publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
    
  }
  

}

void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = (((Input_lf + Input_lb)/2)/360)*2*PI*radius ;    //left wheel speed (in m/s)
  speed_msg.vector.y = (((Input_rf + Input_rb)/2)/360)*2*PI*radius ;;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
