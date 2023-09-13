
#include <Encoder.h> // Encoder Library

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
int command = 0;

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


void setup() {

  // serial config
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
  // setup pins and fix encoders
  setpins();
  
}


void loop() {

  if (Serial.available() > 0) {
    // read the incoming byte:
  command = Serial.read();

  }

  // count encoder ticks
  long ct1 = encoder_lfront.read();
  long ct2 = encoder_rfront.read();
  long ct3 = encoder_lback.read();
  long ct4 = encoder_rback.read();

    if (command == '1'){ // "1" character

    Move_motor(80,RF_PWM,RF_BACK,RF_FORW);
    }

  else if (command == '2') {

    Move_motor(0,RF_PWM,RF_BACK,RF_FORW);

    }

  delay(100);
  Serial.println(ct2);


}
