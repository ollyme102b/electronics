#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

/* 
 * Stepper Motor Class
 */
 
class StepperMotor {
  
  private: 
  int directionPin;
  int stepPin;
  double stepPeriod;
  bool stepDir;
  bool isMoving;
  bool currentState;
  double prevStepTime;
  double wheelRadius;
  

  
  public:

  /* 
  * Creates a stepper motor that attaches to the stPin and dirPin.
  */
  StepperMotor(int stPin, int dirPin, double currTime) {
    directionPin = dirPin;
    stepPin = stPin;
    stepPeriod = 0;
    isMoving = false;
    currentState = false;
    prevStepTime = currTime;
    wheelRadius = 0.05;
  }
  
  /*
  * Takes desired linear velocity and sets the step period for this stepper motor.
  */
  void setStepPeriod(double linVelocity){
    double stepsPerSecond = 200 * linVelocity/(wheelRadius * 2 * 3.14159265);
    if (abs(stepsPerSecond) <= 2) {
      isMoving = false;
    } else {

      isMoving = true;
      stepPeriod = abs(1./stepsPerSecond);
      if (stepDir != (linVelocity > 0)){
        stepDir = linVelocity > 0;
        digitalWrite(directionPin, stepDir);
      }

    }
  }

  /*
  * Flips the bit on this actuator
  */
  void flip(){
    if (currentState) {
      currentState = false;
      digitalWrite(stepPin, LOW);
    } else {
      currentState = true;
      digitalWrite(stepPin, HIGH);
    }
  }

  /*
  * actuates this stepper motor
  */
  void actuate() {

    double currTime = micros() / 1e6;
    if (isMoving && (currTime - prevStepTime > stepPeriod)) {
      flip();
      prevStepTime = currTime;
    }
  }

  int getDirPin(){
    return directionPin;
  }

  int getStepPin(){
    return stepPin;
  }

  bool getIsMoving(){
    return isMoving;
  }
  
  double getStepPeriod(){
    return stepPeriod;
  }
  
};

/* 
* Declare transforms and stepper motors
*/
double referenceVelocities [3] = {0.0, 0.0, 0.0}; // [Vx, Vy, Omega] 
double dynamics [3][3] = {{0.866, 0.5, -0.5},{0.0, -1.0, -0.5},{-0.866, 0.5, -0.5}}; // [w1,w2,w3] = Av
double wheelVelocities [3]={0,0,0};
StepperMotor stepper0 = StepperMotor(7,6, micros()/1e6);
StepperMotor stepper1 = StepperMotor(5,4, micros()/1e6);
StepperMotor stepper2 = StepperMotor(3,2, micros()/1e6);


/*
* Callback function. Receives reference command messages and sets stepper motor speeds.
*/
void messageCb(const geometry_msgs::Twist& toggle_msg) {
  referenceVelocities[0] = toggle_msg.linear.x;
  referenceVelocities[1] = toggle_msg.linear.y;
  referenceVelocities[2] = toggle_msg.angular.z;
  
  for (int k = 0; k<3; k++) {
    wheelVelocities[k] = 0;
  } 
  
  for(int r = 0; r<3; r++){
    for(int c = 0; c<3; c++){
      wheelVelocities[r] = wheelVelocities[r] + dynamics[r][c]*referenceVelocities[c];
    }
  }

  stepper0.setStepPeriod(wheelVelocities[0]);
  stepper1.setStepPeriod(wheelVelocities[1]);
  stepper2.setStepPeriod(wheelVelocities[2]);

}

/* 
 * setup publisher and subscriber
 */
ros::NodeHandle_<ArduinoHardware,1,2,200,150> nh;
ros::Subscriber<geometry_msgs::Twist> sub("velocity_cmd", &messageCb);
// std_msgs::Float64 debug_msg;
// ros::Publisher chatter("debug", &debug_msg);

/* 
 * setup the node 
 */
void setup()
{
  pinMode(stepper0.getStepPin(),OUTPUT);
  pinMode(stepper0.getDirPin(),OUTPUT); 
  pinMode(stepper1.getStepPin(),OUTPUT);
  pinMode(stepper1.getDirPin(),OUTPUT); 
  pinMode(stepper2.getStepPin(),OUTPUT);
  pinMode(stepper2.getDirPin(),OUTPUT); 

  nh.initNode();
  nh.subscribe(sub);
}

/* 
 * run update code
 */
void loop()
{  
  stepper0.actuate();
  stepper1.actuate();
  stepper2.actuate();
  nh.spinOnce();
}

