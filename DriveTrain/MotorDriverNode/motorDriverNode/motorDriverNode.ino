#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
double vector [3] = {0.0, 0.0, 0.0}; // Reference Vector: [Vx, Vy, Omega] 
double dynamics [3][3] = {{0.866, 0.5, -0.5},{0.0, -1.0, -0.5},{-0.866, 0.5, -0.5}}; // Linear Transform [w1,w2,w3] = Av
double speeds [3]={0,0,0};
double wheelR = 0.05; // cm estimate
double wheelF [3] = {0.0, 0.0, 0.0}; // Frequency
double wheelT [3]= {0.0, 0.0, 0.0}; // Period
double wheelDir [3] = {0.0, 0.0, 0.0}; // direction 
double time1; // to control speed
double time2; // to control speed
double stillWheel [3] = {0,0,0};

void messageCb(const geometry_msgs::Twist& toggle_msg) {
  vector[0] = toggle_msg.linear.x;
  vector[1] = toggle_msg.linear.y;
  vector[2] = toggle_msg.angular.z;
  
  for (int k = 0; k<3; k++) {
    speeds[k] = 0;
  } 
  
  for(int r = 0; r<3; r++){
    for(int c = 0; c<3; c++){
      speeds[r] = speeds[r] + dynamics[r][c]*vector[c];
    }
  }

  for(int i = 0; i<3; i++){
    // So input here is in
    wheelF[i] = speeds[i]/wheelR; //radians per second
    if(abs(wheelF[i])<0.05){
      stillWheel[i] = 1;          //Indicator showing that the ith wheel isn't moivng while in this configuration 
      wheelT[i] = 1;              //So that when finding the overall period for the system, this still wheel doesn't have an effect
    } else {
      wheelT[i] = round(1000*abs(1/wheelF[i]))*2*3.14/200; // Now in rounded milliseconds per step
    }
    wheelDir[i] = wheelF[i]>0;
  }
}

/* 
 * setup publisher and subscriber
 */
ros::NodeHandle_<ArduinoHardware,1,2,200,150> nh;
std_msgs::Int32 debug_msg;
ros::Subscriber<geometry_msgs::Twist> sub("velocity_cmd", &messageCb);
ros::Publisher chatter("debug", &debug_msg);

/* 
 * setup the node 
 */
void setup()
{
  pinMode(7,OUTPUT); // 1 step
  pinMode(6,OUTPUT); // 1 direction
  pinMode(5,OUTPUT); // 2 step
  pinMode(4,OUTPUT); // 2 direction
  pinMode(3,OUTPUT); // 3 step
  pinMode(2,OUTPUT); // 3 direction
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

/* 
 * run update code
 */
void loop()
{  
  debug_msg.data = wheelT[0]*wheelT[1]*wheelT[2];
  chatter.publish(&debug_msg);
  
  digitalWrite(6, boolean(wheelDir[0])); 
  digitalWrite(4, boolean(wheelDir[1]));
  digitalWrite(2, boolean(wheelDir[2]));
  
  for (int i = 0; i < wheelT[0]*wheelT[1]*wheelT[2]; i++) {
    time1 = micros(); // Get actuation start time
    if (stillWheel[0] == 0){
      if (i%(int(wheelT[0]*2)) < wheelT[0]){  //multiplied by two because the loop iterates at 0.5 milliseconds
        digitalWrite(7, HIGH);
      }else{
        digitalWrite(7, LOW);
      }
    }
    if (stillWheel[1] == 0){
      if (i%(int(wheelT[1]*2)) < wheelT[1]){
        digitalWrite(5, HIGH);
      }else{
        digitalWrite(5, LOW);
      }
    }
    if (stillWheel[2] == 0){
      if (i%(int(wheelT[2]*2)) < wheelT[2]){
        digitalWrite(3, HIGH);
      }else{
        digitalWrite(3, LOW);
      }
    }
    time2 = micros(); // Get actuation end time
    if (time1 > time2 ){
      time2 = time1; // Correct for modulus measurement error
    }
    delayMicroseconds(500 - (time2 - time1)); // 500 microseconds and the smallest resolution is 1 millisecond step. Also, adjust for the time lost in actuation
  }
  debug_msg.data=1;
  chatter.publish(&debug_msg);
  nh.spinOnce();
}

