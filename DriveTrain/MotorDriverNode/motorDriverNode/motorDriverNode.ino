#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
double vector [3] = {0.0, 0.0, 0.0}; // Reference Vector: [Vx, Vy, Omega] 
double dynamics [3][3] = {{0.866, 0.5, -0.5},{0.0, -1.0, -0.5},{-0.866, 0.5, -0.5}}; // Linear Transform [w1,w2,w3] = Av
double speeds [3]={0,0,0};
double wheelR = 0.05; // cm estimate
double wheelF [3] = {0.0, 0.0, 0.0}; // Frequency
double wheelT [3]= {0.0, 0.0, 0.0}; // Period
double wheelDir [3] = {0.0, 0.0, 0.0}; // direction 

void messageCb(const geometry_msgs::Twist& toggle_msg) {
  for(int r = 0; r<3; r++){
    for(int c = 0; c<3; c++){
      speeds[r] = speeds[r] + dynamics[r][c]*vector[c];
    }
  }

  for(int i = 0; i<3; i++){
    wheelF[i] = speeds[i]/wheelR; //radians per second
    if(abs(wheelF[i])<0.01){
      wheelF[i] = 0.05;
    }
    wheelT[i] = round(100*abs(1/wheelF[i]));
    wheelDir[i] = wheelF[i]>0;
  }
}

/* 
 * setup publisher and subscriber
 */
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("velocity_cmd", &messageCb);

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
}

/* 
 * run update code
 */
void loop()
{
  digitalWrite(6, boolean(wheelDir[0])); 
  digitalWrite(4, !boolean(wheelDir[1]));
  digitalWrite(2, boolean(wheelDir[2]));
  for (int i = 0; i < wheelT[0]*wheelT[1]*wheelT[2]; i++) {
      if (i%(int(wheelT[0])*2) < wheelT[0]){
        digitalWrite(7, HIGH);
      }else{
        digitalWrite(7, LOW);
      }
      
      if (i%(int(wheelT[1])*2) < wheelT[1]){
        digitalWrite(5, HIGH);
      }else{
        digitalWrite(5, LOW);
      }
      
      if (i%(int(wheelT[2])*2) < wheelT[2]){
        digitalWrite(3, HIGH);
      }else{
        digitalWrite(3, LOW);
      }
      delayMicroseconds(500);

  }
  nh.spinOnce();
}

