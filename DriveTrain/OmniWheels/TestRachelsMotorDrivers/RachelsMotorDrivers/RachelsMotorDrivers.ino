int x; 
int i = 1; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(6,OUTPUT); // enable
  pinMode(5,OUTPUT); // step
  pinMode(4,OUTPUT); // direction
  digitalWrite(6,LOW); // set to enable the motor
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(6,LOW); // Set enable low
  
  if(i == 1){
    digitalWrite(4,HIGH);// Set Dir High
    i=-1;
    Serial.println("forward");
  }else{
    digitalWrite(4,LOW);//
    i=1; 
    Serial.println("backward");
  }
  
  Serial.println("loop 200 steps (1 rev)");
  for(x =0; x<225; x++){
    digitalWrite(5,HIGH); 
    delay(3);
    digitalWrite(5,LOW); 
    delay(3);
  }
  Serial.println("Pause");
  delay(100);

}
