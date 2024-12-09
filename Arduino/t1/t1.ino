int m1_EL_Start_Stop=6;  //EL 
int m1_ZF_Direction=12;  // ZF 
int m1_VR_speed=5;     //VR 

int m2_EL_Start_Stop=9;  //EL 
int m2_ZF_Direction=13;  // ZF 
int m2_VR_speed=10;      //VR 

int speed=20; //0-255


void setup() {
pinMode(m1_EL_Start_Stop, OUTPUT);//stop/start - EL 
pinMode(m1_ZF_Direction, OUTPUT); //direction  - ZF
pinMode(m2_EL_Start_Stop, OUTPUT);//stop/start - EL 
pinMode(m2_ZF_Direction, OUTPUT); //direction  - ZF

}

void loop() {
  
//  b();
//  delay(2000);
//  s();
//  delay(5000);
//b();
// delay(5000);
  // delay(1000);
  // l();
  // delay(2000);
//  s();
//  delay(1000);
  // b();
  // delay(2000);
  // s();
  // delay(1000);
  // r();
  // delay(2000);
  // s();
  f();
 
  
}

void f(){
     digitalWrite(m1_EL_Start_Stop,HIGH);
      analogWrite(m1_VR_speed,speed);
      digitalWrite(m1_ZF_Direction,HIGH);
      
      digitalWrite(m2_EL_Start_Stop,HIGH);
      analogWrite(m2_VR_speed,speed);
      digitalWrite(m2_ZF_Direction,LOW);

//digitalWrite(m2_EL_Start_Stop,LOW);
//delay(1000);
//digitalWrite(m2_EL_Start_Stop,HIGH);
//     analogWrite(m2_VR_speed,HIGH);
//     digitalWrite(m2_ZF_Direction,LOW);

}

void b(){
      digitalWrite(m1_EL_Start_Stop,HIGH);
      analogWrite(m1_VR_speed,speed);
      digitalWrite(m1_ZF_Direction,LOW);
      
      digitalWrite(m2_EL_Start_Stop,HIGH);
      analogWrite(m2_VR_speed,speed);
      digitalWrite(m2_ZF_Direction,HIGH);

}
void l(){
       digitalWrite(m1_EL_Start_Stop,HIGH);
      analogWrite(m1_VR_speed,speed);
      digitalWrite(m1_ZF_Direction,HIGH);
      
      digitalWrite(m2_EL_Start_Stop,HIGH);
      analogWrite(m2_VR_speed,speed);
      digitalWrite(m2_ZF_Direction,HIGH);

}

void r(){
       digitalWrite(m1_EL_Start_Stop,HIGH);
      analogWrite(m1_VR_speed,speed);
      digitalWrite(m1_ZF_Direction,LOW);
      
      digitalWrite(m2_EL_Start_Stop,HIGH);
      analogWrite(m2_VR_speed,speed);
      digitalWrite(m2_ZF_Direction,LOW);

}
void s(){
  digitalWrite(m1_EL_Start_Stop,LOW);
  digitalWrite(m2_EL_Start_Stop,LOW);
}
