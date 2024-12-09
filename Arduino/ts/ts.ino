const int el1=6;
const int z1 = 12;
const int vr1 = 5;

const int el2=9;
const int z2 = 13;
const int vr2 = 10;

static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  double pwmLeftReq = -1;
double pwmRightReq = +1;

const int PWM_MIN = 20; // about 0.1 m/s
const int PWM_MAX = 40;

void setup() {
  // put your setup code here, to run once:
  pinMode(el1, OUTPUT);
  pinMode(el2, OUTPUT);
  pinMode(vr1, OUTPUT);
  pinMode(vr2, OUTPUT);
  pinMode(z1, OUTPUT);
  pinMode(z2, OUTPUT);

}

void loop() {


  
  // put your main code here, to run repeatedly:
   if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(z1,HIGH);
    digitalWrite(el1,HIGH);
//    Serial.println("frl");
//    Serial.print(pwmRightOut);
//    Serial.println();
    
    
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse

    digitalWrite(z1,LOW);
    digitalWrite(el1,HIGH);
//    Serial.println("rvl");
//    Serial.print(pwmRightOut);
//    Serial.println();
    
    
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
  
    digitalWrite(el1,LOW);
  }
  else { // Left wheel stop
   
    digitalWrite(el1,LOW); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
     digitalWrite(z2,LOW);
    digitalWrite(el2,HIGH);
//    Serial.println("frr");
//    Serial.print(pwmLeftOut);
//    Serial.println();
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(z2,HIGH);
    digitalWrite(el2,HIGH);
//    Serial.println("rvr");
//    Serial.print(pwmLeftOut);
//    Serial.println();
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    
    digitalWrite(el2,LOW);
  }
  else { // Right wheel stop
    
    digitalWrite(el2,LOW); 
  }

   
if(abs(pwmLeftReq) > PWM_MAX)
pwmLeftOut=PWM_MAX;
else if (abs(pwmLeftReq) < PWM_MIN)
pwmLeftOut=PWM_MIN;
else
pwmLeftOut=abs(pwmLeftReq);

if(abs(pwmRightReq) > PWM_MAX)
pwmRightOut=PWM_MAX;
else if (abs(pwmRightReq) < PWM_MIN)
pwmRightOut=PWM_MIN;
else
pwmRightOut=abs(pwmRightReq);

  analogWrite(vr1, pwmLeftOut); 
  analogWrite(vr2, pwmRightOut); 
 
}
