#define max_pwm 255
#define default_pwm 103

#define Pi 3.14159

volatile float left_count_pulses = 0.0;
volatile float right_count_pulses = 0.0;

//Defining left motors
int in1 = 8;
int in2 = 10;
int enA = 9;

//Defining right motors
int in3 = 12;
int in4 = 13;
int enB = 11;

//Wheel encoder
int left_wheel_encA = 3;
int left_wheel_encB = 5;
int right_wheel_encA = 2;
int right_wheel_encB = 4;
int enc_to_rev_conversion = 170;

void setup() {
  // put your setup code here, to run once:

  //Setting up ouput pins for left motors
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //Setting up ouput pins for right motors
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Setting up built-in LED output
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(left_wheel_encA,INPUT);
  pinMode(left_wheel_encB,INPUT);
  pinMode(right_wheel_encA,INPUT);
  pinMode(right_wheel_encB,INPUT);
  
  Serial.begin(115200);
  
  //Attaching Interrupts to get encoder values
  attachInterrupt(digitalPinToInterrupt(left_wheel_encA),left_encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(right_wheel_encA),right_encoder,RISING);
}


void loop() {
  //Converts encoder pulses to rpm
  float left_rpm = ((left_count_pulses/enc_to_rev_conversion)/100)*1000*60;
  float right_rpm = ((right_count_pulses/enc_to_rev_conversion)/100)*1000*60;
  //Converts rpm to rad/s
  float left_rad_s = (left_rpm*(2*Pi))/60;
  float right_rad_s = (right_rpm*(2*Pi))/60;
  //Converts angular velocity in rad/s to linear velocity in m/s
  float left_m_s = left_rad_s*(0.065/2);
  float right_m_s = right_rad_s*(0.065/2);

  Serial.print(" | Left (rpm): "); Serial.print(left_rpm);
  Serial.print(" | Right (rpm): "); Serial.print(right_rpm);
  Serial.print(" | Left (m/s): "); Serial.print(left_m_s);
  Serial.print(" | Right (m/s): "); Serial.print(right_m_s);
  Serial.println();
  left_count_pulses = 0.0;
  right_count_pulses = 0.0;
  //moveForward();
  delay(100);
}

void left_encoder(){
  if(digitalRead(left_wheel_encB) == HIGH){
    left_count_pulses--;
  }
  else{
    left_count_pulses++;
  }
}

void right_encoder(){
  if(digitalRead(right_wheel_encB) == HIGH){
    right_count_pulses++;
  }
  else{
    right_count_pulses--;
  }
}

void moveForward(){
  //Left motors rotate forwards
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  //Right motors rotate forwards
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  float left_encoder_speed = default_pwm;
  float right_encoder_speed = default_pwm;

  analogWrite(enA,left_encoder_speed);
  analogWrite(enB,right_encoder_speed);
  

}

void moveBackward(){
  //Left motors rotate backwards
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  //Right motors rotate backwards
  digitalWrite(in3,LOW);  
  digitalWrite(in4,HIGH);

  float left_encoder_speed = default_pwm;
  float right_encoder_speed = default_pwm;
  
  analogWrite(enA,left_encoder_speed);
  analogWrite(enB,right_encoder_speed);

}

void stop(){
  //Stops left motors
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  analogWrite(enA,default_pwm);

  //Stops right motors
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  analogWrite(enB,default_pwm);
}