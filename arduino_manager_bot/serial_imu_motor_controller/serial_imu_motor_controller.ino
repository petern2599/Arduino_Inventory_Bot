#define max_pwm 255

#define INPUT_SIZE 2
#define BUFFER_SIZE 70

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdlib.h>

Adafruit_MPU6050 mpu;

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
const float enc_to_rev = 513.5;
const float rpm_to_radians = 0.10471975512;

char *token;
const char *delimiter =",";
char *values[INPUT_SIZE];

char buffer[BUFFER_SIZE];
float linear = 0.0;
float angular = 0.0;

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

  pinMode(left_wheel_encA,INPUT);
  pinMode(left_wheel_encB,INPUT);
  pinMode(right_wheel_encA,INPUT);
  pinMode(right_wheel_encB,INPUT);
  
  attachInterrupt(digitalPinToInterrupt(left_wheel_encA),left_encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(right_wheel_encA),right_encoder,RISING);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  delay(10);
}


void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Initialize char variables for IMU readings
  char accel_x[10], accel_y[10], accel_z[10], gyro_x[10], gyro_y[10], gyro_z[10], temperature[10];

  //Initialize char variables for wheel encoder readings
  char vel_l[10], vel_r[10];

  //Converts encoder pulses to rpm
  float left_rpm = left_count_pulses/enc_to_rev * (60/0.1);
  float right_rpm = right_count_pulses/enc_to_rev * (60/0.1);
  //Converts rpm to rad/s
  float left_rad_s = left_rpm*rpm_to_radians;
  float right_rad_s = right_rpm*rpm_to_radians;
  //Converts angular velocity in rad/s to linear velocity in m/s
  float left_m_s = left_rad_s*(0.065/2);
  float right_m_s = right_rad_s*(0.065/2);

  //Convert float values to string format in char variables
  dtostrf(a.acceleration.x,4,3,accel_x);
  dtostrf(a.acceleration.y,4,3,accel_y);
  dtostrf(a.acceleration.z,4,3,accel_z);
  dtostrf(g.gyro.x,4,3,gyro_x);
  dtostrf(g.gyro.y,4,3,gyro_y);
  dtostrf(g.gyro.z,4,3,gyro_z);
  dtostrf(temp.temperature,4,3,temperature);
  dtostrf(left_m_s,4,3,vel_l);
  dtostrf(right_m_s,4,3,vel_r);

  //Append char values into buffer to send through serial
  sprintf(buffer,"%s,%s,%s,%s,%s,%s,%s,%s,%s",
                  accel_x,accel_y,accel_z,
                  gyro_x,gyro_y,gyro_z,
                  temperature,vel_l,vel_r);
  Serial.println(buffer);

  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    String message = Serial.readStringUntil('\n');
    char message_array[message.length()+1];
    message.toCharArray(message_array,message.length()+1);

    int idx = 0;
    token = strtok(message_array, delimiter);
    while (token != NULL) {
      if( idx < INPUT_SIZE ){
        values[idx++] = token;
      }
      token=strtok(NULL, delimiter);
    } 

    linear = atof(values[0]);
    angular = atof(values[1]);

    move(linear,angular);

  }
  left_count_pulses = 0.0;
  right_count_pulses = 0.0;

  delay(100);
}

void left_encoder(){
  if(digitalRead(left_wheel_encB) == HIGH){
    left_count_pulses++;
  }
  else{
    left_count_pulses--;
  }
}

void right_encoder(){
  if(digitalRead(right_wheel_encB) == HIGH){
    right_count_pulses--;
  }
  else{
    right_count_pulses++;
  }
}

void move(float linear,float angular){
  if (angular > 0){
    rotateRight(angular);
  }
  else if (angular < 0){
    rotateLeft(angular);
  }
  else if (linear > 0){
    moveForward(linear);
  }
  else if (linear < 0){
    moveBackward(linear);
  }
  else{
    stop();
  }
}

void moveForward(float linear){
  //Left motors rotate forwards
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  //Right motors rotate forwards
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  int left_encoder_speed = int(abs(linear));
  int right_encoder_speed = int(abs(linear));

  if (left_encoder_speed < max_pwm){
    analogWrite(enA,left_encoder_speed);
  }
  else{
    analogWrite(enA,max_pwm);
  }
  if (right_encoder_speed < max_pwm){
    analogWrite(enB,right_encoder_speed);
  }
  else{
    analogWrite(enB,max_pwm);
  }

}

void moveBackward(float linear){
  //Left motors rotate backwards
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  //Right motors rotate backwards
  digitalWrite(in3,LOW);  
  digitalWrite(in4,HIGH);

  int left_encoder_speed = int(abs(linear));
  int right_encoder_speed = int(abs(linear));

  if (left_encoder_speed < max_pwm){
    analogWrite(enA,left_encoder_speed);
  }
  else{
    analogWrite(enA,max_pwm);
  }
  if (right_encoder_speed < max_pwm){
    analogWrite(enB,right_encoder_speed);
  }
  else{
    analogWrite(enB,max_pwm);
  }

}

void rotateLeft(float angular){
  //Left motors rotate backwards
  digitalWrite(in1,LOW); 
  digitalWrite(in2,HIGH);

  //Right motors rotate forwards
  digitalWrite(in3,HIGH); 
  digitalWrite(in4,LOW);

  int left_encoder_speed = int(abs(angular));
  int right_encoder_speed = int(abs(angular));

  if (left_encoder_speed < max_pwm){
    analogWrite(enA,left_encoder_speed);
  }
  else{
    analogWrite(enA,max_pwm);
  }
  if (right_encoder_speed < max_pwm){
    analogWrite(enB,right_encoder_speed);
  }
  else{
    analogWrite(enB,max_pwm);
  }
}

void rotateRight(float angular){
  // //Left motors rotate forwards
  // digitalWrite(in1,HIGH);
  // digitalWrite(in2,LOW);
  // //Right motors rotate forwards
  // digitalWrite(in3,HIGH);
  // digitalWrite(in4,LOW);

  //Left motors rotate forwards
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);

  //Right motors rotate backwards
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);

  int left_encoder_speed = int(abs(angular));
  int right_encoder_speed = int(abs(angular));

  if (left_encoder_speed < max_pwm){
    analogWrite(enA,left_encoder_speed);
  }
  else{
    analogWrite(enA,max_pwm);
  }
  if (right_encoder_speed < max_pwm){
    analogWrite(enB,right_encoder_speed);
  }
  else{
    analogWrite(enB,max_pwm);
  }
}

void stop(){
  //Stops left motors
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  analogWrite(enA,0);

  //Stops right motors
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  analogWrite(enB,0);
}