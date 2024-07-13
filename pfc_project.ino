#include <PID_v1.h>


float voltage1;
float voltage2;
float voltage3;
float voltage4;
float difference1;
float difference2;
#define enA 3
#define enB 9
#define IN1 12
#define IN2 13
#define IN3 6
#define IN4 7
#define ENC_IN_RIGHT_A 2
char command;
volatile long right_wheel_pulse_count = 0;
enum Direction {FORWARD, BACKWARD};

//Define Variables we'll be connecting to

double Setpoint1 = 0;
double Input1, Output1;


//Specify the links and initial tuning parameters
double Kp1=1, Ki1=1, Kd1=1;
PID myPID(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(enB,0);
  analogWrite(enA,0);
//  MR_direction(FORWARD);
//  ML_direction(FORWARD);
  // Open the serial port at 9600 bps
 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  myPID.SetMode(AUTOMATIC);
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);

  Serial.begin(9600);
}

void loop() {

  // Calculate voltage 1
  voltage1 = (float)analogRead(A0) / 1023 * 5;

  // Calculate voltage 2
  voltage2 = (float)analogRead(A5) / 1023 * 5;

  // Calculate voltage 3
  voltage3 = (float)analogRead(A3) / 1023 * 5;

  // Calculate voltage 4
  voltage4 = (float)analogRead(A4) / 1023 * 5;
  
  difference1=voltage1-voltage2;//A0 and A5 difference A0-A5
  difference2=voltage3-voltage4;//A3 and A4 difference A3-A4

  Input1 = (difference1);

  myPID.Compute();
  Serial.print("VOltage1");
  Serial.println(voltage1);

  Serial.print("VOltage2");
  Serial.println(voltage2);
  Serial.println(difference1);

  Serial.print("This is Output");
  Serial.println(Output1);
  if (difference1>0.8){
    //move motor clockwise
      ML_direction(BACKWARD);
      analogWrite(enB,Output1);
      //Serial.println(difference1);
      
      }
   else{
     analogWrite(enB,0);
   }
 if (difference1<-0.8){
    //move motor anticlockwise
      ML_direction(FORWARD);
      analogWrite(enB,Output1);
//      Serial.print("Moving anticlockwise");
     // Serial.println(difference1);
  }
   else{
     analogWrite(enB,0);
   }

  // Serial.print("Voltage Difference 1:");
  // Serial.println(difference1);
  // Serial.print("Voltage Difference 2:");
  // Serial.println(difference2);
  delay(500);
//  Serial.print("Voltage_West");
//  Serial.println(voltage2);
  
}

void motion_burst(int num){
  if(num==0){
    analogWrite(enA,0);
    delay(50);
    analogWrite(enA,30);
    delay(50);
    analogWrite(enA,0);
  }
  else{
    analogWrite(enB,0);
    delay(50);
    analogWrite(enB,30);
    delay(50);
    analogWrite(enB,0);
  }
  delay(500);
}

void MR_direction(Direction dir)
{
  if (dir == FORWARD)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (dir == BACKWARD)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

//
void ML_direction(Direction dir)
{
  if (dir == FORWARD)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (dir == BACKWARD)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void right_wheel_pulse() {
  right_wheel_pulse_count++;
//  Serial.println(right_wheel_pulse_count*0.33027);
}