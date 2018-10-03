#include <NewPing.h>
#include <Servo.h>

#define BRAKE 0
#define CW    1
#define CCW   2
#define CS_THRESHOLD 15   // Definition of safety current (Check: "1.3 Monster Shield Example").

//MOTOR 1
#define MOTOR_A1_PIN 5
#define MOTOR_B1_PIN 4
//MOTOR 2
#define MOTOR_A2_PIN 6
#define MOTOR_B2_PIN 7
#define PWM_MOTOR_1 2
#define PWM_MOTOR_2 3

#define CURRENT_SEN_1 A2
#define CURRENT_SEN_2 A3

#define EN_PIN_1 A0
#define EN_PIN_2 A1

#define MOTOR_1 0
#define MOTOR_2 1

#define SONAR_NUM 3  // Number or sensors.
#define TRIG_PIN_MIDDLE 30 
#define ECHO_PIN_MIDDLE 31
#define TRIG_PIN_LEFT 34 
#define ECHO_PIN_LEFT 35
#define TRIG_PIN_RIGHT 40 
#define ECHO_PIN_RIGHT 41
#define MAX_DISTANCE_POSSIBLE 800 
#define COLL_DIST 20 
#define TURN_DIST COLL_DIST+10
#define PING_INTERVAL 30 // Milliseconds between pings.

bool edge;
short usSpeed = 30;  //default motor speed
unsigned short usMotor_Status = BRAKE;
unsigned short usMotor_Status1 = BRAKE;
unsigned short usMotor_Status2 = BRAKE;

//ULTRASONIC SENSORS
unsigned long pingTimer[SONAR_NUM]; // When each pings.
unsigned int cm[SONAR_NUM]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.

NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE_POSSIBLE),
  NewPing(TRIG_PIN_MIDDLE, ECHO_PIN_MIDDLE, MAX_DISTANCE_POSSIBLE),
  NewPing(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE_POSSIBLE)
};

//PERIMETER RECEIVER
int perimeterReceiver = 5;    // Select the input pin for the perimeter receiver
int perimeterReceiverValue[300]; // Array to store the value coming from function generator
int i = 0; //counter
Servo ESC;
 
void setup()                         
{
  //MOTORS
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  pinMode(CURRENT_SEN_1, OUTPUT);
  pinMode(CURRENT_SEN_2, OUTPUT);  

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  //ULTRASONIC SENSORS
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }

  //brushless motor
  ESC.attach(9);
  ESC.write(0);
  delay(1000);
  ESC.write(50);
  delay(6000);
  
  Serial.begin(9600);
  Serial.println("Begin motor control");
  Serial.println();
  Serial.println("Enter number for control option:");
  Serial.println("1. STOP");
  Serial.println("2. FORWARD");
  Serial.println("3. REVERSE");
  Serial.println("4. LEFT");
  Serial.println("5. RIGHT");
  Serial.println("6. READ CURRENT");
  Serial.println("+. INCREASE SPEED");
  Serial.println("-. DECREASE SPEED");
  Serial.println();

}

void changeDirection(){
    Forward();
    delay(1000);
    Stop();
    delay(500);
    if((cm[0] <= cm[2]) && (cm[0] != 0) && (cm[2] != 0) && (cm[1] != 0)) {
      Right();
      Serial.println("right");
    }
    else if((cm[0] > cm[2]) && (cm[0] != 0) && (cm[2] != 0) && (cm[1] != 0)) {
      Left(); 
      Serial.println("left");
    }else if(edge == 1){
      Right();
      }
    delay(2000);
    Stop()
  }

void oneSensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i+1);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}

void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void Stop()
{
  Serial.println("Stop");
  usMotor_Status = BRAKE;
  motorGo(MOTOR_1, usMotor_Status, 0);
  motorGo(MOTOR_2, usMotor_Status, 0);
}

void Forward()
{
  Serial.println("Forward");
  usMotor_Status = CW;
  motorGo(MOTOR_1, usMotor_Status, usSpeed+7);
  motorGo(MOTOR_2, usMotor_Status, usSpeed+7);
}

void Reverse()
{
  Serial.println("Reverse");
  ESC.write(60);
  usMotor_Status = CCW;
  motorGo(MOTOR_1, usMotor_Status, usSpeed);
  motorGo(MOTOR_2, usMotor_Status, usSpeed);
}

void Left()
{
  Serial.println("left");
  usMotor_Status1 = CCW;
  usMotor_Status2 = CW;
  motorGo(MOTOR_1, usMotor_Status1, usSpeed);
  motorGo(MOTOR_2, usMotor_Status2, usSpeed);
}

void Right()
{
  Serial.println("right");
  usMotor_Status1 = CW;
  usMotor_Status2 = CCW;
  motorGo(MOTOR_1, usMotor_Status1, usSpeed);
  motorGo(MOTOR_2, usMotor_Status2, usSpeed);
}

void IncreaseSpeed()
{
  usSpeed = usSpeed + 10;
  if(usSpeed > 255)
  {
    usSpeed = 255;  
  }
  
  Serial.print("Speed +: ");
  Serial.println(usSpeed);

  motorGo(MOTOR_1, usMotor_Status, usSpeed);
  motorGo(MOTOR_2, usMotor_Status, usSpeed);  
}

void DecreaseSpeed()
{
  usSpeed = usSpeed - 10;
  if(usSpeed < 0)
  {
    usSpeed = 0;  
  }
  
  Serial.print("Speed -: ");
  Serial.println(usSpeed);

  motorGo(MOTOR_1, usMotor_Status, usSpeed);
  motorGo(MOTOR_2, usMotor_Status, usSpeed);  
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if(motor == MOTOR_1)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, pwm + 12); 
  }
  else if(motor == MOTOR_2)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_2, pwm);
  }
}

void loop() 
{
  char user_input;
  
  //motors
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);  
  
  //ultrasonic sensors
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) {
        oneSensorCycle(); // Do something with results.
      }
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }

  //perimeter receiver
  edge = 0;
  // reads 300 values for waveform
  for (i=0; i < 299; i++)
  {   
     perimeterReceiverValue[i] = analogRead(perimeterReceiver);
     if(perimeterReceiverValue[i] > 845){
        edge = 1;
        Serial.println();
        Serial.println("EDGE");
        Serial.println();
      }
  }

  Reverse();
  if(((cm[0] < COLL_DIST) && (cm[0] != 0) && (cm[1] != 0) && (cm[2] != 0)) || ((cm[1] < COLL_DIST) && (cm[1] != 0) && (cm[0] != 0) && (cm[2] != 0)) || ((cm[2] < COLL_DIST) && (cm[2] != 0) && (cm[0] != 0) && (cm[1] != 0)) || (edge == 1)){
      changeDirection();
      Serial.println("Change direction to ");
    }
}



