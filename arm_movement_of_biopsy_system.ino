//BME project at CentraleLille
//teamwork of Lumi Xia and Jonior Nduami-Momsempo, etc.
#include <BraccioRobot.h>
#include <Servo.h>
#include <Stepper.h>
//init() will start the robot and move to the initial position, which is:
  //Base     (M1): 90 degrees
  //Shoulder (M2): 90 degrees
  //Elbow    (M3): 90 degrees
  //Wrist    (M4): 90 degrees
  //Wrist rot(M5): 90 degrees
  //gripper  (M6): 72 degrees
  //The initial position can be changed by supplying a Position as a parameter
  //to init() 

const int stepsPerRevolution = 512;  //  the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 13, A2, A1, A3);//1-3-2-4
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance; 
long fsrForce;       // Finally, the resistance converted to force
 int fsrAnalogPin = A0;     // the FSR and 10K pulldown are connected to a0
//const int analogIn = A0;
//int analogVal = 0;
int LEDbrightness;
int LEDpin = 7;
int trigPin=A4; //Sensor Trig pin connected to Arduino pin 13
int echoPin=A5;  //Sensor Echo pin connected to Arduino pin 11
float pingTime;  //time for ping to travel from sensor to target and return
float targetDistance; //Distance to Target in inches
float speedOfSound=776.5; //Speed of sound in miles per hour when temp is 77 degrees.
int n=0;
int m=0;
   


void setup() {
  Serial.begin(115200);
  BraccioRobot.init();
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);
    // initialize the serial port:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

Position pos;

void loop() {
  if (n==0){
  delay(1000);
// Set the position
  // M1=base degrees. Allowed values from 0 to 180 degrees
  // M2=shoulder degrees. Allowed values from 15 to 165 degrees
  // M3=elbow degrees. Allowed values from 0 to 180 degrees
  // M4=wrist degrees. Allowed values from 0 to 180 degrees
  // M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
  // M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  // (M1,  M2,  M3,  M4, M5,  M6)
  
  pos.set( 90,  60, 150, 180,  0,  73);

  // Move the robot to the position with a specified speed between 20-200 degrees per second
  //BraccioRobot.moveToPosition(pos, 100);  
  //Wait 1 second
  delay(1000);
  BraccioRobot.moveToPosition(pos.set(90,  88, 174, 128 , 0,  73), 50);  
  delay(1000); 
  // Move the robot to a new position with speed 50 degrees per second. 
  BraccioRobot.moveToPosition(pos.set(90,  120, 150, 120, 0,  73), 50);  
  delay(1000);
 //BraccioRobot.moveToPosition(pos.set(90,  165, 110, 125, 0,  73), 20);
 //delay(1000);
//Adjust the best approaching angle by changing the angle of M4
 BraccioRobot.moveToPosition(pos.set(90,  165, 110, 105 , 0,  73), 20);
 delay(1000);
  n++;
  }

  Serial.print(fsrForce);
  Serial.print(" , ");
 
  digitalWrite(trigPin, LOW); //Set trigger pin low
  delayMicroseconds(2000); //Let signal settle
  digitalWrite(trigPin, HIGH); //Set trigPin high
  delayMicroseconds(15); //Delay in high state
  digitalWrite(trigPin, LOW); //ping has now been sent
  delayMicroseconds(10); //Delay in low state
  
  pingTime = pulseIn(echoPin, HIGH);  //pingTime is presented in microceconds
  pingTime=pingTime/1000000; //convert pingTime to seconds by dividing by 1000000 (microseconds in a second)
  pingTime=pingTime/3600; //convert pingtime to hourse by dividing by 3600 (seconds in an hour)
  targetDistance= speedOfSound * pingTime;  //This will be in miles, since speed of sound was miles per hour
  targetDistance=targetDistance/2; //Remember ping travels to target and back from target, so you must divide by 2 for actual target distance.
  targetDistance= targetDistance*63360;    //Convert miles to inches by multipling by 63360 (inches per mile)
  Serial.println(targetDistance);
  
  delay(100); //delay tenth of a  second to slow things down a little.
  
  if (targetDistance <= 5 && targetDistance >= 0) {
  
  //step one revolution  in one direction:
  //Serial.println("clockwise");
  if (m <=20)
  {myStepper.step(-230);
   fsrReading = analogRead(fsrAnalogPin);  
  // we'll need to change the range from the analog reading (0-1023) down to the range
  // used by analogWrite (0-255) with map!
  LEDbrightness = map(fsrReading, 0, 1023, 0, 255);
  // LED gets brighter the harder you press
  analogWrite(LEDpin, LEDbrightness);
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  if (fsrVoltage == 0) {
   // Serial.println("No pressure"); 
  Serial.print(fsrVoltage); 
  Serial.print(" , "); 
  } 
  else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
   
    fsrConductance = 1000000;           // we measure in microhms
    fsrConductance /= fsrResistance;
  
    // Use the two FSR guide graphs to approximate the force in newtons
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
     // Serial.println(" Newtons");       
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      // Serial.println(" Newtons");            
    } 
  }
  //Serial.println("--------------------");
   m=m+1;
   }
  
    
  else if (m==21){
    //step one revolution in the other direction:
    delay(9000);
  //Serial.println("counterclockwise");
  myStepper.step(4600);
  //while(1){/*empty*/} 
//will make the stepper motor to perform the task once and stop the all program
 // delay(500); }
  delay(10000);
  BraccioRobot.moveToPosition(pos.set(90,  120, 150, 120, 0,  73), 50);  
  delay(1000);

  BraccioRobot.moveToPosition(pos.set(90,  88, 174, 128 , 0,  73), 50);  
  delay(1000);

  BraccioRobot.moveToPosition(pos.set(90,  60, 150, 180, 0,  73), 100);  
  delay(1000);
  m++;
    }
  }
  delay(1000);
}
