// Required Arduino Libraries
#include <Servo.h>

// Custom Libraries
#include <crawler_basic.h>

// Instantiate Servos for Joints
Servo j1, j2;

// Instantiate State Space
//CrawlerStateSpace * ss;
//CrawlerSSFactory ssFactory;
//Printer printer;

// Instantiate Arm
Arm crawlerArm;
// Instantiate Lidar
Lidar distanceSensor;

// Assign the servo joint pins.
void assignJoints(Arm& arm){
  j1.attach(arm.getJointPin(1));
  j2.attach(arm.getJointPin(2));
}

// Send the joint commands to the servos.
void sendJointCommands(Arm& arm){
  j1.write(arm.getJointAngleCalibrated(1));
  j2.write(arm.getJointAngleCalibrated(2));
}

void setup() {
    // Initialize the serial object.
  Serial.begin(57600);
  delay(2000);
  // Assign the servo objects
  assignJoints(crawlerArm);
  crawlerArm.setJointAngle(2,10);
  
//  ssFactory.buildStateSpace();
//  ss = ssFactory.getStateSpace();
//  CrawlerStateSpaceIter ssIter(ss);
  
//  Serial.print("StateSpace: ");
//  Serial.println(ss->size());
//  ssIter.first();
//  (*ssIter)->sendToPrint(printer);

}

void loop() {

  // Read the distance sensor.
  distanceSensor.readSensorNTimes(5);
    
  // Print the value.
  Serial.println(distanceSensor.getDistance());

//    // Set the joints to some value.
//  crawlerArm.setJointAngle(1,-100);
//  
//  // Command the joints.
//  sendJointCommands(crawlerArm);
//        
//  // Wait a bit
//  delay(50);
//  
//      // Set the joints to some value.
//  crawlerArm.setJointAngle(1,-70);
//  
//  // Command the joints.
//  sendJointCommands(crawlerArm);
//  
//  // Wait a bit
//  delay(50);
  
  ///*** Scripted gait ***///
  
  // J2 = 20 Down
  // J2 = 40 Up
  
  // J1 = [-60,-80,-100] Cycle in each J2 position
  

  int J2 = 40; // Up
  
  crawlerArm.setJointConfig(-100, J2);
  sendJointCommands(crawlerArm);
  delay(200);

  crawlerArm.setJointConfig(-80, J2);
  sendJointCommands(crawlerArm);
  delay(200);
  
  crawlerArm.setJointConfig(-60, J2);
  sendJointCommands(crawlerArm);
  delay(200);

  J2 = 0; // Down

  crawlerArm.setJointConfig(-60, J2);
  sendJointCommands(crawlerArm);
  delay(200);
  
  crawlerArm.setJointConfig(-80, J2);
  sendJointCommands(crawlerArm);
  delay(200);

  crawlerArm.setJointConfig(-100, J2);
  sendJointCommands(crawlerArm);
  delay(200);


}
