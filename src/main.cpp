#include <Arduino.h>
#include <Servo.h>

// Define servo angle limits for normalization
#define MIN_X 0    // Minimum angle for X servo
#define MAX_X 130  // Maximum angle for X servo
#define MIN_Y 0    // Minimum angle for Y servo
#define MAX_Y 90   // Maximum angle for Y servo

// Create servo objects
Servo servoMotor_x;
Servo servoMotor_y;

// Define servo pins
int pinoServo_x = 6; // X servo pin
int pinoServo_y = 5; // Y servo pin

void setupServos() {
  // Attach servos to pins
  servoMotor_x.attach(pinoServo_x);
  servoMotor_y.attach(pinoServo_y);
  
  // Set initial positions
  servoMotor_x.write(90); // Center position
  servoMotor_y.write(45); // Center position
}

void moveServo(int servo, double normalized_position) {
  // Calculate target angle based on servo and normalized position
  int target_angle;

  if (servo == 0) { // X servo
    target_angle = MIN_X + (normalized_position * (MAX_X - MIN_X));
    servoMotor_x.write(target_angle);
    Serial.print("Moving X servo to angle: ");
    Serial.println(target_angle);
  } else if (servo == 1) { // Y servo
    target_angle = MIN_Y + (normalized_position * (MAX_Y - MIN_Y));
    servoMotor_y.write(target_angle);
    Serial.print("Moving Y servo to angle: ");
    Serial.println(target_angle);
  }

  // Wait for servo to reach position (approximate timing)
  delay(500); // Adjust based on servo speed

  Serial.println("Servo reached target position");
}




void processCommand(String command) {
  // Parse the command
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex > 0) {
    // Extract servo number (0 or 1)
    String servoStr = command.substring(0, spaceIndex);
    int servo = servoStr.toInt();

    // Extract position (0.0 to 1.0)
    String positionStr = command.substring(spaceIndex + 1);
    double normalized_position = positionStr.toDouble();

    // Validate inputs
    if (servo >= 0 && servo <= 1 && normalized_position >= 0.0 && normalized_position <= 1.0) {
      // Initialize servos if not already done
      
      // Move servo to target position
      moveServo(servo, normalized_position);
      
      Serial.println("Command completed - ready for next command");
    } else {
      Serial.println("Invalid command. Use format: servo position");
      Serial.println("servo: 0 (X) or 1 (Y)");
      Serial.println("position: 0.0 to 1.0");
    }
  } else {
    Serial.println("Invalid command format. Use: servo position");
    Serial.println("Example: 0 0.5");
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  Serial.println();
  Serial.println("------------------ Servo control initialized ------------------" );
  Serial.println("Waiting for command in format: servo position");
  Serial.println("Example: 0 0.5 (X servo, 50% position)");
  Serial.println("Send commands via serial communication");
  setupServos();

}


void loop() {
  static String inputBuffer = "";
  
  // Read all available characters
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    if (incomingChar == '\n' || incomingChar == '\r') {
      // End of line reached, process the command
      inputBuffer.trim();
      
      if (inputBuffer.length() > 0) {
        Serial.print("Received command: ");
        Serial.println(inputBuffer);
        
        processCommand(inputBuffer);
        Serial.println();
      }
      
      inputBuffer = ""; // Clear buffer for next command
    } else {
      inputBuffer += incomingChar;
    }
  }
}
/*

0 0
0 1
0 0.5

1 0
1 1
1 0.5


*/