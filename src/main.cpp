#include <Arduino.h>
#include <Servo.h>

// The firmware expects normalized commands in the ``x y`` format, where each
// value ranges from 0 to 100. That gives ~1.8° resolution for standard 180°
// hobby servos because 100 steps cover the 1 ms to 2 ms control pulse span.

// Angle limits in degrees for the two axes. They can be updated in the field
// if the mechanical assembly changes.
constexpr float kMinX = 0.0f;
constexpr float kMaxX = 130.0f;
constexpr float kMinY = 0.0f;
constexpr float kMaxY = 80.0f;

// Output pins used by the gimbal.
constexpr uint8_t kServoPinX = 6;
constexpr uint8_t kServoPinY = 5;

Servo servoX;
Servo servoY;

void setupServos()
{
  servoX.attach(kServoPinX);
  servoY.attach(kServoPinY);

  servoX.write(90); // Start in the middle of the travel range.
  servoY.write(45);
}

void moveOne(int servo, int value)
{
  const bool isX = servo == 0;
  const float normalized = value / 100.0f;

  if (isX)
  {
    const int targetAngle = kMinX + normalized * (kMaxX - kMinX);
    servoX.write(targetAngle);
    Serial.print("Moved: servo_x: ");
    Serial.print(value);
    Serial.println(" |  servo_y:  n");
  }
  else if (servo == 1)
  {
    const int targetAngle = kMinY + normalized * (kMaxY - kMinY);
    servoY.write(kMaxY - targetAngle); // Invert to match mechanical setup.
    Serial.print("Moved: servo_x: n");
    Serial.print(" |  servo_y:  ");
    Serial.println(value);
  }
}

void moveServo(int servo_x, int servo_y)
{
  if (servo_x || servo_x == 0)
  {
    const float normalized = servo_x / 100.0f;
    const int targetAngle = kMinX + normalized * (kMaxX - kMinX);
    servoX.write(targetAngle);
  }

  if (servo_y || servo_y == 0)
  {
    const float normalized = servo_y / 100.0f;
    const int targetAngle = kMinY + normalized * (kMaxY - kMinY);
    servoY.write(kMaxY - targetAngle);
  }

  Serial.print("Moved: servo_x: ");
  Serial.print(servo_x);
  Serial.print(" |  servo_y:  ");
  Serial.println(servo_y);
}

void processCommand(String command)
{
  // Commands follow the "<x> <y>" convention. Use "n" to keep the last value.
  command.trim();
  const int spaceIndex = command.indexOf(' ');

  if (spaceIndex <= 0)
  {
    Serial.println("Invalid command format. Use: servo_x servo_y");
    Serial.println("Example: 50 50");
    return;
  }

  String first = command.substring(0, spaceIndex);
  String second = command.substring(spaceIndex + 1);
  first.trim();
  second.trim();

  int servo_x;
  int servo_y;

  if (first == "n")
    servo_x = -1;
  else
    servo_x = first.toInt();

  if (second == "n")
    servo_y = -1;
  else
    servo_y = second.toInt();

  if (servo_x >= 0 && servo_x <= 100 && servo_y >= 0 && servo_y <= 100)
  {
    moveServo(servo_x, servo_y);
  }
  else if (servo_y == -1 && servo_x != -1)
  {
    moveOne(0, servo_x);
  }
  else if (servo_x == -1 && servo_y != -1)
  {
    moveOne(1, servo_y);
  }
  else if (servo_x == -1 && servo_y == -1)
  {
    Serial.println("n n command.");
  }
  else
  {
    Serial.println("Invalid command. Use normalized values (0 - 100).");
    Serial.println("Example: 50 50");
  }
}

void setup()
{
  Serial.begin(9600);

  Serial.println();
  Serial.println("------------------ Servo control initialized ------------------");
  Serial.println("Waiting for commands in the form 'servo_x servo_y'.");
  Serial.println("Example: 55 100 (55% for X, 100% for Y).");
  Serial.println("Send commands via the serial terminal.");

  setupServos();
}

void loop()
{
  static String inputBuffer = "";

  while (Serial.available() > 0)
  {
    const char incomingChar = Serial.read();

    if (incomingChar == '\n' || incomingChar == '\r')
    {
      inputBuffer.trim();

      if (inputBuffer.length() > 0)
      {

        processCommand(inputBuffer);
        Serial.println();
      }

      inputBuffer = "";
    }
    else
    {
      inputBuffer += incomingChar;
    }
  }
}
