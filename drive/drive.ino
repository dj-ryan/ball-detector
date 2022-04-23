#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>

#include <stdint.h>

LSM6 imu;

Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;

int32_t angle;      // units: millidegrees
int32_t angleX;     // units: millidegrees
int32_t angleZ;     // units: millidegrees
int32_t angleRate;  // units: degrees/s (or millidegrees/ms)
int16_t motorSpeed; // current (average) motor speed setting
int32_t distanceLeft;
int32_t speedLeft;
int32_t driveLeft;
int32_t distanceRight;
int32_t speedRight;
int32_t driveRight;

int32_t gYZero;
int32_t gXZero;
int32_t angleRateX; // degrees/s
int32_t gZZero;
int32_t angleRateZ; // degrees/s

const uint8_t CALIBRATION_ITERATIONS = 100;
const uint8_t UPDATE_TIME_MS = 10;
const int16_t MOTOR_SPEED_LIMIT = 300;
const int16_t DISTANCE_DIFF_RESPONSE = -50;

// declare range sensor vars ## ADDED BY GROUP ##
uint8_t rangeSensorPin = A6;

void driveSetup()
{
  // Initialize IMU.
  Wire.begin();

  if (!imu.init())
  {
    while (true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();

  // Set the gyro full scale to 1000 dps because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  int32_t totalX = 0;
  int32_t totalZ = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    totalX += imu.g.x;
    totalZ += imu.g.z;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
  gXZero = totalX / CALIBRATION_ITERATIONS;
  gZZero = totalZ / CALIBRATION_ITERATIONS;
}

void setup()
{
  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  ledYellow(0);
  ledRed(1);
  driveSetup();
  ledRed(0);

  Serial.begin(57600);
  Serial.println("Done starting up.");
}

void integrateGyro()
{
  // Convert from full-scale 1000 deg/s to deg/s.
  angleRate = (imu.g.y - gYZero) / 29;
  angleRateX = (imu.g.x - gXZero) / 29;
  angleRateZ = (imu.g.z - gZZero) / 29;

  angle += angleRate * UPDATE_TIME_MS;
  angleX += angleRateX * UPDATE_TIME_MS;
  angleZ += angleRateZ * UPDATE_TIME_MS;
}

void integrateEncoders()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;
}

void driveUpdateSensors()
{
  imu.read();
  integrateGyro();
  integrateEncoders();
}

void driveDoDriveTicks()
{
  distanceLeft -= driveLeft;
  distanceRight -= driveRight;
  speedLeft -= driveLeft;
  speedRight -= driveRight;
}

void horizontalDrive(int16_t leftSpeed, int16_t rightSpeed)
{
  driveLeft = -leftSpeed;
  driveRight = -rightSpeed;
}

void driveUpdate()
{
  driveUpdateSensors();

//Serial.print("right: ");
//Serial.print(driveRight);
//Serial.print(" | left: ");
//Serial.print(driveLeft);
//Serial.println();

motors.setLeftSpeed(driveLeft);
motors.setRightSpeed(driveRight);

}

/**
   Send debug data over the serial port in human readable format
 **/
void sendDebugData()
{
  uint32_t currentMS = millis();
  static uint32_t lastMS = 0;
  // Only run this every once in a while.
  if ((currentMS - lastMS) < 2000)
    return;

  Serial.println("\r\n----");
  Serial.print("Millis: ");
  Serial.println(millis());
  Serial.print("Batt: ");
  Serial.print(readBatteryMillivolts());
  Serial.println("mV");
  Serial.print("Gyro Y: ");
  Serial.print(angle / 1000);
  Serial.println("deg");
  Serial.print("Gyro X: ");
  Serial.print(angleX / 1000);
  Serial.println("deg");
  Serial.print("Gyro Z: ");
  Serial.print(angleZ / 1000);
  Serial.println("deg");
  Serial.print("Cmd Speed l: ");
  Serial.print(driveLeft);
  Serial.print(" r: ");
  Serial.println(driveRight);
  Serial.print("Speed l: ");
  Serial.print(speedLeft);
  Serial.print(" r: ");
  Serial.println(speedRight);
  Serial.print("Dist l: ");
  Serial.print(distanceLeft);
  Serial.print(" r: ");
  Serial.println(distanceRight);
  Serial.print("Enc l: ");
  Serial.print(encoders.getCountsLeft());
  Serial.print(" r :");
  Serial.println(encoders.getCountsRight());

  lastMS = currentMS;
}

/**
   Sends a 32 bit value over the serial port with the most significant byte first.
   Also updates the checksum.
 **/
void sendSignedInt32MSBAndUpdateChecksum(int32_t val, uint8_t *checksum)
{
  uint8_t v = (val >> 24) & 0xFF;
  *checksum += v;
  Serial.write(v);
  v = (val >> 16) & 0xFF;
  *checksum += v;
  Serial.write(v);
  v = (val >> 8) & 0xFF;
  *checksum += v;
  Serial.write(v);
  v = (val >> 0) & 0xFF;
  *checksum += v;
  Serial.write(v);
}

/**
   Send data packets to ROS
 **/
void sendDataToROS()
{
  uint32_t currentMS = millis();
  static uint32_t lastMS = 0;
  // Only run this every once in a while.
  // N.B. there will be rollovers every ~65sec, but shouldn't be an issue
  if ((currentMS - lastMS) < 100)
    return;

  // read sensor values ## ADDED BY GROUP ##
  int32_t rangeValue = analogRead(rangeSensorPin);

  // Start out with the checksum non-zero
  uint8_t checksum = 0xCD;
  // Start bytes
  Serial.write('C');
  Serial.write('D');
  sendSignedInt32MSBAndUpdateChecksum(currentMS, &checksum);
  sendSignedInt32MSBAndUpdateChecksum(readBatteryMillivolts(), &checksum);
  sendSignedInt32MSBAndUpdateChecksum(angle, &checksum);
  sendSignedInt32MSBAndUpdateChecksum(angleX, &checksum);
  sendSignedInt32MSBAndUpdateChecksum(angleZ, &checksum);
  sendSignedInt32MSBAndUpdateChecksum(driveLeft, &checksum);
  sendSignedInt32MSBAndUpdateChecksum(driveRight, &checksum);
  sendSignedInt32MSBAndUpdateChecksum(speedLeft, &checksum);
  sendSignedInt32MSBAndUpdateChecksum(speedRight, &checksum);
  //  sendSignedInt32MSBAndUpdateChecksum(distanceLeft,&checksum);
  //  sendSignedInt32MSBAndUpdateChecksum(distanceRight,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(encoders.getCountsLeft(), &checksum);
  sendSignedInt32MSBAndUpdateChecksum(encoders.getCountsRight(), &checksum);

  // ## GROUP ADDED ##
  sendSignedInt32MSBAndUpdateChecksum(rangeValue, &checksum);

  /* //Some unit tests
    sendSignedInt32MSBAndUpdateChecksum(0,&checksum);
    sendSignedInt32MSBAndUpdateChecksum(-1,&checksum);
    sendSignedInt32MSBAndUpdateChecksum(1,&checksum);
    sendSignedInt32MSBAndUpdateChecksum(0x7FFFFFFF,&checksum);
    sendSignedInt32MSBAndUpdateChecksum(0x80000000,&checksum);
  */

  Serial.write(checksum);

  lastMS = currentMS;
}

/**
   Process commands from ROS
 **/
void processDataFromROS()
{
#define STATE_START 0
#define STATE_LEFT 1
#define STATE_RIGHT 2
#define STATE_CHECKSUM 3

  // Variables that need to stick around as we iterate through our state machine.
  static int8_t checksum;
  static uint8_t state = STATE_START;
  static int8_t left;
  static int8_t right;

  int8_t c = 0;
  // While we have more bytes from the serial port
  while (Serial.available() > 0)
  {
    Serial.readBytes((char *)&c, 1);
    // Go through our state machine
    switch (state)
    {
      case STATE_START:
        // Check to see if it is our start byte, if so transition
        if (c == (int8_t)0xCD)
        {
          state = STATE_LEFT;
          checksum = 0xCD;
        }
        break;
      case STATE_LEFT:
        checksum += c;
        left = c;
        state = STATE_RIGHT;
        break;
      case STATE_RIGHT:
        checksum += c;
        right = c;
        state = STATE_CHECKSUM;
        break;
      case STATE_CHECKSUM:
        if (c != checksum)
        {
          // Serial.print("command "); Serial.print(left); Serial.print(", "); Serial.println(right);
          Serial.print("chksum failed ");
          Serial.print(c);
          Serial.print(" != ");
          Serial.println(checksum);
        }
        else
        {
          // Serial.print("command "); Serial.print(left); Serial.print(", "); Serial.println(right);
          // balanceDrive(left,right);

          // call drive update function
          
          Serial.print("HD - left: ");
          Serial.print(left);
          Serial.print("| right: ");
          Serial.print(right);
          Serial.println();
          
          horizontalDrive(left, right);
        }
        state = STATE_START;
        break;
      default:
        state = STATE_START;
        break;
    }
  }
}

void loop()
{

  // Send the debug data (human readable) over the serial port. Probably better
  // to not do this while sending ROS packet, but it won't break anything
  // sendDebugData();
  
  if(buttonA.isPressed()){
    horizontalDrive(0,0);
  }
  

  driveUpdate();

  sendDataToROS();
  processDataFromROS();

}
