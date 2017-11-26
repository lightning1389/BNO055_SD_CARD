/********************************************************************************/
// Author: Ilhan Dogan 							                                 //
// Date 20.10.2015                                                               //
// Version 1.0                                                                   //
// Program that logs the output of the 9 DOF Adafruit Sensor and a Bosch BNO055  //
//                                                                               //
/********************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)
/* Assign a unique ID to the sensors */
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_9DOF                dof    = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro        = Adafruit_L3GD20_Unified(20);
/*
      Initialises all the sensors used by this example
*/
void setup(void)
{
  // SD CARD USE to log the BNO055 and Adafruit 9 DOF    
  
  Serial.begin(115200);     // Serial communication is also established
  if (!SD.begin(10, 11, 12, 13)) {
    Serial.println("Card init. failed!");
  }
  // SD CARD END                                                             //
  Serial.println("Orientation Sensor Raw Data Test and Adafruit 9DOF Tester"); Serial.println("");
  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(5000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  // 9DOF Adafruit
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}
void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  String filename = "datalog.csv";
  String dataString = "";
  float Sensor;
  unsigned long currentMillis = millis();

  // Header for the CSV file
  dataString = "Milliseconds; ACCELERATION X_9DOF; ACCELERATION Y_9DOF; ACCELERATION Z_9DOF;  MAG_9DOFX;MAG_9DOFY;MAG_9DOFZ;GYRO_9DOFX;GYRO_9DOFY;GYRO_9DOFZ; ACCELERATION X_L; ACCELERATION Y_L; ACCELERATION Z_L; ACCELERATION X_G; ACCELERATION Y_G; ACCELERATION Z_G; ACCELERATION X_A; ACCELERATION Y_A; ACCELERATION Z_A; Magnetometer X; Magnetometer Y; Magnetometer Z; Euler X; Euler Y; Euler Z\n";
  while (1)
  {

    String stringOne =  String(millis(), DEC);     // Track time

    dataString += stringOne;
    dataString += ";";

    accel.getEvent(&event);
    Sensor = event.acceleration.x;
    dataString += Sensor;
    dataString += ";";
    Sensor = event.acceleration.y;
    dataString += Sensor;
    dataString += ";";
    Sensor = event.acceleration.z;
    dataString += Sensor;
    dataString += ";";

    mag.getEvent(&event);
    Sensor = event.magnetic.x;
    dataString += Sensor;
    dataString += ";";
    Sensor = event.magnetic.y;
    dataString += Sensor;
    dataString += ";";
    Sensor = event.magnetic.z;
    dataString += Sensor;
    dataString += ";";

    gyro.getEvent(&event);
    Sensor = event.gyro.x;
    dataString += Sensor;
    dataString += ";";
    Sensor = event.gyro.y;
    dataString += Sensor;
    dataString += ";";
    Sensor = event.gyro.z;
    dataString += Sensor;
    dataString += ";";


    //BNO055 Data
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    dataString += acc.x();
    dataString += ";";
    dataString += acc.y();
    dataString += ";";
    dataString += acc.z();
    dataString += ";";

    imu::Vector<3> acc1 = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    dataString += acc1.x();
    dataString += ";";
    dataString += acc1.y();
    dataString += ";";
    dataString += acc1.z();
    dataString += ";";

    imu::Vector<3> acc2 = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    dataString += acc2.x();
    dataString += ";";
    dataString += acc2.y();
    dataString += ";";
    dataString += acc2.z();
    dataString += ";";

    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    dataString += mag.x();
    dataString += ";";
    dataString += mag.y();
    dataString += ";";
    dataString += mag.z();
    dataString += ";";
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    dataString += euler.x();
    dataString += ";";
    dataString += euler.y();
    dataString += ";";
    dataString += euler.z();



    File dataFile = SD.open("Datalog.csv", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
    dataString = "";      // Delete dataString
  }
}
