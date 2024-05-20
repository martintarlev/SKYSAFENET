#define I2C_SDA 8
#define I2C_SCL 9

#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h> // for NVS

#define BNO055_SAMPLERATE_DELAY_MS (1000)

Adafruit_BNO055 bno = Adafruit_BNO055();

Preferences Calibration; 

void displayCalStatus(void)
{
   uint8_t system, gyro, accel, mag;
   system = gyro = accel = mag = 0;
   bno.getCalibration(&system, &gyro, &accel, &mag);
   Serial.print("\t");
   if (!system)
   {
       Serial.print("! ");
   }

   Serial.print("Sys:");
   Serial.print(system, DEC);
   Serial.print(" G:");
   Serial.print(gyro, DEC);
   Serial.print(" A:");
   Serial.print(accel, DEC);
   Serial.print(" M:");
   Serial.print(mag, DEC);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
   Serial.print("Accelerometer: ");
   Serial.print(calibData.accel_offset_x); Serial.print(" ");
   Serial.print(calibData.accel_offset_y); Serial.print(" ");
   Serial.print(calibData.accel_offset_z); Serial.print(" ");

   Serial.print("\nGyro: ");
   Serial.print(calibData.gyro_offset_x); Serial.print(" ");
   Serial.print(calibData.gyro_offset_y); Serial.print(" ");
   Serial.print(calibData.gyro_offset_z); Serial.print(" ");

   Serial.print("\nMag: ");
   Serial.print(calibData.mag_offset_x); Serial.print(" ");
   Serial.print(calibData.mag_offset_y); Serial.print(" ");
   Serial.print(calibData.mag_offset_z); Serial.print(" ");

   Serial.print("\nAccel Radius: ");
   Serial.print(calibData.accel_radius);

   Serial.print("\nMag Radius: ");
   Serial.print(calibData.mag_radius);
}


void setup(void)
{
   Wire.begin(I2C_SDA, I2C_SCL, 400000); // begin I2C with 400kHz
   Serial.begin(115200);
   delay(1000);
   Serial.println("Orientation Sensor Test"); Serial.println("");

   if (!bno.begin())
   {
       Serial.print("No BNO055 detected!");
       while (1);
   }

   Calibration.begin("offsets", false);
   // Calib. Data
   adafruit_bno055_offsets_t calibrationData;
   sensor_t sensor;

   int32_t bnoID = Calibration.getInt("bnoID");
    bool foundCalib = false;
   bno.getSensor(&sensor);
   if (bnoID != sensor.sensor_id)
   {
       Serial.println("\nNo Calibration Data for this sensor exists in NVS");
       delay(500);
   }
   else
   {
       Serial.println("\nFound Calibration for this sensor in NVS.");

       // Accelerometer offsets
       calibrationData.accel_offset_x = Calibration.getInt("acc_off_x");
       calibrationData.accel_offset_y = Calibration.getInt("acc_off_y");
       calibrationData.accel_offset_z = Calibration.getInt("acc_off_z");

       // Gyroscrope offsets
       calibrationData.gyro_offset_x = Calibration.getInt("gyr_off_x");
       calibrationData.gyro_offset_y = Calibration.getInt("gyr_off_y");
       calibrationData.gyro_offset_z = Calibration.getInt("gyr_off_z");

       // Magnetometer offsets
       calibrationData.mag_offset_x = Calibration.getInt("mag_off_x");
       calibrationData.mag_offset_y = Calibration.getInt("mag_off_y");
       calibrationData.mag_offset_z = Calibration.getInt("mag_off_z");

       displaySensorOffsets(calibrationData);

       Serial.println("\n\nRestoring Calibration data to the BNO055...");
       bno.setSensorOffsets(calibrationData);

       Serial.println("\n\nCalibration data loaded into BNO055");
       foundCalib = true;
   }

   delay(1000);

   bno.setExtCrystalUse(true);

   sensors_event_t event;
   bno.getEvent(&event);
   if (foundCalib){
       Serial.println("Move sensor slightly to calibrate magnetometers");
       while (!bno.isFullyCalibrated())
       {
           bno.getEvent(&event);
           displayCalStatus();
           Serial.println("");
           delay(BNO055_SAMPLERATE_DELAY_MS);
       }
   }
   else
   {
       Serial.println("Please Calibrate Sensor: ");
       while (!bno.isFullyCalibrated())
       {
           bno.getEvent(&event);

           Serial.print("X: ");
           Serial.print(event.orientation.x, 4);
           Serial.print("\tY: ");
           Serial.print(event.orientation.y, 4);
           Serial.print("\tZ: ");
           Serial.print(event.orientation.z, 4);

           displayCalStatus();

           Serial.println("");

           delay(BNO055_SAMPLERATE_DELAY_MS);
       }
   }

   Serial.println("\nFully calibrated!");
   Serial.println("--------------------------------");
   Serial.println("Calibration Results: ");
   adafruit_bno055_offsets_t newCalib;
   bno.getSensorOffsets(newCalib);
   displaySensorOffsets(newCalib);

   Serial.println("\n\nStoring calibration data to NVS...");

   bno.getSensor(&sensor);
   bnoID = sensor.sensor_id;
   Calibration.putInt("bnoID", bnoID);

   Calibration.putInt("acc_off_x", newCalib.accel_offset_x);
   Calibration.putInt("acc_off_y", newCalib.accel_offset_y);
   Calibration.putInt("acc_off_z", newCalib.accel_offset_z);

   Calibration.putInt("gyr_off_x", newCalib.gyro_offset_x);
   Calibration.putInt("gyr_off_y", newCalib.gyro_offset_y);
   Calibration.putInt("gyr_off_z", newCalib.gyro_offset_z);

   Calibration.putInt("mag_off_x", newCalib.mag_offset_x);
   Calibration.putInt("mag_off_y", newCalib.mag_offset_y);
   Calibration.putInt("mag_off_z", newCalib.mag_offset_z);
   Serial.println("Data stored to NVS..");

   Serial.println("\n--------------------------------\n");
   delay(500);
}

void loop() {
   sensors_event_t event;
   bno.getEvent(&event);

   Serial.print("X: ");
   Serial.print(event.orientation.x, 4);
   Serial.print("\tY: ");
   Serial.print(event.orientation.y, 4);
   Serial.print("\tZ: ");
   Serial.print(event.orientation.z, 4);

   displayCalStatus();

   Serial.println("");

   delay(BNO055_SAMPLERATE_DELAY_MS);
}