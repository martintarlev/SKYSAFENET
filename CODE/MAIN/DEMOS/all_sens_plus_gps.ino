#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>
#include <BohleBots_BNO055.h>

static const int RXPin = 5, TXPin = 6;
static const uint32_t GPSBaud = 115200;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

#define I2C_SDA 8
#define I2C_SCL 9
#define BNO055_SAMPLERATE_DELAY_MS (1000)

Adafruit_BNO055 bno;
Adafruit_BME280 bme;
BNO bohleBNO;
Preferences Calibration;

#define SEALEVELPRESSURE_HPA (1013.25)

float rho = 1.204; // density of air

int veloc_mean_size = 20;
int zero_span = 10;
float DifPresure = 0;

void setup()
{
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Serial.begin(115200);
  ss.begin(GPSBaud);
  pinMode(4, INPUT);

  Serial.println("Sensor Test");
  Serial.println("");

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address!");
    while (1);
  }
  Serial.println("BME280 initialized!");

  if (!bno.begin())
  {
    Serial.print("No BNO055 detected!");
    while (1);
  }
  performCalibration();
  delay(1000);
  initializeBNO();
  Serial.println("Fully Calibrated!");

  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt       Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age  Age             --- from GPS ----    RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop()
{
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  readBME280Data();
  readVelocity();
  readBNOHeading();

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

void performCalibration()
{
  Calibration.begin("offsets", false);
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

    calibrationData.accel_offset_x = Calibration.getInt("acc_off_x");
    calibrationData.accel_offset_y = Calibration.getInt("acc_off_y");
    calibrationData.accel_offset_z = Calibration.getInt("acc_off_z");

    calibrationData.gyro_offset_x = Calibration.getInt("gyr_off_x");
    calibrationData.gyro_offset_y = Calibration.getInt("gyr_off_y");
    calibrationData.gyro_offset_z = Calibration.getInt("gyr_off_z");

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
  if (foundCalib)
  {
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
}

void initializeBNO()
{
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to NVS...");

  int32_t bnoID = getBNOID();
  Calibration.putInt("bnoID", bnoID);

  storeCalibrationData(newCalib);
  Serial.println("Data stored to NVS..");

  Serial.println("\n--------------------------------\n");
  delay(500);
}

int32_t getBNOID()
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  return sensor.sensor_id;
}

void storeCalibrationData(const adafruit_bno055_offsets_t &calibData)
{
  Calibration.putInt("acc_off_x", calibData.accel_offset_x);
  Calibration.putInt("acc_off_y", calibData.accel_offset_y);
  Calibration.putInt("acc_off_z", calibData.accel_offset_z);

  Calibration.putInt("gyr_off_x", calibData.gyro_offset_x);
  Calibration.putInt("gyr_off_y", calibData.gyro_offset_y);
  Calibration.putInt("gyr_off_z", calibData.gyro_offset_z);

  Calibration.putInt("mag_off_x", calibData.mag_offset_x);
  Calibration.putInt("mag_off_y", calibData.mag_offset_y);
  Calibration.putInt("mag_off_z", calibData.mag_offset_z);
}

void displayCalStatus()
{
  uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("\t");
    if (!system) {
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
    Serial.print(calibData.accel_offset_x);
    Serial.print(" ");
    Serial.print(calibData.accel_offset_y);
    Serial.print(" ");
    Serial.print(calibData.accel_offset_z);
    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x);
    Serial.print(" ");
    Serial.print(calibData.gyro_offset_y);
    Serial.print(" ");
    Serial.print(calibData.gyro_offset_z);
    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x);
    Serial.print(" ");
    Serial.print(calibData.mag_offset_y);
    Serial.print(" ");
    Serial.print(calibData.mag_offset_z);
    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);
    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void readBME280Data()
{
  Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    Serial.println();
}

void readVelocity()
{
  float adc_avg = 0;
    float veloc = 0.0;
  
    for (int ii = 0; ii < veloc_mean_size; ii++) {
        adc_avg += analogRead(4);
    }
    adc_avg /= veloc_mean_size;

    if (adc_avg < 3125 - zero_span) {
        DifPresure = ((((adc_avg * 0.0008) / 5) - 0.5) / 0.2) * -1;
        veloc = sqrt(2 * (DifPresure * 1000) / rho);
    } else {
        Serial.println("ERROR");
    }
    Serial.print("Velocity: ");
    Serial.println(veloc); // print velocity
    delay(10); // delay for stability
}

void readBNOHeading()
{
  Serial.print("Heading: ");
    Serial.println((bohleBNO.getHeadingAuto(100) + 90) % 360, DEC); // Give out the current heading with adjustment
    delay(1000);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}