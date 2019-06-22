#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

/*
 * Sensor Config
 */
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MPU6050_tockn.h>
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10
Adafruit_BMP280 altimeter;
MPU6050 mpu6050(Wire);

#define MSG_DATA 1

#define PIEZO_PIN   36

/*
 * Datalogger 
 */
const int chipSelect = BUILTIN_SDCARD;
bool CAN_WRITE = true;
String file_name = "";
char* log_name;

/*
 * GPS Config
 */
#include <TinyGPS.h>
TinyGPS gps;
float latit = 0.0, longit = 0.0;

float starting_height = 0.0;
float ATM_PRESSURE = 102.0; // Pressure in hPa

void setup()
{
  delay(500);
  Serial.begin(9600);
  Serial1.begin(9600); //Xbee
  Serial2.begin(9600); //GPS
  if (!altimeter.begin(0x76)) {
       Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        write_to_disk("DEBUG: Alti Init Failed");
  } else {
    Serial.println(F("Altimeter Initialised"));
    delay(500);
    starting_height = altimeter.readAltitude(ATM_PRESSURE);
    Serial.print("Starting Height: ");
    Serial.println(starting_height);
  }
  delay(200);
  
  Serial.print(F("Initializing SD card..."));
  
  // See if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    write_to_disk("DEBUG: SD Init Failed");
    CAN_WRITE = false;
  } else {
    Serial.println("SD Card Initialized");
  }
  // Check the accelerometer is working
  delay(300);
  mpu6050.begin();
  delay(200);
  mpu6050.update();
  if (sqrtf(pow(mpu6050.getAccX(), 2) + pow(mpu6050.getAccY(), 2) + pow(mpu6050.getAccZ(), 2)) > 0.5){
    Serial.println(F("Initialised Accelerometer"));
    Serial1.println(F("Initialised Accelerometer"));
  } else {
    Serial.println(F("Accelerometer Initialisation Failed"));
    Serial1.println(F("Accelerometer Initialisation Failed"));
    write_to_disk("DEBUG: Accel Init Failed");
  }
  mpu6050.calcGyroOffsets(true);
  delay(100);
  Serial.println(F("Accelerometer and Gyro Initialised"));

  // Sound Buzzer 10 times in a second to confirm continuity, skies are clear, range is clear 5,4,3,2,1 LAUNCH.
  pinMode(PIEZO_PIN, OUTPUT);
  for (int i = 0; i < 10; i++) {
    tone(PIEZO_PIN, 1000);
    delay(100);
    noTone(PIEZO_PIN);
  }
  
}

/**
 * Takes GPS data to find the current longitude and latitude
 */
String get_lat_long() {
  String lat_long = String(latit, 6) + String(',')+ String(longit, 6);
  return lat_long;
}

/*
 * Polling and Sampling code.
 */
#define POLL_RATE     400
#define SAMPLES       7
#define SAMPLE_RATE   POLL_RATE/SAMPLES

unsigned long INTERNAL_TIMER = 0;
unsigned long POLL_TIME = 0;
bool has_sent = true;
int count = 0;
float avg_alti = 0.0;     //Sampled and averaged altitude
float avg_accel_x = 0.0;  //Sampled and averaged accel in x direction
float avg_accel_y = 0.0;  //Sampled and averaged accel in y direction
float avg_accel_z = 0.0;  //Sampled and averaged accel in z direction

void reset_avgs() {
  count = 0;
  avg_alti = 0.0;
  avg_accel_x = 0.0;
  avg_accel_y = 0.0;
  avg_accel_z = 0.0;
}


// Write to the SD Card
void write_to_disk(const char* data_string) {
  if (file_name.length() == 0) {
    randomSeed(analogRead(0));
    delay(random(0, 999));
    file_name += String(random(0, 9999)) + ".txt";
    log_name = (char*)file_name.c_str();
    Serial.println(log_name);
    Serial1.print("File Name: ");
    Serial1.println(log_name);
  }
  File dataFile = SD.open(log_name, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(data_string);
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening file on SD");
  }   
}


//Main Loop
void loop()
{
  //GPS Vars
  String lat_long_str = String(latit, 6) + String(',')+ String(longit, 6);
  
  mpu6050.update(); //Acceleration Update
  if (POLL_TIME >= millis()) {
      //Sample Condition - if true take a sample of the measurements available and add them to the average.
      if (INTERNAL_TIMER <= millis()) { //We need to sample in order to stop the averages being too skewed from sensor refresh rates being too low.
        INTERNAL_TIMER = millis() + SAMPLE_RATE;
        avg_alti += altimeter.readAltitude(ATM_PRESSURE) - starting_height; //Altitude is always current height minus the starting height
        avg_accel_x += mpu6050.getAccX();
        avg_accel_y += mpu6050.getAccY();
        avg_accel_z += mpu6050.getAccZ();
        count++;
        
      }
  } else {
    POLL_TIME = millis() + POLL_RATE;  //Update the polling timer to the next POLL time
    INTERNAL_TIMER = 0; // This should trigger a sample as soon as we hit the sample condition
    char data[200]; //specify the send buffer used by the Xbee;
    const char * data_string = data;

    //Take instantaneous data if the averages are not available for some reason.
    if (count > 0) { 
      avg_alti /= (float)count;
      avg_accel_x /= (float)count;
      avg_accel_y /= (float)count;
      avg_accel_z /= (float)count;
    } else {
      Serial1.println("COUNT IS 0 FORCED TO TAKE INSTANTANEOUS SAMPLE");
      avg_alti = altimeter.readAltitude(ATM_PRESSURE) - starting_height;
      avg_accel_x = mpu6050.getAccX();
      avg_accel_y = mpu6050.getAccY();
      avg_accel_z = mpu6050.getAccZ();
    }

    //Get the magnitude of the acceleration and print to 3 dp.
    String acceleration = String(sqrtf(pow(avg_accel_x, 2) + pow(avg_accel_y, 2) + pow(avg_accel_z, 2)), 3);
    //Write into the send buffer;
    sprintf(data, "%lu,%i,%s,%s,%s,%s,%s", millis(), MSG_DATA, String(avg_alti, 3).c_str(), acceleration.c_str(), String(altimeter.readPressure()/1000.0f, 3).c_str(), String(altimeter.readTemperature()).c_str(), lat_long_str.c_str());
    write_to_coms(data_string); //Write the the COMS serial port i.e. Xbee AND function also writes whatever went via the serial port to disk.

    //Check if we have an SD card to write too, if we do then we need to write to SD the gyroscope data.
    if (CAN_WRITE) {
        char data[50];
        sprintf(data, "GYRO: %lu,%i,%s,%s,%s", millis(), MSG_DATA, String(mpu6050.getGyroX(), 3).c_str(), String(mpu6050.getGyroY(), 3).c_str(), String(mpu6050.getGyroZ(), 3).c_str());
        write_to_disk(data);
      } //Save to MicroSD if there is one
    
    //Remember to RESET the averages otherwise you'll have too many samples and it will affect the recordings
    reset_avgs();
  }
}

void write_to_coms(const char* event_data) {
    Serial1.println(event_data);
    Serial1.flush();
    write_to_disk(event_data);
}

//Interrup Driven GPS I/O
void serialEvent2() {
    boolean newData = false;
    if (Serial2.available()) {
      while (Serial2.available()) {
        //char c = ;
         //Serial.print(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(Serial2.read())) { // Did a new valid sentence come in?
          newData = true;
        }
      }
    }
    if (newData) {
      float flat, flon;
      unsigned long age;
      //Get the data from the GPS.
      gps.f_get_position(&flat, &flon, &age);
      //Update the global variables holding the last known longitude and latitude.
      latit = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
      longit = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
      Serial.print("POS=");
      Serial.println(get_lat_long());
    }
}
