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

#include "FlightHandler.h"

#define CMD_DELIM ';'

// Ground State
boolean STARTUP = false;
boolean IN_DIAG = false;
boolean GPS_LOCK = false;
boolean STATS_SENT = false;
boolean STATS_REQUEST = false;

String data_buffer[4];
String command;

FlightLogger *avionics_log = new FlightLogger(false); // WILL NEED A REAL PIN
Status *stat = new Status(RK_OFF, RK_ON);
Calculations *calc = new Calculations(5, 2, stat->get_current_status());
FlightHandler flight_comp(stat, calc, avionics_log, 10000);

/*FlightLogger *diag_logger = new FlightLogger(true);
Status *diag_status = new Status(RK_OFF, RK_ON);
Calculations *diag_calc = new Calculations(6, 7, diag_status->get_current_status());
FlightHandler diag_flight_comp(diag_status, diag_calc, diag_logger, 10000);*/

void setup()
{
  delay(2000);
  Serial.begin(9600);
  Serial1.begin(9600); //Xbee
  Serial2.begin(9600); //GPS
	stat->set_status(RK_STARTUP);
  if (!altimeter.begin(0x76)) {
       Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        write_to_disk("DEBUG: Alti Init Failed", MSG_DATA);
  } else {
    Serial.println(F("Altimeter Initialised"));
    delay(3000);
    flight_comp.starting_height = altimeter.readAltitude(ATM_PRESSURE);
    Serial.print("Starting Height: ");
    Serial.println(flight_comp.starting_height);
  }
  delay(500);
  
  Serial.print(F("Initializing SD card..."));
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    write_to_disk("DEBUG: SD Init Failed", MSG_DATA);
    CAN_WRITE = false;
    // don't do anything more:
  } else {
    Serial.println("SD Card Initialized");
  }
  
  delay(800);
  mpu6050.begin();
  delay(400);
  mpu6050.update();
  if (sqrtf(pow(mpu6050.getAccX(), 2) + pow(mpu6050.getAccY(), 2) + pow(mpu6050.getAccZ(), 2)) > 0.5){
    Serial.println(F("Initialised Accelerometer"));
    Serial1.println(F("Initialised Accelerometer"));
  } else {
    Serial.println(F("Accelerometer Initialisation Failed"));
    Serial1.println(F("Accelerometer Initialisation Failed"));
    write_to_disk("DEBUG: Accel Init Failed", MSG_DATA);
  }
  mpu6050.calcGyroOffsets(true);
  delay(500);
  Serial.println(F("Accelerometer and Gyro Initialised"));
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

#define POLL_RATE     1000
#define SAMPLES       25
#define SAMPLE_RATE   POLL_RATE/SAMPLES

unsigned long INTERNAL_TIMER = 0;
unsigned long POLL_TIME = 0;
bool has_sent = true;
int count = 0;
float avg_alti = 0.0;
float avg_accel_x = 0.0;
float avg_accel_y = 0.0;
float avg_accel_z = 0.0;

void reset_avgs() {
  count = 0;
  avg_alti = 0.0;
  avg_accel_x = 0.0;
  avg_accel_y = 0.0;
  avg_accel_z = 0.0;
}

void write_to_disk(const char* data_string, int msg_type) {
  char send_buffer[300];
  sprintf(send_buffer, "%lu,%i,%i,%i,%s", (uint32_t)millis(), msg_type, stat->get_current_sequence(), stat->get_current_status(), data_string);
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
    dataFile.print(send_buffer);
    dataFile.println(';');
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening file on SD");
  }   
}

void loop()
{
  //GPS Vars
  const char* lat_long_str = get_lat_long().c_str();
  
  //Acceleration Update
  mpu6050.update();
  
	if (STARTUP && flight_comp.status->get_current_sequence() >= RK_SEQ_LAUNCH) {
    if (POLL_TIME >= millis()) {
        //Sample Condition - if true take a sample of the measurements available and add them to the average.
        if (INTERNAL_TIMER <= millis()) { //We need to sample in order to stop the averages being too skewed from sensor refresh rates being too low.
          INTERNAL_TIMER = millis() + SAMPLE_RATE;
          avg_alti += altimeter.readAltitude(ATM_PRESSURE) - flight_comp.starting_height;
          avg_accel_x += mpu6050.getAccX();
          avg_accel_y += mpu6050.getAccY();
          avg_accel_z += mpu6050.getAccZ();
          count++;
          
        }
    } else {
      POLL_TIME = millis() + POLL_RATE;
      INTERNAL_TIMER = 0; // This should trigger a sample as soon as we hit the sample condition
      char data[200];
      const char * data_string = data;

      //Take instantaneous data if the averages are not available for some reason.
      if (count > 0) { 
        avg_alti /= (float)count;
        avg_accel_x /= (float)count;
        avg_accel_y /= (float)count;
        avg_accel_z /= (float)count;
      } else {
        Serial.println("COUNT IS 0 FORCED TO TAKE INSTANTANEOUS SAMPLE");
        Serial1.println("COUNT IS 0 FORCED TO TAKE INSTANTANEOUS SAMPLE");
        avg_alti = altimeter.readAltitude(ATM_PRESSURE) - flight_comp.starting_height;
        avg_accel_x = mpu6050.getAccX();
        avg_accel_y = mpu6050.getAccY();
        avg_accel_z = mpu6050.getAccZ();
      }
      
      flight_comp.update(avg_alti, avg_accel_x, avg_accel_y, avg_accel_z);
      String acceleration = String(flight_comp.calculator->get_curr_acceleration(), 3);
      float curr_velo = flight_comp.calculator->get_curr_velocity();
      sprintf(data, "%s,%s,%s,%s,%s,%s", String(avg_alti, 3).c_str(), acceleration.c_str(), String(curr_velo).c_str(), String(altimeter.readPressure()/1000.0f, 3).c_str(), String(altimeter.readTemperature()).c_str(), lat_long_str);
      avionics_log->write_to_coms(millis(), MSG_DATA, stat->get_current_sequence(), stat->get_current_status(), data_string);
      
      if (CAN_WRITE) { 
        write_to_disk(data, MSG_DATA); 
          char data[50];
          sprintf(data, "GYRO: %s,%s,%s,%s,%s,%s", String(mpu6050.getGyroX(), 3).c_str(), String(mpu6050.getGyroY(), 3).c_str(), String(mpu6050.getGyroZ(), 3).c_str());
          write_to_disk(data, MSG_DATA);
        } //Save to MicroSD if there is one
      
      if (flight_comp.status->get_current_sequence() >= RK_SEQ_RECOVERY || STATS_REQUEST) {
        STATS_REQUEST = false;
      
        char data[300];
        const char * data_string = data;
        float max_alti = flight_comp.calculator->max_alti;
        float min_alti = flight_comp.calculator->min_alti;
        float max_accel = flight_comp.calculator->max_accel;
        float min_accel = flight_comp.calculator->min_accel;
        float max_v = flight_comp.calculator->max_v;
        float min_v = flight_comp.calculator->min_v;
        float max_delta_alti = flight_comp.calculator->max_delta_alti;
        float min_delta_alti = flight_comp.calculator->min_delta_alti;
        float max_delta_a = flight_comp.calculator->max_delta_a;
        float min_delta_a = flight_comp.calculator->min_delta_a;
        float max_delta_v = flight_comp.calculator->max_delta_v;
        float min_delta_v = flight_comp.calculator->min_delta_v;
        sprintf(data, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", String(max_alti).c_str(), String(min_alti).c_str(), String(max_accel).c_str(), String(min_accel).c_str(), String(max_v).c_str(), String(min_v).c_str(), String(max_delta_alti).c_str(), String(min_delta_alti).c_str(), String(max_delta_a).c_str(), String(min_delta_a).c_str(), String(max_delta_v).c_str(), String(min_delta_v).c_str());
        avionics_log->write_to_coms(millis(), MSG_STATS, stat->get_current_sequence(), stat->get_current_status(), data_string);
        
        if (CAN_WRITE) write_to_disk(data, MSG_STATS); //Save to MicroSD if there is one
      }
      
      //Remember to RESET the averages otherwise you'll have too many samples and it will affect the recordings
      reset_avgs();
    }
   
		/*if (flight_comp.status->get_current_sequence() >= RK_SEQ_LAUNCH && flight_comp.status->get_current_sequence() < RK_SEQ_RECOVERY) {
		}*/
	} else {
    delay(1000);
    
    char data[200];
    const char * data_string = data;
    String acceleration = String(sqrtf(powf(mpu6050.getAccX(),2)+powf(mpu6050.getAccY(),2)+powf(mpu6050.getAccZ(),2)), 6);
    
    sprintf(data, "%s,%s,%s,%s,%s,%s", String(altimeter.readAltitude(ATM_PRESSURE) - flight_comp.starting_height).c_str(), acceleration.c_str(), String(0.0).c_str(), String(altimeter.readPressure()/1000.0f).c_str(), String(altimeter.readTemperature()).c_str(), lat_long_str);
		avionics_log->write_to_coms(millis(), MSG_DATA, stat->get_current_sequence(), stat->get_current_status(), data_string);
    
    if (CAN_WRITE) write_to_disk(data, MSG_DATA); //Save to MicroSD if there is one
  }
}

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
      gps.f_get_position(&flat, &flon, &age);
      latit = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
      longit = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
      Serial.print("POS=");
      Serial.println(get_lat_long());
    }
}

void serialEvent1() {
	while (Serial1.available()) {
		command = Serial1.readStringUntil(CMD_DELIM);
		command += ',';
		int count = 0;
		int start_index = 0;
		for (int i = 0; i < command.length(); i++) {
			if (i != start_index && command.charAt(i) == ',') {
				data_buffer[count++] = command.substring(start_index, i);
				start_index = i + 1;
				i++;
			}
		}
		int command_code = data_buffer[0].toInt();
		switch (command_code) {
			case GND_STARTUP:
				STARTUP = true;
				avionics_log->write_to_log(millis(), "CONNECTION TO GROUND");
				break;
			case GND_CONFIRM_READY:
				flight_comp.set_gnd_ready_flag();
				avionics_log->write_to_log(millis(), "CONFIRM READY");
				break;
			case GND_FORCE_READY:
				flight_comp.set_status(RK_READY);
				avionics_log->write_to_log(millis(), "FORCE READY STATE");
        POLL_TIME = millis() + POLL_RATE;
				break;
			case GND_START_DIAG:
				IN_DIAG = true;
				//diag_flight_comp.reset();
				//diag_flight_comp.calculator->in_diagnostics = true;
				flight_comp.set_status(RK_DIAG_START);
				//avionics_log->write_to_log(millis(), "START DIAGNOSTICS", stat->get_current_sequence(), stat->get_current_status());
				Serial1.flush();
				//diag_logger->write_to_log(millis(), "START");
				break;
			case GND_DIAG_DATA:
				if (IN_DIAG) {
					//diag_flight_comp.handle_diagnostic_data(data_buffer);
				}
				break;
			case GND_STOP_DIAG:
				IN_DIAG = false;
				//diag_flight_comp.calculator->in_diagnostics = false;
				flight_comp.set_status(RK_DIAG_FINISH);
				avionics_log->write_to_log(millis(), "STOP DIAGNOSTICS");
				break;
			case GND_DIAG_ERROR:
				IN_DIAG = false;
				//diag_flight_comp.calculator->in_diagnostics = false;
				flight_comp.set_status(RK_DIAG_ERROR);
				avionics_log->write_to_log(millis(), "DIAGNOSTICS ERROR");
				break;
			case GND_APOGEE_OVERRIDE:
				Serial1.println("OVERRIDE");
				flight_comp.set_status(RK_APOGEE);
				avionics_log->write_to_log(millis(), "FORCE APOGEE STATE");
				break;
			case GND_DROG_DEPLOY:
				flight_comp.set_status(RK_DROG_DEPLOY_PRM);
				avionics_log->write_to_log(millis(), "FORCE DROUGE DEPLOY STATE");
				break;
			case GND_MAIN_DEPLOY:
				flight_comp.set_status(RK_MAIN_DEPLOY_PRM);
				avionics_log->write_to_log(millis(), "FORCE MAIN DEPLOY STATE");
				break;
      case GND_STATS_REQUEST:
        STATS_REQUEST = true;
        break;
      case STOP_DATA_WRITE:
        CAN_WRITE = false;
        break;
		}
		for (int i = 0; i < 4; i++) {
			if (data_buffer[i].length() > 0) {
				//Serial.println(data_buffer[i]);
				data_buffer[i] = String(); //Reset Buffer
			}
		}
	}
}

