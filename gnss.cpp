#include "delay.h"
#include "main.h"

gnssData_t gnssData[MAX_SAMPLES];
char utc_time[12], latitude[12], longitude[12];
double lat_decimal = 0.0, long_decimal = 0.0;
String command;  // String to store BG77 commnads.
/*******************************************/
/*              GPS                        */
/*******************************************/
void GSM_wakeup();
void GSM_Sleep();

void convert_to_decimal_degrees(char *coord, double *result);
void parseGPSdata(char Gps_data[]);

// Read the return value of BG77.
void BG77_read(time_t timeout, char Gps_buffer[],int size) {
  memset(Gps_buffer, '\0', size);
  int count = 0;
  time_t timeStamp = millis();
  while ((millis() - timeStamp) < timeout) {
    if (Serial1.available() > 0) {
      Gps_buffer[count] = (char)Serial1.read();
      count++;
      delay(1);
    }
  }
  // Serial.print(Gps_buffer);
  Serial.println("");
}

// Write commnads to BG77.
void BG77_write(const char* command) {
  while (*command) {
    Serial1.write(*command);
    command++;
  }
  Serial1.println();
}

void convert_to_decimal_degrees(char *coord, double *result) {
    // Parse the degrees and minutes from the coordinate
    int degrees = 0;
    double minutes = 0.0;
    int len = strlen(coord);
    char direction = coord[len-1];

    // Determine if it's latitude or longitude by checking the length
    if (len == 10) {
        // Latitude format: DDMM.MMMM
        char degrees_str[3] = {coord[0], coord[1], '\0'};  // First 2 characters
        char minutes_str[8];
        strncpy(minutes_str, coord + 2, 7);  // Rest for minutes, up to decimal precision
        minutes_str[7] = '\0';

        degrees = atoi(degrees_str);
        minutes = atof(minutes_str);
    } else if (len == 11) {
        // Longitude format: DDDMM.MMMM
        char degrees_str[4] = {coord[0], coord[1], coord[2], '\0'};  // First 3 characters
        char minutes_str[8];
        strncpy(minutes_str, coord + 3, 7);  // Rest for minutes, up to decimal precision
        minutes_str[7] = '\0';

        degrees = atoi(degrees_str);
        minutes = atof(minutes_str);
    } else {
        printf("Unexpected coordinate format\n");
        return;
    }
    // Convert to decimal degrees
    *result = degrees + (minutes / 60.0);

    // Apply direction (N, E -> positive; S, W -> negative)
    if (direction == 'S' || direction == 'W') {
        *result = -*result;
    }
}

void parseGPSdata(char Gps_data[]) {
  static int index = 0;
  char* token = strtok(Gps_data, ":,");
  token = strtok(NULL, " , ");
  strcpy(utc_time, token);

  token = strtok(NULL, " , ");
  strcpy(latitude, token);

  token = strtok(NULL, " , ");
  strcpy(longitude, token);

  convert_to_decimal_degrees(latitude, &lat_decimal);
  convert_to_decimal_degrees(longitude, &long_decimal);

  gnssData[index % MAX_SAMPLES].timestamp = millis();
  gnssData[index % MAX_SAMPLES].latitude = lat_decimal;
  gnssData[index % MAX_SAMPLES].longitude = long_decimal;
  index++;
  Serial.printf("Latitude : %lf\n\r", lat_decimal);
  Serial.printf("Longitude : %lf\n\r", long_decimal);
}

void GSM_wakeup() {
  pinMode(BG77_POWER_KEY, OUTPUT);
  digitalWrite(BG77_POWER_KEY, 0);
  delay(1000);
  digitalWrite(BG77_POWER_KEY, 1);
  delay(2000);
  digitalWrite(BG77_POWER_KEY, 0);
  delay(1000);
}

void GSM_Sleep()
{
  pinMode(BG77_POWER_KEY, OUTPUT);
  digitalWrite(BG77_POWER_KEY, 0);
  delay(2000);
  digitalWrite(BG77_POWER_KEY, 1);
  delay(1000);
  digitalWrite(BG77_POWER_KEY, 0);
  delay(2000);
}

void BG77_init() {

  time_t timeout = millis();
  bool moduleSleeps = true;

  Serial1.println("ATI");
  //BG77 init
  while ((millis() - timeout) < 4000) {
    if (Serial1.available()) {
      String result = Serial1.readString();
      // Serial.println("GNSS Modem Initialized...");
      moduleSleeps = false;
    }
  }
  if (moduleSleeps) {
    // Module slept, wake it up
    GSM_wakeup();
  }
  delay(1000);

}


void Get_GNSSData() {
  bool loc_done = false;
  if(BG77_status != GSM_RUNNING){
    BG77_status = GNSS_RUNNING;
    BG77_init();

    char Gps_buffer[512];

    // Turn on GNSS
    command = "AT+QGPS=1\r";
    BG77_write(command.c_str());
    BG77_read(2000, Gps_buffer,512);
    delay(100);

    command = "AT+QGPSLOC=1\r";
    BG77_write(command.c_str());
    BG77_read(1000, Gps_buffer,512);

    // Acquire Positioning Information
    command = "AT+QGPSLOC?\r";
    BG77_write(command.c_str());
    BG77_read(1000, Gps_buffer,512);
    if (strstr(Gps_buffer, "OK") != 0){
      parseGPSdata(Gps_buffer);
      loc_done = true;
    }
    else{
      Serial.println("Trying to get location....");
      loc_done = false;
    }

    // Acquire NMEA Sentences with GSV type
    command = "AT+QGPSGNMEA=\"GSV\"\r";
    BG77_write(command.c_str());
    BG77_read(1000, Gps_buffer,512);
  }
  if(loc_done){
      // command = "AT+QPOWD\r";
      command = "AT+QGPSEND\r";
      BG77_write(command.c_str());
      BG77_status = BG77_IDLE;
    delay(getTimerInterval_ms(g_config_settings.gpsLocationFrequency ));
    // delay(MINUTES * 5);
  }
  else{
    delay(1000 * 15);
  }
}