#include "variant.h"
#include "WString.h"
#include "Adafruit_USBD_CDC.h"
#include "avr/pgmspace.h"
#include "main.h"

int numWasteSensors = 3;
int numDoorSensors = 3;
BG77_status_t BG77_status = BG77_IDLE;

// Local variables to store JSON data
int configVersion;
uint8_t mac[6];
uint8_t doorSensorMacs[3][6];  // Max 3 entries of 6 bytes each
uint8_t wasteSensorMacs[3][6]; // Max 3 entries of 6 bytes each

int reportingFrequency;
int gpsLocationFrequency;
int wasteLevelFrequency;
int unitOrientationFrequency;
int batteryLevelFrequency;


// JSON data (as a raw string)
const char* jsonData = R"(
{
  "configVersion": 1,
  "unitId": "2f003d000447343339383138",
  "mac": "3C-CD-D7-87-32-FD",
  "sensors": [
    {
        "doorSensorMac": ["51-E6-32-00-00-C3", "52-E6-32-00-00-C3","53-E6-32-00-00-C3"],
        "wasteSensorMac": ["84-C1-35-35-3E-F9", "8B-8B-A5-CB-B0-C5", "AA-BB-CC-DD-EE-FF"]
    }
  ],
  "reportingFrequency": 4,
  "captureFrequency": {
    "gpsLocation": 96,
    "wasteLevel": 96,
    "unitOrientation": 96,
    "batteryLevel": 96
  }
}
)";


void parseMacAddress(const char* macStr, uint8_t* macArray) {
  unsigned int temp[6];
  sscanf(macStr, "%02X-%02X-%02X-%02X-%02X-%02X", &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
  for (int i = 0; i < 6; i++) {
    macArray[i] = static_cast<uint8_t>(temp[i]);
  }
}

void updateConfigParams()
{
  // Define JSON document with an estimated size
  const size_t capacity = JSON_OBJECT_SIZE(5) + JSON_ARRAY_SIZE(1) + JSON_ARRAY_SIZE(3) * 2 + 400;
  DynamicJsonDocument doc(capacity);
  // Parse JSON data
  DeserializationError error = deserializeJson(doc, jsonData);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  // Extract and store values in local variables
  g_config_settings.configVersion = doc["configVersion"];
  
  parseMacAddress(doc["mac"], mac);  // Parse and convert main MAC
  memcpy_P(g_config_settings.hub_mac, mac, 6);


  // Extract door sensor MAC addresses dynamically
  JsonArray doorSensorMacArray = doc["sensors"][0]["doorSensorMac"];
  numDoorSensors = min(doorSensorMacArray.size(), 3); // Limit to max 3
  for (int i = 0; i < numDoorSensors; i++) {
    parseMacAddress(doorSensorMacArray[i], doorSensorMacs[i]);
    memcpy(g_config_settings.doorSensorMacs[i], doorSensorMacs[i], 6);
  }

  // Extract waste sensor MAC addresses dynamically
  JsonArray wasteSensorMacArray = doc["sensors"][0]["wasteSensorMac"];
  numWasteSensors = min(wasteSensorMacArray.size(), 3); // Limit to max 3
  for (int i = 0; i < numWasteSensors; i++) {
    parseMacAddress(wasteSensorMacArray[i], wasteSensorMacs[i]);
    memcpy(g_config_settings.wasteSensorMacs[i], wasteSensorMacs[i], 6);
  }

  // Extract frequencies
  g_config_settings.reporting_frequency       = doc["reportingFrequency"];
  g_config_settings.gpsLocationFrequency      = doc["captureFrequency"]["gpsLocation"];
  g_config_settings.wasteLevelFrequency       = doc["captureFrequency"]["wasteLevel"];
  g_config_settings.unitOrientationFrequency  = doc["captureFrequency"]["unitOrientation"];
  g_config_settings.batteryLevelFrequency     = doc["captureFrequency"]["batteryLevel"];

  save_settings();
  
}

void byteArrayToString(uint8_t *hexArray, size_t length, char *str) {
    for (size_t i = 0; i < length; i++) {
        sprintf(&str[i * 3], "%02X:", hexArray[i]);  // Format each byte as 2 hex characters
    }
    str[(length * 3)-1] = '\0';  // Null-terminate the string
}

void createJson()
{
  BG77_status = GSM_RUNNING;
  DynamicJsonDocument doc(700*MAX_SAMPLES);

  // Add accelerometer data
  JsonArray accelArray = doc.createNestedArray("Hub Data");

  // Add Door data
  JsonArray doordataArray = doc.createNestedArray("Door Sensor");
  // Add Radar data
  JsonArray radardataArray = doc.createNestedArray("Radar Sensor");
  // Add GNSS data
  JsonArray gnssArray = doc.createNestedArray("GNSS");


  for (int j = 0; j < MAX_SAMPLES; j++) {
    if (hubdata[j].timestamp != 0) 
    { 
      JsonObject h_dataPoint = accelArray.createNestedObject();
      h_dataPoint["hubBatteryLevel"] = hubdata[j].batteryLevel;
      h_dataPoint["unitOrientation"] = hubdata[j].orientation;
    }


    char mac[18];
    if (doorSenData[j].timestamp != 0) 
    {                                      // Ensure we skip uninitialized data
      JsonObject d_dataPoint = doordataArray.createNestedObject();
      byteArrayToString(doorSenData[j].MAC, sizeof(doorSenData[j].MAC), mac);
      d_dataPoint["timestamp"] = doorSenData[j].timestamp;
      d_dataPoint["type"]  = "S4";
      d_dataPoint["MAC"]  = mac;
      d_dataPoint["rssi"]  = doorSenData[j].rssi;
      d_dataPoint["doorUsage"]  = doorSenData[j].doorcount;
      d_dataPoint["batteryLevel"]  = doorSenData[j].batteryLevel;
    }
    memset(mac,'\0',18);
    if (RadarSenData[j].timestamp != 0) {  // Ensure we skip uninitialized data
      JsonObject r_dataPoint = radardataArray.createNestedObject();
      byteArrayToString(RadarSenData[j].MAC, sizeof(RadarSenData[j].MAC), mac);
      r_dataPoint["timestamp"] = RadarSenData[j].timestamp;
      r_dataPoint["type"]  = "XM122";
      r_dataPoint["MAC"]  = mac;
      r_dataPoint["rssi"]  = RadarSenData[j].rssi;
      r_dataPoint["distance"]  = RadarSenData[j].distance;
      r_dataPoint["batteryLevel"]  = RadarSenData[j].batteryLevel;
    }
    if (gnssData[j].timestamp != 0) {
      JsonObject g_dataPoint = gnssArray.createNestedObject();
      g_dataPoint["timestamp"] = gnssData[j].timestamp;
      g_dataPoint["latitude"] = gnssData[j].latitude;  // Use GNSS latitude for simplicity
      g_dataPoint["longitude"] = gnssData[j].longitude;  // Use GNSS latitude for simplicity
    }
  }

  size_t jsonLen = measureJson(doc);
  // Print the length
  Serial.print("JSON length: ");
  Serial.println(jsonLen);


  String jsonStr;
  serializeJson(doc, jsonStr);
  Serial.print("JSON string length: ");
  Serial.println(jsonStr.length());

  Serial.println(jsonStr);


  memset((void*)doorSenData,0,sizeof(doorSenData));
  memset((void*)RadarSenData,0,sizeof(RadarSenData));
  memset(&hubdata,0,sizeof(hubdata));
  memset((void*)gnssData,0,sizeof(gnssData));
  BG77_status = BG77_IDLE;
}
