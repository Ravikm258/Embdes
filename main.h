#ifndef __MAIN_H
#define __MAIN_H

#include <Arduino.h>
#include <ArduinoJson.h>

#define PRINT_LOG   1

#define   MINUTES   1000*60
uint32_t getTimerInterval_ms(int frequency) ;
// BLE
#include <bluefruit.h>
#define MAX_CONN 4

const uint8_t RADARSERVICE[] = 
{
  0x7d, 0xbb, 0x2a, 0x85, 0x7a, 0x78, 0xb3, 0xab,
  0x22, 0x4d, 0x2b, 0x05, 0x01, 0x00, 0xc0, 0xac
};
const uint8_t RADARCHAR[] = 
{  
  0x7D, 0xBB, 0x2A, 0x85, 0x7A, 0x78, 0xB3, 0xAB,
  0x22, 0x4D, 0x2B, 0x05, 0x05, 0x00, 0xC0, 0xAC   
};

const uint8_t RADARRES[] = 
{  
  0x7D, 0xBB, 0x2A, 0x85, 0x7A, 0x78, 0xB3, 0xAB,
  0x22, 0x4D, 0x2B, 0x05, 0x04, 0x00, 0xC0, 0xAC   
};

typedef struct
{
  char      name[16 + 1];
  uint16_t  conn_handle;
  uint8_t   MacAddress[6];
  float     distance;
  int8_t    rssi;
  uint8_t   batterypercentage;

  BLEClientService radarService         = BLEClientService(RADARSERVICE);
  BLEClientCharacteristic radarChar     = BLEClientCharacteristic(RADARCHAR);
  BLEClientCharacteristic radarRes      = BLEClientCharacteristic(RADARRES);
  BLEClientService batteryService       = BLEClientService(UUID16_SVC_BATTERY);
  BLEClientCharacteristic batteryLevel  = BLEClientCharacteristic(UUID16_CHR_BATTERY_LEVEL);

} prph_info_t;


extern prph_info_t prphs[MAX_CONN];

extern uint8_t  doorSen_batt_level;
extern int8_t   s4_rssi;
extern uint32_t UsageCount;
extern bool     doorStatus;
extern int8_t   Radar_Rssi;
extern float    distance;
extern uint8_t radar_batteryLevel;

void ble_init();


// ACCELEROMETER
#include <Rak_BMX160.h>
#define INT1_PIN WB_IO5 // if use SLOT A INT1_PIN use WB_IO1,if use SLOT B,the INT1_PIN use WB_IO2. if use SLOT C,the INT1_PIN use WB_IO3.  if use SLOT D,the INT1_PIN use WB_IO5
#define HIGH_G_INT 0x07
#define HIGH_G_THRESHOLD 0x160
/*HIGH_G_THRESHOLD means to set the interrupt trigger threshold,If you want the accelerometer to trigger interrupts at 1g, just set HIGH_G_THRESHOLD to 0x80[0x80*7.81=999.68mg(2g range)].
Because accelerometers have different ranges, so the interrupt threshold is set differently. threshold = (HIGH_G_THRESHOLD*7.81) mg(2g range), (HIGH_G_THRESHOLD*15.63 )mg (4g range), (HIGH_G_THRESHOLD*31.25)mg (8g range),(HIGH_G_THRESHOLD* 62.5)mg (16g range) */

extern RAK_BMX160 bmx160;
extern const float G_FORCE_THRESHOLD;
extern const float TIPOVER_ANGLE_CHANGE;
extern sBmx160SensorData_t Oaccel;
extern float tiltAngle;
extern bool INT1_Flag;
extern SemaphoreHandle_t AccelInterrupt;
void Accelerometer_init(void);
void get_accelerometer_data(sBmx160SensorData_t *Oaccel);
void Accelerometer_Task();


// HUB BATTERY VOLATGE
#define PIN_VBAT WB_A0
#define VBAT_MV_PER_LSB (0.73242188F)  // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)       // Compensation factor for the VBAT divider, depend on the board
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

void Vbat_init() ;
void Battery_Level_task(void);

// GNSS
#define BG77_POWER_KEY WB_IO1
#define GPS_ANTENA WB_IO2
extern double lat_decimal;
extern double long_decimal;

void BG77_init();
void Get_GNSSData();
void BG77_read(time_t timeout, char Gps_buffer[],int size);
void BG77_write(const char* command);

// Flash
void init_flash(void);
void save_settings(void);
void log_settings(void);
void flash_reset(void);

struct s_config_params
{
  uint8_t valid_mark_1 = 0xAA; // Just a marker for the Flash
  uint8_t valid_mark_2 = 0xBB; // Just a marker for the Flash
  uint8_t configVersion = 0;
  uint8_t reporting_frequency = 96;
  uint8_t gpsLocationFrequency = 96;
  uint8_t wasteLevelFrequency = 96;
  uint8_t unitOrientationFrequency = 96;
  uint8_t batteryLevelFrequency = 96;

  uint8_t hub_mac[6] = {0x00,0x00,0x00,0x00,0x00,0x00};

  uint8_t wasteSensorMacs[3][6] = {
                                    {0x00,0x00,0x00,0x00,0x00,0x00},
                                    {0x00,0x00,0x00,0x00,0x00,0x00},
                                    {0x00,0x00,0x00,0x00,0x00,0x00}
                                  };

  uint8_t doorSensorMacs[3][6] = {
                                    {0x00,0x00,0x00,0x00,0x00,0x00},
                                    {0x00,0x00,0x00,0x00,0x00,0x00},
                                    {0x00,0x00,0x00,0x00,0x00,0x00}
                                  }; 
};

// int size = sizeof(s_lorawan_settings);
extern s_config_params g_config_settings;

extern int numWasteSensors;
extern int numDoorSensors;

extern int reportingFrequency;
extern int gpsLocationFrequency;
extern int wasteLevelFrequency;
extern int unitOrientationFrequency;
extern int batteryLevelFrequency;

void updateConfigParams();

#define   MAX_SAMPLES   100

typedef struct hubdata_t {
  uint32_t timestamp;
  uint8_t batteryLevel;
  uint8_t orientation;
};
extern hubdata_t hubdata[MAX_SAMPLES];

typedef struct gnssData_t {
  float latitude;
  float longitude;
  uint32_t timestamp;
};
extern gnssData_t gnssData[MAX_SAMPLES];

typedef struct doorSenData_t {
  uint8_t  MAC[6];
  int8_t   rssi;
  uint8_t  batteryLevel;
  uint32_t doorcount;
  uint32_t timestamp; 
  
};
extern doorSenData_t doorSenData[MAX_SAMPLES];
extern doorSenData_t temp_doorSenData[3];

typedef struct radarSenData_t {
  uint8_t  MAC[6];
  int8_t   rssi;
  float    distance;
  uint8_t  batteryLevel;
  uint32_t timestamp; 
};
extern radarSenData_t RadarSenData[MAX_SAMPLES];

void createJson();



  typedef enum BG77_status_t {
    BG77_IDLE = 0,
    GNSS_RUNNING,
    GSM_RUNNING
  };

extern BG77_status_t  BG77_status;
#endif // __MAIN_H