#include "main.h"

SoftwareTimer reportTimer, scan_timer;
s_config_params g_config_settings;
SemaphoreHandle_t AccelInterrupt;
TimerHandle_t ReportTimer;

bool state;

void setup() {
  #if PRINT_LOG
  Serial.begin(115200);
  while (!Serial) delay(10);  

  Serial.println("---------------------------------------------");
  Serial.println("-------- PortaConX Integrated project -------");
  Serial.println("---------------------------------------------\n");
  #endif

  pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, LOW);

  AccelInterrupt = xSemaphoreCreateBinary();
  xSemaphoreGive(AccelInterrupt);
  
  Serial1.begin(115200);
  delay(1000);

  pinMode(GPS_ANTENA, OUTPUT);
  digitalWrite(GPS_ANTENA, HIGH);
  xSemaphoreTake(AccelInterrupt, 10);
  Accelerometer_init();
  Vbat_init();
  init_flash();
  ble_init();

  scan_timer.begin(getTimerInterval_ms(g_config_settings.wasteLevelFrequency), Scan_TimerCallback);
  // scan_timer.begin(MINUTES * 5, Scan_TimerCallback);
  scan_timer.start();

  ReportTimer = xTimerCreate("Reporting timer", getTimerInterval_ms(g_config_settings.reporting_frequency), true, NULL, Report_TimerCallback);
  // ReportTimer = xTimerCreate("Reporting timer", MINUTES * 60 * 8, true, NULL, Report_TimerCallback);
  xTimerStart(ReportTimer, MINUTES * 2);   //strat report timer after 2 minutes

  Scheduler.startLoop(Accelerometer_Task);
  Scheduler.startLoop(Battery_Level_task);
  Scheduler.startLoop(Get_GNSSData);
}

void loop() {
  digitalWrite(LED_GREEN, LOW);
  if (xSemaphoreTake(AccelInterrupt, portMAX_DELAY) == pdTRUE)
	{
    digitalWrite(LED_GREEN, HIGH);
    float X_g, Y_g, Z_g;
    float G_force;
    Serial.println("");
    delay(100);
    bmx160.getAllData(NULL, NULL, &Oaccel);
    X_g = Oaccel.x;
    Y_g = Oaccel.y;
    Z_g = Oaccel.z;
    // Calculate total G-force
    G_force = sqrt(X_g * X_g + Y_g * Y_g + Z_g * Z_g);
    // Check for G-force event (impact)
    if ((G_force - 12) >= G_FORCE_THRESHOLD) {

      Serial.print("Impact detected..! G-force: ");
      Serial.println(G_force - 12);

    }
    delay(2000);
    bmx160.getAllData(NULL, NULL, &Oaccel);
    X_g = Oaccel.x;
    Y_g = Oaccel.y;
    Z_g = Oaccel.z;
    tiltAngle = atan2(sqrt(X_g * X_g + Y_g * Y_g), Z_g) * 180.0 / PI;
    // Tipover detection (e.g., tilt angle greater than 45 degrees)
    if ((180 - tiltAngle) > TIPOVER_ANGLE_CHANGE) {
      Serial.print("Tipover detected..! Orientation : ");
      Serial.println(180-tiltAngle);
    }
    xSemaphoreTake(AccelInterrupt, 10);

  }
  if(state){
    digitalToggle(LED_BLUE);
    delay(500);
  }

}


void Scan_TimerCallback(TimerHandle_t xtimer) {
  Serial.println("Scanning restarted");
  Bluefruit.Scanner.start(2000);
}

void Report_TimerCallback(TimerHandle_t xtimer)
{
  state = true;
  Serial.println("Create json and Send data to cloud");
  createJson();
}

uint32_t getTimerInterval_ms(int frequency) {
    return frequency > 0 ? MINUTES * ((24 * 60) / frequency) : MINUTES * (1440/96); // Default to 96 times/day if 0
}
