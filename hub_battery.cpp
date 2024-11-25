#include "delay.h"
#include "main.h"

uint32_t vbat_pin = PIN_VBAT;
extern BLEBas batsrv;


float readVBAT(void);
uint8_t mvToPercent(float mvolts) ;
/************************************************/
/*                  BATTERY                     */
/***********************************************/
void Battery_Level_task(void) {
  static uint8_t index;
  // Get a raw ADC reading
  float vbat_mv = readVBAT();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_mv);
  Serial.println("");
  Serial.print("LIPO = ");
  Serial.print(vbat_mv);
  Serial.print(" mV (");
  Serial.print(vbat_per);
  Serial.println(" %)");
  batsrv.write(vbat_per);
  batsrv.notify(vbat_per);
  hubdata[index % MAX_SAMPLES].timestamp = millis();
  hubdata[index % MAX_SAMPLES].batteryLevel = vbat_per;
  index++;
  delay(getTimerInterval_ms(g_config_settings.batteryLevelFrequency));
  // delay(MINUTES * 5);
}

void Vbat_init() {
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12);  // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(10);

}

/**
 * @brief Get RAW Battery Voltage
 */
float readVBAT(void) {
  float raw;

  // Get the raw 12-bit, 0..3000mV ADC value
  for (int i = 0; i < 10; i++)
    raw += analogRead(vbat_pin);
  raw /= 10;
  // Serial.println(raw);
  return raw * REAL_VBAT_MV_PER_LSB;
}

/**
 * @brief Convert from raw mv to percentage
 * @param mvolts
 *    RAW Battery Voltage
 */
uint8_t mvToPercent(float mvolts) {
  if (mvolts < 3300)
    return 0;

  if (mvolts < 3600) {
    mvolts -= 3300;
    return mvolts / 30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F);  // thats mvolts /6.66666666
}