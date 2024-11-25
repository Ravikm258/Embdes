#include "delay.h"
#include "avr/pgmspace.h"
#include "WString.h"
#include "Adafruit_USBD_CDC.h"

#include "main.h"
#include <nrf_nvic.h>

s_config_params g_flash_content;

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

static const char settings_name[] = "RAK";

File file(InternalFS);

void flash_reset(void);
void log_settings(void);
void remove_file();
/**
   @brief Initialize access to nRF52 internal file system

*/
void init_flash(void)
{
  // Initialize Internal File System
  InternalFS.begin();

  // Check if file exists
  file.open(settings_name, FILE_O_READ);
  if (!file)
  {
    Serial.println("File doesn't exist, update with default values");
    delay(100);
    flash_reset();
    return;
  }
  Serial.println("Read configuartion settings from flash");
  file.read((uint8_t *)&g_config_settings, sizeof(s_config_params));
  file.close();
  
  // Check if it is config settings
  if ((g_flash_content.valid_mark_1 != 0xAA) || (g_flash_content.valid_mark_2 != 0xBB))
  {
    // Data is not valid, reset to defaults
    Serial.println("Invalid data set, deleting and restart node");
    // Remove the file
    remove_file();
    delay(1000);
    sd_nvic_SystemReset();
  }
  log_settings();
}

/**
   @brief Save changed settings if required

   @return boolean
  			result of saving
*/
void save_settings(void)
{
  Serial.println("Save config settings to flash");
  bool result = false;
  // Read saved content
  file.open(settings_name, FILE_O_READ);
  if (!file)
  {
    Serial.println("File doesn't exist, force format");
    delay(100);
    flash_reset();
  }
  file.read((uint8_t *)&g_flash_content, sizeof(s_config_params));
  file.close();
  if (memcmp((void *)&g_flash_content, (void *)&g_config_settings, sizeof(s_config_params)) != 0)
  {
    Serial.println("Flash content changed, erase old configuration and writing new data");
    // Remove the file
    remove_file();
    delay(100);
    if (file.open(settings_name, FILE_O_WRITE))
    {
      Serial.println("....!");
      file.write((uint8_t *)&g_config_settings, sizeof(s_config_params));
      file.flush();
      result = true;
    }
    else
    {
      result = false;
    }
    file.close();
  }
  log_settings();
  if(result){
    delay(1000);
    sd_nvic_SystemReset();
  }
}

void remove_file(){
  // Check if the file exists
  File file = InternalFS.open(settings_name, FILE_O_READ);
  if (file) 
  {
    file.close();   // Close the file before attempting to remove it

    // Attempt to remove the file 
    if (InternalFS.remove(settings_name)) 
    {
      Serial.print("Old configuration removed successfully");
    } 
  } 
  else 
  {
    Serial.print("File not found");
  }
}


/**
   @brief Reset content of the filesystem

*/
void flash_reset(void)
{
  if (file.open(settings_name, FILE_O_WRITE))
  {
    file.write((uint8_t *)&g_config_settings, sizeof(s_config_params));
    file.flush();
    file.close();
  }
}


void log_settings(void)
{
  Serial.println("");
  Serial.println("Configuration version : " + String(g_config_settings.configVersion));
  Serial.print("HUB MAC : ");
  Serial.printBuffer(g_config_settings.hub_mac,6,':');
  Serial.println("");
  
  Serial.print("Door MAC : ");
  for(int i = 0; i < 3; i++){
    Serial.printBuffer(g_config_settings.doorSensorMacs[i],6,':');
    Serial.println("");
  }
    Serial.print("Radar MAC : ");
  for(int i = 0; i < 3; i++){
    Serial.printBuffer(g_config_settings.wasteSensorMacs[i],6,':');
    Serial.println("");
  }

  
  Serial.print("Reporting Frequency: "); Serial.println(g_config_settings.reporting_frequency);
  Serial.println("Capture Frequency : ");
  Serial.print("\t\t\tGPS Location : "); Serial.println(g_config_settings.gpsLocationFrequency);
  Serial.print("\t\t\tWaste Level : "); Serial.println(g_config_settings.wasteLevelFrequency);
  Serial.print("\t\t\tUnit Orientation : "); Serial.println(g_config_settings.unitOrientationFrequency);
  Serial.print("\t\t\tBattery Level : "); Serial.println(g_config_settings.batteryLevelFrequency);
}








