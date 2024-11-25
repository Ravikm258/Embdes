#include "services/BLEBas.h"
#include "delay.h"
#include "Adafruit_USBD_CDC.h"
#include "bluefruit.h"
#include "main.h"


int8_t s4_rssi = 0;
uint32_t UsageCount = 0;
bool doorStatus = false;
int8_t Radar_Rssi = 0;
float distance;
uint8_t radar_batteryLevel = 0;
uint8_t doorSen_batt_level = 0;

BLEBas batsrv;


doorSenData_t temp_doorSenData[3];
doorSenData_t doorSenData[MAX_SAMPLES];
radarSenData_t RadarSenData[MAX_SAMPLES];

void scan_callback(ble_gap_evt_adv_report_t* report);
void connect_callback(uint16_t conn_handle) ;
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void scanstop_callabck(void);
void app_connect_callback(uint16_t conn_handle);
void app_disconnect_callback(uint16_t conn_handle, uint8_t reason);
void pairing_complete_callback(uint16_t conn_handle, uint8_t auth_status);
void connection_secured_callback(uint16_t conn_handle);
int findConnHandle(uint16_t conn_handle) ;
// Struct containing peripheral info


prph_info_t prphs[MAX_CONN];

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // Bluefruit.Advertising.addTxPower();

  // Secondary Scan Response packet
  Bluefruit.ScanResponse.addName();

  // Start Advertising
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(1600, 4000);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(10);    // fast mode timeout in seconds
  Bluefruit.Advertising.start(0);              // advertise forever

}

void start_scan(void) {
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setStopCallback(scanstop_callabck);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.setInterval(250, 80);  // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);  // Don't request scan response data
  Bluefruit.Scanner.start(3000);           // 0 = Don't stop scanning after n seconds
}


void ble_init()
{
  Serial.println("Initialize BLE");
  // Initialize Bluefruit with max concurrent connections as Peripheral = 0, Central = 4
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(1, 4);

  
  // Set Name
  ble_gap_addr_t hub_addr = Bluefruit.getAddr();
  char devicename[16];
  sprintf(devicename,"PortaConX-%02X%02X",hub_addr.addr[1],hub_addr.addr[0]);
  Bluefruit.setName(devicename);
  Bluefruit.setTxPower(-4);

  Bluefruit.Security.setIOCaps(false, false, false);  // display = true, yes/no = true, keyboard = false

  // Set complete callback to print the pairing result
  Bluefruit.Security.setPairCompleteCallback(pairing_complete_callback);

  // Set connection secured callback, invoked when connection is encrypted
  Bluefruit.Security.setSecuredCallback(connection_secured_callback);
  // Init peripheral pool
  for (uint8_t idx = 0; idx < MAX_CONN; idx++) {
    // Invalid all connection handle
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;

    // All of BLE Central Radar Serivce
    prphs[idx].radarService.begin();
    prphs[idx].radarChar.begin();
    prphs[idx].radarRes.begin();
    prphs[idx].batteryService.begin();
    prphs[idx].batteryLevel.begin();
  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  // Callbacks for Peripheral
  Bluefruit.Periph.setConnectCallback(app_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(app_disconnect_callback);

  Bluefruit.autoConnLed(false);
  Bluefruit.setConnLedInterval(500);
  Bluefruit.Periph.setConnInterval(400, 800);
  Bluefruit.Periph.setConnSupervisionTimeout(400);

  batsrv.begin();

  startAdv();
  start_scan();
}



/**
 * Callback invoked when scanner picks up an advertising packet
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t door_sensor_data[4];
  memset(door_sensor_data,0,4);
  static uint8_t index = 0;
  static uint32_t prev_count = 0xFFFFFFFF;
  uint8_t* adv_data = report->data.p_data;
  uint8_t adv_len = report->data.len;
  
  uint8_t* peer_addr = report->peer_addr.addr;

  for (int i = 0; i < numDoorSensors; i++) 
  {
    if (memcmp((uint8_t*)peer_addr, (uint8_t*)g_config_settings.doorSensorMacs[i], 6) == 0) 
    {
      uint8_t frame_version = adv_data[8];

      s4_rssi = report->rssi;

      if (frame_version == 0x00) {
        doorSen_batt_level = adv_data[15];
        Serial.print("Door Sensor Battery Level: ");
        Serial.print(doorSen_batt_level, DEC);
        Serial.println(" %");

        temp_doorSenData[i].batteryLevel = doorSen_batt_level;
        
      } else if (frame_version == 0x03) {
        door_sensor_data[0] = adv_data[10];
        door_sensor_data[1] = adv_data[11];
        door_sensor_data[2] = adv_data[12];

        doorStatus = (door_sensor_data[0] >> 7) && 0x01;
        UsageCount  = (door_sensor_data[1] + door_sensor_data[2]) / 2;
        
        Serial.print("UsageCount: ");
        Serial.println(UsageCount, DEC);

        temp_doorSenData[i].timestamp = millis();
        memcpy(temp_doorSenData[i].MAC,peer_addr,6);
        temp_doorSenData[i].rssi = s4_rssi;
        temp_doorSenData[i].doorcount = UsageCount;
      }
      else{
        //do nothing
      }
    }
  }
  for (int i = 0; i < numWasteSensors; i++) 
  {
    if (memcmp((uint8_t*)peer_addr, (uint8_t*)g_config_settings.wasteSensorMacs[i], 6) == 0) {
      int id = findConnHandle(BLE_CONN_HANDLE_INVALID);
      prphs[id].rssi = report->rssi;
      Bluefruit.Central.connect(report);

    } 
  }
  Bluefruit.Scanner.resume();
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle) {
  uint8_t data[12];
  float distance = 0;
  static uint8_t index = 0;
  ble_gap_addr_t address;
  // Find an available ID to use
  int id = findConnHandle(BLE_CONN_HANDLE_INVALID);

  // Eeek: Exceeded the number of connections !!!
  if (id < 0) return;
  
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  // Serial.print("Connection handle is : ");
  // Serial.println(id);
  prphs[id].conn_handle = conn_handle;

  conn->getPeerName(prphs[id].name, sizeof(prphs[id].name) - 1);  // get name of connection
  address = conn->getPeerAddr();                                  // get address struct of connection
  memcpy(prphs[id].MacAddress, address.addr, 6);                  // fetch MAC address from address struct

  // print MAC address and Name of the the connected device
  Serial.printBuffer(prphs[id].MacAddress, 6, ':');
  Serial.print("\n\rConnected to ");
  Serial.println(prphs[id].name);

  // Check if the device is paired, if not request pairing
  if (!conn->bonded()) {
    conn->requestPairing();
  }

  // Discover service and Characteristics
  // Serial.print("Discovering service ... ");

  if (prphs[id].radarService.discover(conn_handle)) 
  {
    Serial.println("Radar Service discovered!");

    // Discover characteristic
    if (prphs[id].radarChar.discover()) 
    {
      // Serial.println("Radar Characteristic discovered!");

      // Write value to the characteristic
      uint8_t value_to_write = 0x01;  // Example value to write
      if (prphs[id].radarChar.write8(value_to_write)) 
      {
        // Serial.println("Value written to the characteristic!");
        delay(500);
        if (prphs[id].radarRes.discover()) 
        {
          prphs[id].radarRes.read(data, 12);
          memcpy(&distance, &data[0], 4);  // Extract the 32-bit float (Distance)
          prphs[id].distance = distance;
          Serial.print("Distance (m): ");
          Serial.println(distance);
          if (prphs[id].batteryService.discover(conn_handle)) 
          {
            Serial.println("Battery Service discovered!");
            if (prphs[id].batteryLevel.discover()) 
            {
              // Serial.println("Battery Characteristic discovered!");
              prphs[id].batterypercentage = prphs[id].batteryLevel.read8();
              Serial.print("Radar Battery Level (%): ");
              Serial.println(prphs[id].batterypercentage);
            }
            RadarSenData[index % MAX_SAMPLES].timestamp = millis(); 
            memcpy(RadarSenData[index % MAX_SAMPLES].MAC,prphs[id].MacAddress,6);  
            RadarSenData[index % MAX_SAMPLES].rssi =  prphs[id].rssi;
            RadarSenData[index % MAX_SAMPLES].distance = prphs[id].distance;  
            RadarSenData[index % MAX_SAMPLES].batteryLevel = prphs[id].batterypercentage;  
            index++;
          }
          Bluefruit.Scanner.start(3000);
        }
      }
    }
  } 
  else 
  {
    Serial.println("Found ... NOTHING!");
    Bluefruit.disconnect(conn_handle);
    Bluefruit.Scanner.start(3000);
  }
}


/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  // Mark the ID as invalid
  int id = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if (id < 0) return;

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].name);
  Serial.println(" disconnected!");


  memset(prphs[id].name, '\0', 17);
  memset(prphs[id].MacAddress, '\0', 6);
  prphs[id].distance = 0;
  prphs[id].rssi = 0;
  prphs[id].batterypercentage = 0;
  
}

/****************** Scan Stop Callback *******************/

/**
 * Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
void scanstop_callabck(void) {
  static int index;
  for(int i = 0; i < numDoorSensors; i++)
  {
    if(temp_doorSenData[i].timestamp != 0) {
      doorSenData[index % MAX_SAMPLES].timestamp = temp_doorSenData[i].timestamp;
      memcpy(doorSenData[index % MAX_SAMPLES].MAC,temp_doorSenData[i].MAC,6);
      doorSenData[index % MAX_SAMPLES].rssi = temp_doorSenData[i].rssi;
      doorSenData[index % MAX_SAMPLES].doorcount = temp_doorSenData[i].doorcount;
      doorSenData[index % MAX_SAMPLES].batteryLevel = temp_doorSenData[i].batteryLevel;
      index++;
    }
  }
  for(int i = 0;i < MAX_CONN; i++) 
  {
    if(Bluefruit.connected(prphs[i].conn_handle))
      Bluefruit.disconnect(prphs[i].conn_handle);
  }
  Serial.println("Scanning Stopped");
}

/**
 * Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
int findConnHandle(uint16_t conn_handle) {
  for (int id = 0; id < MAX_CONN; id++) {
    if (conn_handle == prphs[id].conn_handle) {
      return id;
    }
  }

  return -1;
}



/******************  APP BLE CALLBACK FUNCTIONS *********************/

void app_connect_callback(uint16_t conn_handle) {
  Serial.println("Connected from APP");
  updateConfigParams();
}

void app_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  Serial.println("Disconnected from APP");
}


void pairing_complete_callback(uint16_t conn_handle, uint8_t auth_status) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  Serial.println(auth_status);
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS) {
    Serial.println("Succeeded");
  } else {
    Serial.println("Failed");
  }
}


void connection_secured_callback(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  if (!conn->secured()) {
    conn->requestPairing();
  }
}