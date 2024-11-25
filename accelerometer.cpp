#include "delay.h"
#include "main.h"


RAK_BMX160 bmx160;
sBmx160SensorData_t Oaccel;
float accelero_X,accelero_Y,accelero_Z;
bool INT1_Flag = false;
float tiltAngle = 0;
const float G_FORCE_THRESHOLD = 2.0;  // G-force threshold for impact detection
const float TIPOVER_ANGLE_CHANGE = 45.0;  // Tipover threshold in degrees
hubdata_t hubdata[MAX_SAMPLES];


void To_INT1_Interrupt(void);
void get_accelerometer_data(void);

/* Initialize Accelerometer */
void Accelerometer_init(void) {
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(300);
  //init the hardware bmx160
  if (bmx160.begin() != true) {
    Serial.println("bmx160 init false");
    while (1) delay(100);
  }
  bmx160.wakeUp();        //enable the gyroscope and accelerometer sensor
  pinMode(INT1_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), To_INT1_Interrupt, RISING);
  //  LOW： when the pin is low, the interrupt service program is triggered
  //  CHANGE： trigger the interrupt service program when the pin level changes
  //  RISING： when the pin level changes from low level to high level, the interrupt service program is triggered
  //  FALLING： when the pin level changes from high level to low level, the interrupt service program is triggered

  uint8_t PMU_Status = 0;
  bmx160.readReg(0x03, &PMU_Status, 1);
  Serial.printf("PMU_Status=%x\r\n", PMU_Status);

  bmx160.InterruptConfig(HIGH_G_INT, HIGH_G_THRESHOLD);  // Enable HIGH_G_Interrupt ands et the accelerometer threshold

  // bmx160.ODR_Config(BMX160_ACCEL_ODR_200HZ, BMX160_GYRO_ODR_200HZ); // set output data rate
  float OrdBuf[2] = { 0 };
  bmx160.get_ORD_Config(&OrdBuf[0], &OrdBuf[1]);
  Serial.printf("OrdBuf[0]=%f,OrdBuf[1]=%f\r\n", OrdBuf[0], OrdBuf[1]);

  bmx160.setAccelRange(eAccelRange_2G);

  Serial.println("Accelerometer initialized....");
}

/* Callback function to Acce;erometer interrupt */
void To_INT1_Interrupt(void) {
  xSemaphoreGiveFromISR(AccelInterrupt, pdFALSE);
}


/* Get Accelerometer data */
void get_accelerometer_data(sBmx160SensorData_t *Oaccel) {
  bmx160.getAllData(NULL, NULL, Oaccel);
  accelero_X = Oaccel->x ;
  accelero_Y = Oaccel->y ;
  accelero_Z = Oaccel->z ;
}


/*  RTOS task for getting accelerometer data */
void Accelerometer_Task() {
  static int index = 0;
  sBmx160SensorData_t Oaccel;
  get_accelerometer_data(&Oaccel);

  Serial.print("Accel in m/s^2: ");
  Serial.printf("X: %f   Y : %f   Z : %f\n\r", Oaccel.x, Oaccel.y, Oaccel.z);

  tiltAngle = atan2(sqrt(Oaccel.x * Oaccel.x + Oaccel.y * Oaccel.y), Oaccel.z) * 180.0 / PI;
  tiltAngle = 180 - tiltAngle;

  Serial.printf("Tilt Angle : %f\n\r",tiltAngle);
  hubdata[index % MAX_SAMPLES].timestamp = millis();
  hubdata[index % MAX_SAMPLES].orientation = tiltAngle;
  index++;
  
  // delay(MINUTES * 5);
  delay(getTimerInterval_ms(g_config_settings.unitOrientationFrequency));  
}