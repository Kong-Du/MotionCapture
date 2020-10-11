//MASTER
#include <SPI.h>
#include "MPU9250.h"
#include "BluetoothSerial.h"

BluetoothSerial bt;

#define CHIP_SELECT_PIN         13

#define IMU_POLL_DELAY_MS         2
#define MFILTER_SAMPLE_FREQ_HZ    480
#define IMU_LOWPASSFILTER_BANDWIDTH   MPU9250::DLPF_BANDWIDTH_184HZ

#define CALIB_DISABLE
#define CALIB_DISABLE
//#define ACC_CALIB_DONE
//#define ACC_CALIB_Disable
//#define MAG_CALIB_DONE
#define MAG_CALIB_DISABLE

float q_value[3][4];
int yaw_print[3];
int pitch_print[3];
int roll_print[3];
const byte gpio_input = 5;
int state;
int count;
#define RXD1 34
#define TXD1 32

#define RXD_RS485 33
#define TXD_RS485 25
#define RS485_E 26

#define ACC 0xF0
#define MAG 0xF1
#define GYRO 0xF2

String data_read;
char data_print[50];

int revision(float GYRO_X, float GYRO_Y, float GYRO_Z, float ACC_X_H, float ACC_X_L, float ACC_Y_H, float ACC_Y_L, float ACC_Z_H, float ACC_Z_L, float MAG_X_H, float MAG_X_L, float MAG_Y_H, float MAG_Y_L, float MAG_Z_H, float MAG_Z_L)
{
  unsigned long settime = millis();
  while(1)
  {
    Send_revision(GYRO, (int)((GYRO_X+50)*1000000), (int)((GYRO_Y+50)*1000000), (int)((GYRO_Z+50)*1000000), (int)((GYRO_X+50)*1000000), (int)((GYRO_Y+50)*1000000), (int)((GYRO_Z+50)*1000000));
    Send_revision(ACC, (int)((ACC_X_H+50)*1000000), (int)((ACC_Y_H+50)*1000000), (int)((ACC_Z_H+50)*1000000), (int)((ACC_X_L+50)*1000000), (int)((ACC_Y_L+50)*1000000), (int)((ACC_Z_L+50)*1000000));
    Send_revision(MAG, (int)((MAG_X_H+50)*1000000), (int)((MAG_Y_H+50)*1000000), (int)((MAG_Z_H+50)*1000000), (int)((MAG_X_L+50)*1000000), (int)((MAG_Y_L+50)*1000000), (int)((MAG_Z_L+50)*1000000));
    const byte bufSize_check = 5;
    byte buf_check[bufSize_check];
    if(((millis() - settime) > 5000)){
      return 1;
    }
  }
}

void Send_revision(int id, int X_H, int Y_H, int Z_H, int X_L, int Y_L, int Z_L)
{
  //미리 값들에 +50하고, *1000000로 넣어주기
  //받을 때 -100 해주기.
  byte _id = id;  
  byte X_H_3 = (byte)((X_H/1000000 + 100));
  byte X_H_2 = (byte)(((X_H-((int)(X_H_3-100)*1000000))/10000 + 100));
  byte X_H_1 = (byte)(((X_H -((int)(X_H_3-100)*1000000)-((int)(X_H_2-100)*10000))/100 + 100));
  byte X_H_0 = (byte)((X_H -((int)(X_H_3-100)*1000000)-((int)(X_H_2-100)*10000)-((int)(X_H_1-100)*100) + 100));
  byte Y_H_3 = (byte)((Y_H/1000000 + 100));
  byte Y_H_2 = (byte)(((Y_H-((int)(Y_H_3-100)*1000000))/10000 + 100));
  byte Y_H_1 = (byte)(((Y_H -((int)(Y_H_3-100)*1000000)-((int)(Y_H_2-100)*10000))/100 + 100));
  byte Y_H_0 = (byte)((Y_H -((int)(Y_H_3-100)*1000000)-((int)(Y_H_2-100)*10000)-((int)(Y_H_1-100)*100) + 100));
  byte Z_H_3 = (byte)((Z_H/1000000 + 100));
  byte Z_H_2 = (byte)(((Z_H-((int)(Z_H_3-100)*1000000))/10000 + 100));
  byte Z_H_1 = (byte)(((Z_H -((int)(Z_H_3-100)*1000000)-((int)(Z_H_2-100)*10000))/100 + 100));
  byte Z_H_0 = (byte)((Z_H -((int)(Z_H_3-100)*1000000)-((int)(Z_H_2-100)*10000)-((int)(Z_H_1-100)*100) + 100));

  byte X_L_3 = (byte)((X_L/1000000 + 100));
  byte X_L_2 = (byte)(((X_L-((int)(X_L_3-100)*1000000))/10000 + 100));
  byte X_L_1 = (byte)(((X_L -((int)(X_L_3-100)*1000000)-((int)(X_L_2-100)*10000))/100 + 100));
  byte X_L_0 = (byte)((X_L -((int)(X_L_3-100)*1000000)-((int)(X_L_2-100)*10000)-((int)(X_L_1-100)*100) + 100));
  byte Y_L_3 = (byte)((Y_L/1000000 + 100));
  byte Y_L_2 = (byte)(((Y_L-((int)(Y_L_3-100)*1000000))/10000 + 100));
  byte Y_L_1 = (byte)(((Y_L -((int)(Y_L_3-100)*1000000)-((int)(Y_L_2-100)*10000))/100 + 100));
  byte Y_L_0 = (byte)((Y_L -((int)(Y_L_3-100)*1000000)-((int)(Y_L_2-100)*10000)-((int)(Y_L_1-100)*100) + 100));
  byte Z_L_3 = (byte)((Z_L/1000000 + 100));
  byte Z_L_2 = (byte)(((Z_L-((int)(Z_L_3-100)*1000000))/10000 + 100));
  byte Z_L_1 = (byte)(((Z_L -((int)(Z_L_3-100)*1000000)-((int)(Z_L_2-100)*10000))/100 + 100));
  byte Z_L_0 = (byte)((Z_L -((int)(Z_L_3-100)*1000000)-((int)(Z_L_2-100)*10000)-((int)(Z_L_1-100)*100) + 100));
  
  byte checksum = ~lowByte(id + X_H_3 + X_H_2 + X_H_1 + X_H_0 + Y_H_3 + Y_H_2 + Y_H_1 + Y_H_0 + Z_H_3 + Z_H_2 + Z_H_1 + Z_H_0 + X_L_3 + X_L_2 + X_L_1 + X_L_0 + Y_L_3 + Y_L_2 + Y_L_1 + Y_L_0 + Z_L_3 + Z_L_2 + Z_L_1 + Z_L_0);
  
  digitalWrite(RS485_E, HIGH);
  delay(5);
  Serial2.write(0xFE);
  Serial2.write(0xFE);
  Serial2.write(_id);
  Serial2.write(X_H_3);
  Serial2.write(X_H_2);
  Serial2.write(X_H_1);
  Serial2.write(X_H_0);
  Serial2.write(Y_H_3);
  Serial2.write(Y_H_2);
  Serial2.write(Y_H_1);
  Serial2.write(Y_H_0);
  Serial2.write(Z_H_3);
  Serial2.write(Z_H_2);
  Serial2.write(Z_H_1);
  Serial2.write(Z_H_0);
  Serial2.write(X_L_3);
  Serial2.write(X_L_2);
  Serial2.write(X_L_1);
  Serial2.write(X_L_0);
  Serial2.write(Y_L_3);
  Serial2.write(Y_L_2);
  Serial2.write(Y_L_1);
  Serial2.write(Y_L_0);
  Serial2.write(Z_L_3);
  Serial2.write(Z_L_2);
  Serial2.write(Z_L_1);
  Serial2.write(Z_L_0);
  Serial2.write(checksum);
  Serial2.flush();
  digitalWrite(RS485_E, LOW);
  delay(1);
}

int Recivedata(byte* buf, uint8_t n)
{
  uint8_t bufIdx = 2;
  uint8_t check = 0;
  byte c;
  byte checksum = 0;
  digitalWrite(RS485_E, LOW);
  delay(1);
  do
  {
    while(!(check == 2))
    {
      c = Serial2.read();
      if(c == 0xFE)
      {
        check++;
      }
      else{
        return 0;
      }
    }
    c = Serial2.read();
    buf[bufIdx++] = c;
  }
  while((bufIdx < n));
  
  for(int i = 2; i<n; i++)
  {
    checksum += buf[i];
  }
  checksum = ~lowByte(checksum);
  buf[0] = 0xFE;
  buf[1] = 0xFE;
  digitalWrite(RS485_E, HIGH);
  delay(1);  
  if(!checksum){
    return 1;
  }
  else{
    return 0;
  }
}
MPU9250 IMU(SPI, gpio_input);
Madgwick AHRSFilter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
/*
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
//  Serial.println(data_print);
  bt.println(data_print);
  portEXIT_CRITICAL_ISR(&timerMux);
}
*/
void start_set(void)
{
  digitalWrite(RS485_E, HIGH);
  delay(5);
  Serial2.write(0xFE);
  Serial2.write(0xFE);
  Serial2.write(0xFE);
  Serial2.flush();  
}
int status;

typedef union {
  struct {
    float roll;
    float pitch;
    float yaw;
  };
  uint8_t valArray[sizeof(float) * 3];
} NotifBytesBunch;

NotifBytesBunch AHRSValues;

void setup() {
  Serial.begin(2000000);
//  Serial1.begin(1000000, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(2000000, SERIAL_8N1, RXD_RS485, TXD_RS485);
  bt.begin("ESP32_SIBAR");
//  timer = timerBegin(0, 80, true);
//  timerAttachInterrupt(timer, &onTimer, true);
//  timerAlarmWrite(timer, 100000, true);  //1s = 1000000, 1ms = 10000 0.1ms = 1000
  pinMode(RS485_E, OUTPUT);
  while (!Serial) {}
  
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  IMU.setDlpfBandwidth(IMU_LOWPASSFILTER_BANDWIDTH);
  
  while (! Serial.available() ) {
      Serial.println("Start calibration");
      delay(10);
  }
   while (Serial.available() ) {
      Serial.read();    
      delay(5);
    }
  Serial.println(F("********** GYRO calib **************"));
  float gyrodir[3][3];
  for (uint8_t num = 0; num < 2; num++) {
    
    Serial.print(F("Don't move until I tell you.  "));
    Serial.print(num+1);
    Serial.println("...");  
    delay(500);
    state = IMU.calibrateGyro();
    if(state == 1){
      Serial.println(F("GYRO calib done"));
    }
    else{
      Serial.println(F("GYRO calib fail"));
    }
    Serial.println(F("Vals: "));
    Serial.print(F("X: "));
    gyrodir[0][num] = IMU.getGyroBiasX_rads();
    Serial.print(gyrodir[0][num], 6);
    Serial.println("rad/s");
    Serial.print(F("Y: "));
    gyrodir[1][num] = IMU.getGyroBiasY_rads();
    Serial.print(gyrodir[1][num], 6);
    Serial.println("rad/s");
    Serial.print(F("Z: "));
    gyrodir[2][num] = IMU.getGyroBiasZ_rads();
    Serial.print(gyrodir[2][num], 6);
    Serial.println("rad/s");
    if(num == 1){
      gyrodir[0][2] = (gyrodir[0][0] + gyrodir[0][1])/2;
      gyrodir[1][2] = (gyrodir[1][0] + gyrodir[1][1])/2;
      gyrodir[2][2] = (gyrodir[2][0] + gyrodir[2][1])/2;
      Serial.println(F("GYRO calib ALL done"));
      Serial.println(F("Vals: "));
      Serial.print(F("X: "));
      Serial.print(gyrodir[0][2], 6);
      Serial.println("rad/s");
      
      Serial.print(F("Y: "));
      Serial.print(gyrodir[1][2], 6);
      Serial.println("rad/s");
      
      Serial.print(F("Z: "));
      Serial.print(gyrodir[2][2], 6);      \
      Serial.println("rad/s");
      Serial.println();
      }
    }

  Serial.println(F("********** ACC calib **************"));
  float accdir[6][3];
  char * dirs[6] = { "X+", "X-", "Y+", "Y-", "Z+", "Z-"};

  for (uint8_t num = 0; num < 2; num++) {
      for (uint8_t i = 0; i < 6; i++) {
        Serial.print(F("Enter "));
        Serial.print((int)(num + 1));
        Serial.print(F(" when ready for dir "));
        Serial.print((int)(i + 1));
        Serial.print(' ');
        Serial.print(dirs[i]);
        while (! Serial.available() ) {
          delay(10);
        }
    
        while (Serial.available()) {
          Serial.read();
          delay(10);
          Serial.print('.');
        }
        Serial.println();
        state = IMU.calibrateAccel();
      }
      if(state == 1){
        Serial.println(F("Acc calib done"));
      }
      else{
        Serial.println(F("Acc calib fail"));        
      }
      Serial.println(F("Vals: "));
      Serial.print(F("X: "));
      accdir[0][num] = IMU.getAccelBiasX_mss();      
      Serial.print(IMU.getAccelBiasX_mss(), 6);
      Serial.print("m/s^2");
      Serial.print(" / ");
      accdir[1][num] = IMU.getAccelScaleFactorX();      
      Serial.print(IMU.getAccelScaleFactorX(), 6);
      Serial.println("m/s^2");
      Serial.print(F("Y: "));
      accdir[2][num] = IMU.getAccelBiasY_mss();
      Serial.print(IMU.getAccelBiasY_mss(), 6);
      Serial.print("m/s^2");      
      Serial.print(" / ");
      accdir[3][num] = IMU.getAccelScaleFactorY();      
      Serial.print(IMU.getAccelScaleFactorY(), 6);
      Serial.println("m/s^2");
      Serial.print(F("Z: "));
      accdir[4][num] = IMU.getAccelBiasZ_mss();
      Serial.print(IMU.getAccelBiasZ_mss(), 6);      
      Serial.print("m/s^2");
      Serial.print(" / ");
      accdir[5][num] = IMU.getAccelScaleFactorZ();      
      Serial.print(IMU.getAccelScaleFactorZ(), 6);
      Serial.println("m/s^2");      
      if(num == 1){
        accdir[0][2] = (accdir[0][0] + accdir[0][1])/2;
        accdir[1][2] = (accdir[1][0] + accdir[1][1])/2;
        accdir[2][2] = (accdir[2][0] + accdir[2][1])/2;
        accdir[3][2] = (accdir[3][0] + accdir[3][1])/2;
        accdir[4][2] = (accdir[4][0] + accdir[4][1])/2;
        accdir[5][2] = (accdir[5][0] + accdir[5][1])/2;
        
        Serial.println(F("Acc calib ALL done"));
        Serial.println(F("Vals: "));
        Serial.print(F("X: "));
        Serial.print(accdir[0][2], 6);
        Serial.print("m/s^2");
        Serial.print(" / ");
        Serial.print(accdir[1][2], 6);
        Serial.println("m/s^2");
        
        Serial.print(F("Y: "));
        Serial.print(accdir[2][2], 6);
        Serial.print("m/s^2");        
        Serial.print(" / ");
        Serial.print(accdir[3][2], 6);
        Serial.println("m/s^2");
        
        Serial.print(F("Z: "));
        Serial.print(accdir[4][2], 6);
        Serial.print("m/s^2");        
        Serial.print(" / ");
        Serial.print(accdir[5][2], 6);
        Serial.println("m/s^2");
        Serial.println();        
      }
  }

  Serial.println(F("********** MAG calib **************"));  
  float magdir[6][3];
  for (uint8_t num = 0; num < 2; num++) {
    Serial.print(F("CALIB MAG "));
    Serial.print((int)(num + 1));
    Serial.println(F(" -- move in figure 8s until I say stop!!! "));
    delay(500);
    state = IMU.calibrateMag();
    Serial.print((int)(num + 1));    
    if(state == 1){
        Serial.println(F("Mag calib done"));
    }
    else{
        Serial.println(F("Mag calib fail"));
    }
    Serial.println(F("Vals: "));    
    Serial.print(F("X: "));
    magdir[0][num] = IMU.getMagBiasX_uT();
    Serial.print(magdir[0][num], 6);
    Serial.print("uT");
    Serial.print(" / ");
    magdir[1][num] = IMU.getMagScaleFactorX();
    Serial.print(magdir[1][num], 6);
    Serial.println("uT");    
    Serial.print(F("Y: "));
    magdir[2][num] = IMU.getMagBiasY_uT();
    Serial.print(magdir[2][num], 6);
    Serial.print("uT");
    Serial.print(" / ");
    magdir[3][num] = IMU.getMagScaleFactorY();
    Serial.print(magdir[3][num], 6);
    Serial.println("uT");        
    Serial.print(F("Z: "));
    magdir[4][num] = IMU.getMagBiasZ_uT();
    Serial.print(magdir[4][num], 6);
    Serial.print("uT");
    Serial.print(" / ");
    magdir[5][num] = IMU.getMagScaleFactorZ();
    Serial.print(magdir[5][num], 6);
    Serial.println("uT");        
    if (num == 1) {
      magdir[0][2] = (magdir[0][0] + magdir[0][1]) / 2;
      magdir[1][2] = (magdir[1][0] + magdir[1][1]) / 2;
      magdir[2][2] = (magdir[2][0] + magdir[2][1]) / 2;
      magdir[3][2] = (magdir[3][0] + magdir[3][1]) / 2;
      magdir[4][2] = (magdir[4][0] + magdir[4][1]) / 2;
      magdir[5][2] = (magdir[5][0] + magdir[5][1]) / 2;
      Serial.println(F("Mag calib ALL done"));
      Serial.println(F("Vals: "));      
      Serial.print(F("X: "));
      Serial.print(magdir[0][2], 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(magdir[1][2], 6);
      Serial.println("uT");          
      Serial.print(F("Y: "));
      Serial.print(magdir[2][2], 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(magdir[3][2], 6);
      Serial.println("uT");          
      Serial.print(F("Z: "));
      Serial.print(magdir[4][2], 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(magdir[5][2], 6);
      Serial.println("uT");          
    }
  }
  AHRSFilter.begin(MFILTER_SAMPLE_FREQ_HZ);

  IMU.setGyroBiasX_rads(gyrodir[0][2]);
  IMU.setGyroBiasY_rads(gyrodir[1][2]);
  IMU.setGyroBiasZ_rads(gyrodir[2][2]);

  IMU.setAccelCalX(accdir[0][2], accdir[1][2]);
  IMU.setAccelCalY(accdir[2][2], accdir[3][2]);
  IMU.setAccelCalZ(accdir[4][2], accdir[5][2]);
  
  IMU.setMagCalX(magdir[0][2], magdir[1][2]);
  IMU.setMagCalY(magdir[2][2], magdir[3][2]);
  IMU.setMagCalZ(magdir[4][2], magdir[5][2]);
//  revision(accdir[0][2], accdir[1][2], accdir[2][2], accdir[3][2], accdir[4][2], accdir[5][2], magdir[0][2], magdir[1][2], magdir[2][2], magdir[3][2], magdir[4][2], magdir[5][2]);
  revision(gyrodir[0][2], gyrodir[1][2], gyrodir[2][2], accdir[0][2], accdir[1][2], accdir[2][2], accdir[3][2], accdir[4][2], accdir[5][2], magdir[0][2], magdir[1][2], magdir[2][2], magdir[3][2], magdir[4][2], magdir[5][2]);
  delay(2000);
  start_set();
//  timerAlarmEnable(timer);
  digitalWrite(RS485_E, LOW);  

}
float past_gx;
float past_gy;
float past_gz;

uint8_t loopCount = 0;
unsigned long ret_time;
void loop() {
  if(Serial2.available() > 0)
  {
    data_read = Serial2.readStringUntil('\n');
    if(data_read[0] == 's' && data_read[5] == ',' && data_read[9] == ',' && data_read[13] == ',')
    {
      int num = (data_read[1] - '0');
      q_value[num][0] = (((data_read[2] - '0') * 100 + (data_read[3] - '0') * 10 + (data_read[4] - '0')) / 100.0f) - 1.0f;
      q_value[num][1] = (((data_read[6] - '0') * 100 + (data_read[7] - '0') * 10 + (data_read[8] - '0')) / 100.0f) - 1.0f;
      q_value[num][2] = (((data_read[10] - '0') * 100 + (data_read[11] - '0') * 10 + (data_read[12] - '0')) / 100.0f) - 1.0f;
      q_value[num][3] = (((data_read[14] - '0') * 100 + (data_read[15] - '0') * 10 + (data_read[16] - '0')) / 100.0f) - 1.0f;
      roll_print[num] = atan2f(q_value[num][0]*q_value[num][1] + q_value[num][2]*q_value[num][3], 0.5f - q_value[num][1]*q_value[num][1] - q_value[num][2]*q_value[num][2]) * 57.29578f;
      pitch_print[num] = asinf(-2.0f * (q_value[num][1]*q_value[num][3] - q_value[num][0]*q_value[num][2])) * 57.29578f;
      yaw_print[num] = atan2f(q_value[num][1]*q_value[num][2] + q_value[num][0]*q_value[num][3], 0.5f - q_value[num][2]*q_value[num][2] - q_value[num][3]*q_value[num][3]) * 57.29578f + 180.0f;
      if (roll_print[num] < 0) {
        roll_print[num] += 360.0;
      }
      sprintf(data_print, "s%03d,%03d,%03d,%03d,%03d,%03d", (yaw_print[1]+500), (pitch_print[1]+500), (roll_print[1]+500), (yaw_print[2]+500), (pitch_print[2]+500), (roll_print[2]+500));
//      bt.println(data_print);
    }
  }
}

void Transdata_q(int id, int q0, int q1, int q2, int q3)
{
  byte _id = id;
  byte q0_H = (byte)((q0)/100);
  byte q0_L = (byte)((q0) - (((int)(q0)/100) * 100));
  byte q1_H = (byte)((q1)/100);
  byte q1_L = (byte)((q1) - (((int)(q1)/100) * 100));
  byte q2_H = (byte)((q2)/100);
  byte q2_L = (byte)((q2) - (((int)(q2)/100) * 100));
  byte q3_H = (byte)((q2)/100);
  byte q3_L = (byte)((q3) - (((int)(q3)/100) * 100));  
  byte checksum = ~lowByte(id + q0_H + q0_L + q1_H + q1_L + q2_H + q2_L + q3_L + q3_L);

  digitalWrite(RS485_E, HIGH);
  delay(1);    
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(_id);
  Serial2.write(q0_H);
  Serial2.write(q0_L);
  Serial2.write(q1_H);
  Serial2.write(q1_L);
  Serial2.write(q2_H);
  Serial2.write(q2_L);
  Serial2.write(q3_H);
  Serial2.write(q3_L);
  Serial2.write(checksum);
  Serial2.flush();
  delay(1);  
  digitalWrite(RS485_E, LOW);

  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(_id);
  Serial.write(q0_H);
  Serial.write(q0_L);
  Serial.write(q1_H);
  Serial.write(q1_L);
  Serial.write(q2_H);
  Serial.write(q2_L);
  Serial.write(q3_H);
  Serial.write(q3_L);
  Serial.write(checksum);
  Serial.flush();
}
