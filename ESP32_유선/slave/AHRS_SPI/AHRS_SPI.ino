//SLAVE
#include <SPI.h>
#include "MPU9250.h"

#define CHIP_SELECT_PIN         13

#define IMU_POLL_DELAY_MS         2
#define MFILTER_SAMPLE_FREQ_HZ    480
#define IMU_LOWPASSFILTER_BANDWIDTH   MPU9250::DLPF_BANDWIDTH_184HZ

const byte gpio_input = 5;
#define RXD1 34
#define TXD1 32

#define RXD_RS485 33
#define TXD_RS485 25
#define RS485_E 26
int use_IMU = 1;

#define ACC 0xF0
#define MAG 0xF1
#define GYRO 0xF2

int count;
char data_print[50];

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if(count == use_IMU){
    digitalWrite(RS485_E, HIGH);
    Serial2.println(data_print);
//    Serial.println(data_print);
    Serial2.flush();
    digitalWrite(RS485_E, LOW);
  }
  else if(count == 10){
    count = 0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  count++;
}

MPU9250 IMU(SPI, gpio_input);
Madgwick AHRSFilter;

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
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);  //1s = 1000000, 1ms = 10000 0.1ms = 10000 
  pinMode(RS485_E, OUTPUT);

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
  Serial.print("ID: ");
  Serial.println(use_IMU);
  Recive_revision();
  delay(2000);
  Recive_start();
  timerAlarmEnable(timer);
}
float past_gx;
float past_gy;
float past_gz;

uint8_t loopCount = 0;
unsigned long ret_time;

void loop() {
  IMU.readSensor(AHRSFilter);

  if (loopCount++ > 10) {
    loopCount = 0;
    float rolly = AHRSFilter.getRoll();
    if (rolly < 0) {
      // I like them positive
      rolly = 360.0 + rolly;
    }
    AHRSValues.roll = rolly;
    AHRSValues.pitch = AHRSFilter.getPitch();
    AHRSValues.yaw = AHRSFilter.getYaw();    
    int q0 = AHRSFilter.getq0();
    int q1 = AHRSFilter.getq1();
    int q2 = AHRSFilter.getq2();
    int q3 = AHRSFilter.getq3();
    sprintf(data_print, "s%d%03d,%03d,%03d,%03d", use_IMU, q0, q1, q2, q3);

//    Serial.print("R: ");
//    Serial.print(rolly, 2);
//    Serial.print("\tP: ");
//    Serial.print(AHRSValues.pitch , 2);
//    Serial.print("\tY: ");
//    Serial.println(AHRSValues.yaw, 2);
  }
  delay(IMU_POLL_DELAY_MS);
}

void Transdata_euler(int id, int yaw, int pitch, int roll)
{
  byte _id = id;
  byte yaw_H = (byte)((yaw+500)/100);      
  byte yaw_L = (byte)((yaw+500) - (((int)(yaw+500)/100) * 100));
  byte pitch_H = (byte)((pitch+500)/100);
  byte pitch_L = (byte)((pitch+500) - (((int)(pitch+500)/100) * 100));
  byte roll_H = (byte)((roll+500)/100);
  byte roll_L = (byte)((roll+500) - (((int)(roll+500)/100) * 100));
  byte checksum = ~lowByte(id + yaw_H + yaw_L + pitch_H + pitch_L + roll_H + roll_L);

  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(_id);
  Serial1.write(yaw_H);
  Serial1.write(yaw_L);
  Serial1.write(pitch_H);
  Serial1.write(pitch_L);
  Serial1.write(roll_H);
  Serial1.write(roll_L);
  Serial1.write(checksum);
  Serial1.flush();

  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(_id);
  Serial.write(yaw_H);
  Serial.write(yaw_L);
  Serial.write(pitch_H);
  Serial.write(pitch_L);
  Serial.write(roll_H);
  Serial.write(roll_L);
  Serial.write(checksum);
  Serial.flush();

}
int Recivedata(byte* buf, uint8_t n)
{
  digitalWrite(RS485_E, LOW);
  delay(1);
  uint8_t bufIdx = 2;
  uint8_t check = 0;
  byte c;
  byte checksum = 0;

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

int Recive_revision(void)
{
  byte ACC_CHECK = 0;
  byte MAG_CHECK = 0;
  byte GYRO_CHECK = 0;
  const byte bufSize_revision = 28;
  byte buf_revision[bufSize_revision];
  while(1)
  {
    if(Recivedata(buf_revision, bufSize_revision))
    {
      if(buf_revision[2] == ACC)
      {
        float X_H = (float)((((int)buf_revision[3]-100)*1000000 + ((int)buf_revision[4]-100)*10000 + ((int)buf_revision[5]-100)*100 + ((int)buf_revision[6]-100))/1000000.0f)-50;
        float Y_H = (float)((((int)buf_revision[7]-100)*1000000 + ((int)buf_revision[8]-100)*10000 + ((int)buf_revision[9]-100)*100 + ((int)buf_revision[10]-100))/1000000.0f)-50;
        float Z_H = (float)((((int)buf_revision[11]-100)*1000000 + ((int)buf_revision[12]-100)*10000 + ((int)buf_revision[13]-100)*100 + ((int)buf_revision[14]-100))/1000000.0f)-50;
        float X_L = (float)((((int)buf_revision[15]-100)*1000000 + ((int)buf_revision[16]-100)*10000 + ((int)buf_revision[17]-100)*100 + ((int)buf_revision[18]-100))/1000000.0f)-50;
        float Y_L = (float)((((int)buf_revision[19]-100)*1000000 + ((int)buf_revision[20]-100)*10000 + ((int)buf_revision[21]-100)*100 + ((int)buf_revision[22]-100))/1000000.0f)-50; 
        float Z_L = (float)((((int)buf_revision[23]-100)*1000000 + ((int)buf_revision[24]-100)*10000 + ((int)buf_revision[25]-100)*100 + ((int)buf_revision[26]-100))/1000000.0f)-50;

        IMU.setAccelCalX(X_H, X_L);
        IMU.setAccelCalY(Y_H, Y_L);
        IMU.setAccelCalZ(Z_H, Z_L);
        Serial.println("ACC");
        Serial.print(F("X: "));
        Serial.print(X_H,6);
        Serial.print("m/s^2");        
        Serial.print(" / ");
        Serial.print(X_L,6);
        Serial.println("m/s^2");                
        Serial.print(F("Y: "));
        Serial.print(Y_H,6);
        Serial.print("m/s^2");                
        Serial.print(" / ");        
        Serial.print(Y_L,6);
        Serial.println("m/s^2");                
        Serial.print(F("Z: "));
        Serial.print(Z_H,6);
        Serial.print("m/s^2");                
        Serial.print(" / ");        
        Serial.print(Z_L,6);
        Serial.println("m/s^2");        
        Serial.println();                        
        ACC_CHECK = 1;
      }
      else if(buf_revision[2] == MAG)
      {
        float X_H = (float)((((int)buf_revision[3]-100)*1000000 + ((int)buf_revision[4]-100)*10000 + ((int)buf_revision[5]-100)*100 + ((int)buf_revision[6]-100))/1000000.0f)-50;
        float Y_H = (float)((((int)buf_revision[7]-100)*1000000 + ((int)buf_revision[8]-100)*10000 + ((int)buf_revision[9]-100)*100 + ((int)buf_revision[10]-100))/1000000.0f)-50;
        float Z_H = (float)((((int)buf_revision[11]-100)*1000000 + ((int)buf_revision[12]-100)*10000 + ((int)buf_revision[13]-100)*100 + ((int)buf_revision[14]-100))/1000000.0f)-50;
        float X_L = (float)((((int)buf_revision[15]-100)*1000000 + ((int)buf_revision[16]-100)*10000 + ((int)buf_revision[17]-100)*100 + ((int)buf_revision[18]-100))/1000000.0f)-50;
        float Y_L = (float)((((int)buf_revision[19]-100)*1000000 + ((int)buf_revision[20]-100)*10000 + ((int)buf_revision[21]-100)*100 + ((int)buf_revision[22]-100))/1000000.0f)-50; 
        float Z_L = (float)((((int)buf_revision[23]-100)*1000000 + ((int)buf_revision[24]-100)*10000 + ((int)buf_revision[25]-100)*100 + ((int)buf_revision[26]-100))/1000000.0f)-50;

        IMU.setMagCalX(X_H, X_L);
        IMU.setMagCalY(Y_H, Y_L);
        IMU.setMagCalZ(Z_H, Z_L);
        Serial.println("MAG");
        Serial.print(F("X: "));
        Serial.print(X_H,6);
        Serial.print("uT");        
        Serial.print(" / ");        
        Serial.print(X_L,6);
        Serial.println("uT");                
        Serial.print(F("Y: "));
        Serial.print(Y_H,6);
        Serial.print("uT");                
        Serial.print(" / ");        
        Serial.print(Y_L,6);
        Serial.println("uT");                
        Serial.print(F("Z: "));
        Serial.print(Z_H,6);
        Serial.print("uT");                
        Serial.print(" / ");        
        Serial.print(Z_L,6);
        Serial.println("uT");        
        Serial.println();                        
        MAG_CHECK = 1;      
      }
      else if(buf_revision[2] == GYRO)
      {
        float X_H = (float)((((int)buf_revision[3]-100)*1000000 + ((int)buf_revision[4]-100)*10000 + ((int)buf_revision[5]-100)*100 + ((int)buf_revision[6]-100))/1000000.0f)-50;
        float Y_H = (float)((((int)buf_revision[7]-100)*1000000 + ((int)buf_revision[8]-100)*10000 + ((int)buf_revision[9]-100)*100 + ((int)buf_revision[10]-100))/1000000.0f)-50;
        float Z_H = (float)((((int)buf_revision[11]-100)*1000000 + ((int)buf_revision[12]-100)*10000 + ((int)buf_revision[13]-100)*100 + ((int)buf_revision[14]-100))/1000000.0f)-50;
        IMU.setGyroBiasX_rads(X_H);
        IMU.setGyroBiasY_rads(Y_H);
        IMU.setGyroBiasZ_rads(Z_H);
        Serial.println("GYRO");
        Serial.print(F("X: "));
        Serial.print(X_H,6);
        Serial.println("rad/s");        
        Serial.print(F("Y: "));
        Serial.print(Y_H,6);
        Serial.println("rad/s");                
        Serial.print(F("Z: "));
        Serial.print(Z_H,6);
        Serial.println("rad/s");
        Serial.println();
        GYRO_CHECK = 1;      
      }      
      if(ACC_CHECK && MAG_CHECK && GYRO_CHECK)
      {
        return 1;
      }
    }
  }
}

int Recivedata_start(byte* buf, uint8_t n)
{
  digitalWrite(RS485_E, LOW);
  delay(1);
  uint8_t bufIdx = 2;
  uint8_t check = 0;
  byte c;
  byte checksum = 0;
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

int Recive_start(void)
{
  const byte bufSize_start = 3;
  byte buf_start[bufSize_start];
  while(1)
  {
    if(Recivedata_start(buf_start, bufSize_start) && buf_start[2] == 0xFE)
    {
      return 1;
    }
  }
}
