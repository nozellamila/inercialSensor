//Final sketch for linear position
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include <Wire.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define DEBUG
#define LED_PIN 13

//NOTE: Antes de usar vc deve alterar a frequenciana biblioteca mpu6050
//CASO ISSO NAO SEJA FEITO CORRE PERIGO DA FIFO ESTOURAR
#define MPUsampFreq 40 //Hz
#define mpu_interval 25 //Each 10ms

#define PSDMP 42 //Packet size DMP

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead


//TODO: trocar esses millis por timer
unsigned long currentMillis = 0;
unsigned long previousMPUMillis = 0;
uint32_t timer = 0;
double dt;

//Variaveis Inercial
MPU6050 mpu(0x68);
uint8_t fifoBuffer[42], yprH, yprL, SH, SL ; // FIFO storage fifoBuffer of mpu
int numbPackets;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];
float accel_x[2] = {0, 0}, vel_x[2] = {0, 0}, pos_x[2] = {0, 0}, soma = 0, soma1 = 0, media, media1, vel_x_zero[2] = {0, 0}, pos_x_zero[1] = {0}, i = 0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset, yprI, SI, n = 1, a = 0, b = 0;

int16_t ax, ay, az, gx, gy, gz;

void setup() {
  //Sensor Inercial
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(200000); //NOTE: Ajustar de acordo com arduino utilizado
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  iniciar_sensor_inercial();

  //Serial:
  Serial.begin(115200);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMPUMillis >= mpu_interval) {
    previousMPUMillis = currentMillis;
    ler_sensor_inercial(); //Realiza leitura e envia pacote(ou mostra) dados
  }
  

}

////////////////////
//Sensor Inercial //
////////////////////

void iniciar_sensor_inercial() {
  if (mpu.testConnection()) {
    mpu.initialize(); //Initializes the IMU
    uint8_t ret = mpu.dmpInitialize(); //Initializes the DMP
    delay(50);
    if (ret == 0) {
      mpu.setDMPEnabled(true);
      mpu.setDLPFMode(3);
      b = mpu.getFullScaleAccelRange();
      //trocar
      ax_offset = mpu.getXAccelOffset();
      ay_offset = mpu.getYAccelOffset();
      az_offset = mpu.getZAccelOffset();
      gx_offset = mpu.getXGyroOffset();
      gy_offset = mpu.getYGyroOffset();
      gz_offset = mpu.getZGyroOffset();
      
      mpu.setXAccelOffset(ax_offset);
      mpu.setYAccelOffset(ay_offset);
      mpu.setZAccelOffset(az_offset);
      mpu.setXGyroOffset(gx_offset);
      mpu.setYGyroOffset(gy_offset);
      mpu.setZGyroOffset(gz_offset);
      Serial.println("Sensor Inercial configurado com sucesso.\n");
    } else {
      //TODO: adicionar uma forma melhor de aviso. outro led?
      Serial.println("Erro na inicializacao do sensor Inercial!\n");
    }
  } else {
    Serial.println("Erro na conexao do sensor Inercial.\n");
  }
}

void ler_sensor_inercial() {
  numbPackets = floor(mpu.getFIFOCount() / PSDMP);
  if (numbPackets >= 24) {
    mpu.resetFIFO();
    DEBUG_PRINT("FIFO sensor 1 overflow!\n"); //TODO: mostrar isso de alguma forma. outro led?
  } else {
    while (numbPackets > 0) {
      mpu.getFIFOBytes(fifoBuffer, PSDMP);
      numbPackets--;
    }
    enviar_pacote_inercial();
  }
}

void enviar_pacote_inercial() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //ax = mpu.getAccelerationX ();

  dt = (double)(millis() - timer); // Calculate delta time
  timer = millis();

  i =  (float) ay;
  
  accel_x[1] = (i/16834)*9.8;

//  if (accel_x[1] > -3 && accel_x[1] < 3){
//    accel_x[1] = 0;
//  }
  
  vel_x[1] = vel_x[0] + (accel_x[1] - accel_x[0])*dt;

  soma = soma + vel_x[1];

  media = soma/n;

  vel_x_zero[1] = vel_x[1] - media;

  pos_x[1] = pos_x[0] + (vel_x_zero[1] - vel_x_zero[0])*dt;

  soma1 = soma1 + pos_x[1];

  media1 = soma1/n;

  pos_x_zero[1] = pos_x[1] - media1;
  
  accel_x[0] = accel_x[1];
  vel_x[0] = vel_x[1];
  pos_x[0] = pos_x[1];
  vel_x_zero[0] = vel_x_zero[1];
  n++;
  

  SI = (int) (pos_x[1]*10000);
  SH = SI / 256;
  SL = SI - SH*256;
  
  yprI = (int) (accel_x[1]*10000);
  yprH = yprI / 256;
  yprL = yprI - yprH*256;
  
    Serial.print("acel ");
    Serial.println(accel_x[1]); //USAR ESSE
    Serial.print("pos ");
    Serial.println(pos_x[1]);
    Serial.print("delta ");
    Serial.println(dt);
//    Serial.print("\t");
//  Serial.write(0x7E);
//  Serial.write(yprH);
//  Serial.write(yprL);
//  Serial.write(SH);
//  Serial.write(SL);
//  Serial.print("accel e pos inteiro\t");
//  Serial.print(SI);
//  Serial.print("\t");
//  Serial.print(yprI);
//  Serial.print("\n");
//  Serial.print("aceleracao\t");
//  Serial.println(accel_x[1]);
//  Serial.print("\t");
//  Serial.println(pos_x[1]);
//  Serial.print("\n");
//  Serial.print("aceleracao ypr\t");
//  Serial.println(accel_x[1]*10000);
//  Serial.print("\t");
 // Serial.println(ypr[0]);
//  Serial.print("\n");
//Serial.write(yprI);
 // Serial.write(SI);
//  Serial.write(0x81);
  
//  delay(100);
//  Serial.print("\t");
//  Serial.print(kalAngleX);
//  Serial.println(S);

  
 // Serial.println("Offsets\t");
//  Serial.print(ax_offset); //USAR ESSE
//  Serial.print("\t");
//  Serial.print(ay_offset);
//  Serial.print("\t");
//  Serial.println(ax_offset);
//  Serial.print(gz_offset);
//  Serial.print("\t");

}
