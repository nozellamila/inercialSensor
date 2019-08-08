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

#define PSDMP 42 //Packet size DMP

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead


uint32_t timer = 0, timer2;
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
  ler_sensor_inercial(); //Realiza leitura e envia pacote(ou mostra) dados
}

////////////////////
//Sensor Inercial //
////////////////////

void iniciar_sensor_inercial() {
  if (mpu.testConnection()) {
    mpu.initialize(); //Initializes the IMU
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
  dt = timer - timer2;
  
  //ay =  abs(ay);
  
  accel_x[1] = (((float)ay)/16834)*9.8;

//  if (accel_x[1] > -3 && accel_x[1] < 3){
//    accel_x[1] = 0;
//  }
  
  vel_x[1] = vel_x[0] + ((accel_x[1] + accel_x[0])*dt)/2000;

  soma = soma + vel_x[1];

  media = soma/n;

  vel_x_zero[1] = vel_x[1] - media;

  pos_x[1] = pos_x[0] + ((vel_x_zero[1] + vel_x_zero[0])*dt)/2000;

  soma1 = soma1 + pos_x[1];

  media1 = soma1/n;

  pos_x_zero[1] = pos_x[1] - media1;
  
  accel_x[0] = accel_x[1];
  vel_x[0] = vel_x[1];
  pos_x[0] = pos_x[1];
  vel_x_zero[0] = vel_x_zero[1];
  n++;
  timer2 = millis();

  SI = (int) (pos_x[1]*10000);
  SH = SI / 256;
  SL = SI - SH*256;
  
  yprI = (int) (accel_x[1]*10000);
  yprH = yprI / 256;
  yprL = yprI - yprH*256;
  /*
  Serial.print("acel ");
  Serial.println(accel_x[1]); //USAR ESSE
  Serial.print("vel ");
  Serial.println(vel_x[1]);
  Serial.print("pos ");
  Serial.println(pos_x[1]);
  Serial.print("delta ");
  Serial.println(dt);
  */
  Serial.print(vel_x[1]);
  Serial.print(" ");
  Serial.println(pos_x[1]);
}

void enviar_pacote_inercial() {
  timer = millis();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

}
