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
#define INTERRUPT_PIN 2

//NOTE: Antes de usar vc deve alterar a frequenciana biblioteca mpu6050
//CASO ISSO NAO SEJA FEITO CORRE PERIGO DA FIFO ESTOURAR
#define MPUsampFreq 40 //Hz

#define PSDMP 42 //Packet size DMP

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead


// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

//Variaveis Inercial
MPU6050 mpu(0x68);
uint8_t fifoBuffer[42], yprH, yprL, SH, SL ; // FIFO storage fifoBuffer of mpu
int numbPackets;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 a;
VectorInt16 aRaw;

float accel_x[2] = {0, 0}, vel_x[2] = {0, 0}, pos_x[2] = {0, 0}, soma = 0, soma1 = 0, media, media1, vel_x_zero[2] = {0, 0}, pos_x_zero[1] = {0}, n = 0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset, yprI, SI;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  //Serial.println("Interrupt");
  mpuInterrupt = true;
}

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
  pinMode(INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  
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
    uint8_t ret = mpu.dmpInitialize(); //Initializes the DMP
    delay(50);
    if (ret == 0) {
      mpu.setDMPEnabled(true);
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
  } 
   else if (mpuInterrupt == true && numbPackets < 24){
    while (numbPackets > 0) {
      mpu.getFIFOBytes(fifoBuffer, PSDMP);
      numbPackets--;
    }
    //Serial.println("Ler");
      enviar_pacote_inercial();
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  accel_x[1] = (((float)a.y - (-32768)) * (2 - (-2)) / (32768 - (-32768)) + (-2))*9.81;
 
  vel_x[1] = vel_x[0] + ((accel_x[1] + accel_x[0])*1)/2000;

  pos_x[1] = pos_x[0] + ((vel_x[1] + vel_x[0])*1)/2000;
  
  accel_x[0] = accel_x[1];
  vel_x[0] = vel_x[1];
  pos_x[0] = pos_x[1];

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
  Serial.println(accel_x[1]);
  
}

void enviar_pacote_inercial() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&aRaw, fifoBuffer);
  mpu.dmpGetLinearAccel(&a, &aRaw, &gravity);
  //Serial.println("Enviar");
}
