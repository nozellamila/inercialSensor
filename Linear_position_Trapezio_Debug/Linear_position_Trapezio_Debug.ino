//Final sketch for linear position
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include <Wire.h>
#include <avr/sleep.h>  
#include <Wire.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//#define DEBUG
#define LED_PIN 13


#define amostras 50

uint32_t timer = 0;
double dt;

//Variaveis Inercial
MPU6050 mpu(0x68);

int16_t ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

int16_t ax = 0, ay, az, gx, gy, gz;
float aceleracoes[amostras], velocidades[amostras], valores_media[10];
int contagem = 0;

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.begin();
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer

}

//example showing using readbytev   ----    readByte(MPU6050_ADDRESS, GYRO_CONFIG);
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                            // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
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
  
}

uint16_t readdata;

void loop() {
  calculaPosicao(); //Acumula dados e calcula posicao
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
      //mpu.setDLPFMode(3);
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

void lerAcc(){
  timer = micros();
  //Serial.println(timer);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  
}

void calculaPosicao() {

    lerAcc();
    
    ay = abs(ay);
    
    aceleracoes[contagem] = (((float)ay - (-32768)) * (2 - (-2)) / (32768 - (-32768)) + (-2))*9.81;
   // Serial.println(aceleracoes[contagem]);
    timer = micros() - timer;
    
    if(contagem == amostras){
    contagem=0;
    float posicao;
    posicao += calculoTrapezio(timer)*100; // A função retorna o valor em metros, então multiplico por 100 para converter para centímetros
   // Serial.println(posicao);
    }else{
    contagem++;
    }
 }

float calculoTrapezio(unsigned long tempo){
  float posicao = 0;

  // Calcula as velocidades pela regra do trapézio
  for(int i=1; i<amostras; i++){
    velocidades[i] =  (aceleracoes[i-1]+aceleracoes[i])*tempo/2000000;
    velocidades[i] = mediaMovel(velocidades[i]);
    Serial.println(velocidades[i]);
    velocidades[i] += velocidades[i-1];
  }

  // Calcula o deslocamento pela regra do trapézio
  for(int i=0; i<amostras - 1; i++){
    posicao +=  (velocidades[i]+velocidades[i+1])*tempo/2000000;
    //mediaMovel(posicao);
  }

  //Armazena a ultima velocidade para ser a primeira de todas da próxima medição
  velocidades[0] = velocidades[amostras - 1];

  return posicao;
}

float mediaMovel(float valor){
  int i;               //variável auxiliar para iterações
  float acc = 0;        //acumulador
  
  //Desloca o vetor completamente eliminando o valor mais antigo
  for(i = 10; i > 0; i--) valores_media[i] = valores_media[i-1];    
  
  valores_media[0] = valor;           //carrega o sinal no primeiro elemento do vetor
  
 // long sum = 0;            //Variável para somatório
  
  for(i = 0; i < 10; i++) acc += valores_media[i];

  return(valor = acc/10);
}
