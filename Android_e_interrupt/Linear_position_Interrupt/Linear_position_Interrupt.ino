/*  
  GY-521  NodeMCU
  MPU6050 devkit 1.0
  board   Lolin         Description
  ======= ==========    ====================================================
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D8 (GPIO15)   Interrupt pin
*/

#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <FirebaseArduino.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <Ticker.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define FIREBASE_HOST "seb01-465f5.firebaseio.com"
#define FIREBASE_AUTH "pAAbm0SufQuio0OT0uxxhBXJYtCiggSBw0llqI0o"
#define WIFI_SSID "Celular Mila"
#define WIFI_PASSWORD "12345678"

// Aquisição a cada 50ms
//Obs.: Aumentar muito o intervalo de aquisição (500ms por exemplo) impede a leitura dos dados
#define READING_INTERVAL 50

#define MPU_INTERVAL 1000

#define AMOSTRAS 25

const char DEVICE_NAME[] = "mpu6050";

//NOTE: Antes de usar vc deve alterar a frequenciana biblioteca mpu6050
//CASO ISSO NAO SEJA FEITO CORRE PERIGO DA FIFO ESTOURAR
#define MPUsampFreq 40 //Hz

#define PSDMP 42 //Packet size DMP

Ticker ticker; //Controle da aquisição

Ticker ticker2; //Controle da publicação

MPU6050 mpu(0x68);
 
// MPU control/status vars
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
int numbPackets;
uint8_t fifoBuffer[42]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 a;         // [x, y, z]            accel sensor measurements
VectorInt16 aRaw;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
 
static float accel_x[2] = {0, 0}, vel_x[2] = {0, 0}, pos_x[2] = {0, 0};
static float accel[AMOSTRAS], vel[AMOSTRAS - 1], posicao;
int i = 0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
volatile bool flag = false, flag_publish = false;

unsigned long currentMillis = 0;
unsigned long previousMPUMillis = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
 
void reading(){
  flag = true;
  //Serial.println(flag);
}

void publish(){
  //Serial.println("publique");
  //Firebase.pushFloat("Temp", pos_x[1]);
}

void setupWifi(){
/*  
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset saved settings
  //wifiManager.resetSettings();

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect(DEVICE_NAME);

  Serial.print(F("WiFi connected! IP address: "));
  Serial.println(WiFi.localIP());
*/
  
  //Segunda opção para conexão
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  
}

void setupFirebase(){
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}
 
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
      iniciar_sensor_inercial();
      
  // initialize serial communication
  Serial.begin(115200);

  setupWifi();    

  setupFirebase();

  // Registra o ticker para ler de tempos em tempos
  ticker.attach_ms(READING_INTERVAL, reading);

  Serial.print(0.20);
  
  //Registra o ticker2 para publicar de tempos em tempos
  //ticker2.attach_ms(MPU_INTERVAL, publish);
}
 
void loop() {
 
 if(flag == true){
  //Serial.println("leia");
  ler_sensor_inercial();
  //Serial.print(accel_x[1]);
  //Serial.print(" ");
  //Serial.print(vel_x[1]);
  //Serial.print(" ");
  flag = false;
 }

 /*
 currentMillis = millis();
 if (currentMillis - previousMPUMillis >= MPU_INTERVAL) {
  previousMPUMillis = currentMillis;
  //Firebase.pushFloat("Temp", pos_x[1]);
  //Serial.println("publique");
  vel_x[0] = 0;
  pos_x[0] = 0;
 }
 */
}

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
   else if (numbPackets < 24){
    while (numbPackets > 0) {
      mpu.getFIFOBytes(fifoBuffer, PSDMP);
      numbPackets--;
    }
    //Serial.println("Ler");
    enviar_pacote_inercial();
    
  }

  accel[i] =  ((((float)a.y - (-32768)) * (2 - (-2)) / (32768 - (-32768)) + (-2))*9.81);

  //Serial.println(accel[i]);
  
  i++;

  if(i == AMOSTRAS){
    //flag == false;
    ticker.detach();
    float posicao;
    posicao = calculaPosicao();
    //Serial.println(posicao*100);
    ticker.attach_ms(READING_INTERVAL, reading);
    i = 0;
 }

}

void enviar_pacote_inercial() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&aRaw, fifoBuffer);
  mpu.dmpGetLinearAccel(&a, &aRaw, &gravity);
  //delay(50);
  //Serial.println(a.y);
}

float calculaPosicao(){
  int n = 0, cont[2] = {0,0};
  float velAux = 0;
  float pos = 0;
  
  while(n < AMOSTRAS){
    vel[n] = velAux + ((accel[n] + accel[n+1])*50)/2000;
    velAux = vel[n];
    //Serial.println(vel[n]);
    n++;
  }
  
  n = 1;

  
  while(true){
    if(vel[n] < vel[n-1] && vel[n] < vel[n+1]){
      if(cont[0] == 0)
        cont[0] = n;
      else{
        cont[1] = n;
        break;
      }
    }
    n++;
  }

  //Serial.print(vel[cont[0]]);
  //Serial.print(" ");
  //Serial.println(vel[cont[1]]);
  
  for(int j = cont[0]; j < cont[1]; j++){
    pos += ((vel[j] + vel[j+1])*50)/2000;
  }

  Serial.println(pos);
  
  return pos;
}
