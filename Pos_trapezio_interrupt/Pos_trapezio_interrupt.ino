// Biblioteca para comunicação I2C
#include <Wire.h>

// Endereço I2C do acelerometro
const int MPU_addr=0x68; 

// Variavel para armazenar o tempo
unsigned long tempo;
// Vetor para guardar os valores de aceleração
float aceleracoes[100];
// Vetor para guardar os valores de velocidade
float velocidades[100];
// Variavel para armazenar quantas amostras ja foram pegas
char contagem=0;

void setup(){
  velocidades[0] = 0;
  Wire.begin(); //Inicia a comunicação I2C
  Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
  Wire.write(0x6B); // registrador PWR_MGMT_1
  Wire.write(0); // Manda 0 e "acorda" o MPU 6050
  Wire.endTransmission(true);

  Serial.begin(19200); //Inicia a comunicaçao serial (para exibir os valores lidos)
}

void loop(){
  // Variavel para armazenar o valor medido
  int AcX; 

  // Fica lendo o sensor até o valor em x ser maior que 800
  while (abs(AcX) < 800){
    // Mede o tempo total no inicio da medição e mais a frente subtrai para saber o tempo percorrido
    tempo = micros();
    
    Wire.beginTransmission(MPU_addr); // Começa a transmissao de dados para o sensor
    Wire.write(0x3B); // Registrador dos dados em x (ACCEL_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(MPU_addr,2); // Faz um "pedido" para ler 2 registradores

    // Se estiverem dados disponíveis para leitura
    if(Wire.available()<=2) { 
      AcX=Wire.read()<<8; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcX=AcX|Wire.read();  
    }
    
  }

  AcX = abs(AcX);
  // Converte o valor medido
  aceleracoes[contagem] = (((float)AcX - (-32768)) * (2 - (-2)) / (32768 - (-32768)) + (-2))*9.81;
  // Calcula o tempo percorrido
  tempo = micros()-tempo;
  
  //Se já fez 100 mediçoes, faz o calculo do deslocamento
  if(contagem == 99){
    contagem=0;
    float posicao;
    posicao += calculoTrapezio(tempo)*100; // A função retorna o valor em metros, então multiplico por 100 para converter para centímetros
    
    Serial.print("\nMoveu ");
    Serial.print(posicao,6);
    Serial.println(" cms em x.");
    
  }else{
    contagem++;
  }

  
}

float calculoTrapezio(unsigned long tempo){
  float posicao=0;
  
  // Calcula as velocidades pela regra do trapézio
  for(int i=1; i<100; i++){
    velocidades[i] =  (aceleracoes[i-1]+aceleracoes[i])*tempo/2000000;
    velocidades[i] += velocidades[i-1];
  }

  // Calcula o deslocamento pela regra do trapézio
  for(int i=0; i<99; i++){
    posicao +=  (velocidades[i]+velocidades[i+1])*tempo/2000000;
  }

  //Armazena a ultima velocidade para ser a primeira de todas da próxima medição
  velocidades[0] = velocidades[99];

  return posicao;
}
