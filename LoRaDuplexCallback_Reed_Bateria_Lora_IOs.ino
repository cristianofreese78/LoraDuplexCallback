/*
  LoRa Duplex communication with callback
  Sensor é acionado pelo reed switch
  1 - Sensor é ativado pelo reed switch. Habilita tranceptor Lora e acelerometro.
  2 - Fica monitorando tensão da bateria e aguardando comando para iniciar telemetria
  2 - Assim que receber comando da base (botão push botton pressionado) começa a enviar telemetria
  3 - Monitora novo comando da base (novamente botão push botton pressionado) para travar telemetria
*/

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <Bounce2.h>          // Biblioteca para tratar efeito boucing
#include <Adafruit_MPU6050.h> // Biblioteca do acelerometro/giroscopio
#include <Adafruit_Sensor.h>  // Biblioteca auxiliar do acelerometro/giroscopio
#include <Narcoleptic.h>      // Biblioteca Standby

# define RX_Monitor 14        // Pino RX da comunicação serial entre Arduino e GPS
# define TX_Monitor 15        // Pino TX da comunicação serial entre Arduino e GPS
# define SaidaLora 6          // Pino digital que irá ligar a antena Lora
# define SaidaI2C 7           // Pino digital que irá ligar o shield Giroscópio/Acelerometro I2C
# define Reed 4               // Pino de entrada utilizado para acordar emissor
# define Vibra 8              // Pino digital do vibracall
# define ADTensaoBateria 3    // Pino AD para leitura de tensão da bateria

const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin
const long END_ANTLORA = 433E6;   // Frequencia de operação da antena Lora. Neste caso 433Mhz

SoftwareSerial SerialMonitor(RX_Monitor,TX_Monitor);

Adafruit_MPU6050 AcelGiros;       // Objeto acelerometro/giroscopio
sensors_event_t Acel, Giros, Temp_MPU;  // Váriáveis que irão receber os dados de acelerometro/giroscopio
const float gyroXerror = 0.23;    // Offset giroscópio eixo x
const float gyroYerror = 0.51;    // Offset giroscópio eixo y
const float gyroZerror = 0.01;    // Offset giroscópio eixo z

volatile String comando;

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 200;          // interval between sends

long int tempo;

const uint32_t TempoStanby = 5E3; // Tempo de standby até Emissor receber ordem para acordar - 5s
Bounce bouncer = Bounce(Reed,5);  // Objeto bouncer para tratar boucing no pino Reed

const float RefBateria = 3.2;     // Comparativo de tensão da bateria com a referência que impede operação abaixo de 3,2V
float TensaoBateriaGlobal;        // Usada para enviar a tensão da bateria para a serial e pelo receptor Lora

bool Dormindo=true;               // Status que identifica se o emissor continua dormindo e economizando energia
bool InitPerifericos=false;       // Status que identifica se periféricos foram inicializados corretamente para iniciar telemetria

void setup() 
{
  analogReference(INTERNAL);      // Define referência interna em 1,1V no conversor AD
  pinMode(Reed, INPUT);           // Configura o pino Reed como entrada
  pinMode(SaidaI2C, OUTPUT);      // Configuração pino Acelerometro/Giroscópio
  pinMode(SaidaLora, OUTPUT);     // Configuração pino antena Lora
  pinMode(Vibra, OUTPUT);         // Configuração pino vibracall
      
  InitSaidasPerifericos();        // Inicializa todas as saidas de controle de periféricos
  
  SerialMonitor.begin(9600);                   // initialize serial
  while (!SerialMonitor);

  SerialMonitor.println(F("LoRa Duplex with callback - INICIANDO SENSOR"));
}

// Inicializa saídas de controle dos periféricos para nível baixo
void InitSaidasPerifericos()
{
  digitalWrite(SaidaI2C, LOW);    
  digitalWrite(SaidaLora, LOW);    
  digitalWrite(Vibra, LOW); 
  delay(500);  
}

// Função de leitura de tensão da bateria. Se estiver acima da referência permite operação do sensor retornando true 
bool StatusBateria() 
  {
  float TensaoBat = 0;
  SerialMonitor.print("BT - ");
  TensaoBat = TensaoBateria();
  SerialMonitor.print(TensaoBat); SerialMonitor.print("V - "); 
  if (TensaoBat >= RefBateria) {SerialMonitor.println("OK"); return true;}  
  else {SerialMonitor.println("NO"); return false;}
  }  

// Função para medir a tensão da bateria. Realiza leitura na entrada analógica e converte em tensão depois de realizar média aritmética em amostragem
float TensaoBateria()
  {
   float total=0;  
   int AMOSTRAS=12;
   float TensaoBat = 0;
   for (int i=0; i<AMOSTRAS; i++) { total += 1.0 * analogRead(ADTensaoBateria); delay(5); }
   TensaoBat = ((total / (float)AMOSTRAS) * (float)(4.14 / 1024)); 
   return TensaoBat; 
  }

// Função de inicialização da antena Lora - Envia status inicial ao receptor Lora se inicializou corretamente
bool InitLora()
  { digitalWrite(SaidaLora, HIGH); delay(100);

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    SerialMonitor.println("LORA OFF ERRO INI");
    while (true);                       // if failed, do nothing
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();
  sendMessage("LORA - ON");
  SerialMonitor.println("Sending LORA - ON");
  return true;}

// Função que verifica se houve pulso no Reed e se bateria OK, retornando true se precisar acordar. Possui funcionalidade para evitar debouncer no reed.
// Inicia interrupção temporizada e seta OscilaVibra para alternar estado da saida Vibracall a cada tratamento de interrupção
bool StatusFicarDormindo()
  {
   bool BateriaOK;
   //BateriaOK = StatusBateria();
   for (int i=0; i<20; i++) 
   {
    if (bouncer.update() && bouncer.read() == LOW) 
    { 
      if (StatusBateria()) {digitalWrite(Vibra, HIGH); delay(500); digitalWrite(Vibra, LOW); return false; }
      else {digitalWrite(Vibra, HIGH); delay(300); digitalWrite(Vibra, LOW); delay(300); digitalWrite(Vibra, HIGH); delay(300); digitalWrite(Vibra, LOW); delay(300); return true;}
    }
    delay(10);     
   }
   return true;
  }

// Função que verifica se houve pulso no Reed e se bateria ok, retornando false se precisar voltar a dormir. Possui funcionalidade para evitar debouncer no reed.
// Desliga interrupção temporizada
bool StatusFicarTravado()
  {
    if ((bouncer.update() && bouncer.read() == LOW) || (!StatusBateria())){ digitalWrite(Vibra, HIGH); delay(1000); digitalWrite(Vibra, LOW);  return false; }
    else return true; 
  }

// Função de inicialização do Acelerometro/Giroscópio - Envia status ao receptor Lora
bool InitAcel_Giros()
  {
   digitalWrite(SaidaI2C, HIGH); delay(50); 
   if (AcelGiros.begin()) 
   {
    AcelGiros.setAccelerometerRange(MPU6050_RANGE_4_G);  // Range de 16g
    AcelGiros.setGyroRange(MPU6050_RANGE_500_DEG);        // Range de 250 graus/segundo
    AcelGiros.setFilterBandwidth(MPU6050_BAND_21_HZ);     // Filtro de banda de 21Hz
    sendMessage("GIR_ACE - ON");
    SerialMonitor.println("Sending GIR_ACE - ON");
    LoRa.receive();
    return true;
   }   
   else {SerialMonitor.println("Sending GIR_ACE - OFF"); sendMessage("GIR_ACE - OFF"); LoRa.receive(); delay(50); return false;}
  }  

void loop() 
{
  while (Dormindo)  // Fica em looping enquanto não houver solicitação para acordar via pulso no Reed e bateria OK na função StatusFicarDormindo
  {
   Narcoleptic.delay(TempoStanby);
   Dormindo = StatusFicarDormindo();
   delay(10); 
  }

  if (!InitPerifericos) // Se ainda não inicializou perifericos então testa sucesso da inicialização
  {
      delay(2000);
      if (InitLora()&&InitAcel_Giros()) 
      {
       sendMessage("TELEMETRIA PAUSADA");
       LoRa.receive();
       SerialMonitor.println("Sending TELEMETRIA PAUSADA");
       InitPerifericos = true;
      } 
      else
      {
       InitPerifericos = false;
       SerialMonitor.println("ERRO AO INICIALIZAR PERIFERICOS - VOLTARÁ A DORMIR");
       Dormindo = true;
       InitSaidasPerifericos(); 
      }
  } 
  if (InitPerifericos) // Se todos os periféricos inicializaram então inicia telemetria
  {  
      // Niveis de aceleração e orientação nos eixos x,y,z
      AcelGiros.getEvent(&Acel, &Giros, &Temp_MPU);

      if (!comando.compareTo("TP")) tempo = millis();
      if ((millis() - lastSendTime > interval) && (!comando.compareTo("TI"))){
      String message;   // send a message
      message.concat((float)(millis()-tempo)/1000); message.concat(";");
      message.concat((Acel.acceleration.x)-gyroXerror); message.concat(";"); message.concat((Acel.acceleration.y)-gyroYerror); message.concat(";"); message.concat((Acel.acceleration.z)-gyroZerror); message.concat(";");
      message.concat(Giros.gyro.x); message.concat(";"); message.concat(Giros.gyro.y); message.concat(";"); message.concat(Giros.gyro.z); 
      sendMessage(message);
      SerialMonitor.println("Sending " + message);
      lastSendTime = millis();            // timestamp the message
      LoRa.receive();                     // go back into receive mode
      }
  }           
  Dormindo = !StatusFicarTravado();  // Se o reed for acionado ou a bateria ficar abaixo da tensão de referência então a função irá retornar false
  // Se Dormindo for true, então volta a dormir e desliga periféricos
  if (Dormindo)                      
  {
      InitPerifericos = false;
      SerialMonitor.println("LORA OFF - GIR_ACE OFF");
      sendMessage("Sending LORA OFF - GIR_ACE OFF");
      InitSaidasPerifericos(); 
      delay(100);
  }
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  }

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    SerialMonitor.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    SerialMonitor.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  SerialMonitor.println("Received from: 0x" + String(sender, HEX));
  SerialMonitor.println("Sent to: 0x" + String(recipient, HEX));
  SerialMonitor.println("Message ID: " + String(incomingMsgId));
  SerialMonitor.println("Message length: " + String(incomingLength));
  SerialMonitor.println("Message: " + incoming);
  SerialMonitor.println("RSSI: " + String(LoRa.packetRssi()));
  SerialMonitor.println("Snr: " + String(LoRa.packetSnr()));
  SerialMonitor.println();
  comando = incoming;
}
