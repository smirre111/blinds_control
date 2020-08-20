/*
  LoRa Duplex communication wth callback

  Sends a message every half second, and uses callback
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Note: while sending, LoRa radio is not listening for incoming messages.
  Note2: when using the callback method, you can't use any of the Stream
  functions that rely on the timeout, such as readString, parseInt(), etc.

  created 28 April 2017
  by Tom Igoe
*/
#include <Arduino.h>
#include <FreeRTOS.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "FS.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <EasyButton.h>
#include <queue>
#include "SSD1306.h"
//#include <string>
#include <iostream>
#include <sstream>

//#include "driver/ledc.h"
#include "C:\Users\Reinhold\.platformio\packages\framework-espidf\components\driver\include\driver\ledc.h"

 
#if CONFIG_FREERTOS_UNICORE

#endif




#define BAND 433E6 //915E6
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1

#define LEDC_TEST_CH_NUM (2)
#define LEDC_TEST_DUTY (8000)
#define LEDC_TEST_FADE_TIME (3000)


// Pin assignments 

#ifdef NODEMCU_32S
static const int motorEnPin = 4;
static const int motorPwmUpPin = 5;
static const int motorPwmDownPin = 19;
static const int ctrlButtonUpPin = 25;
static const int ctrlButtonDownPin = 26;
#else
static const int motorEnPin = 2;
static const int motorPwmUpPin = 21;
static const int motorPwmDownPin = 17;
static const int ctrlButtonUpPin = 12;
static const int ctrlButtonDownPin = 13;
#endif

static const int oledSdaPin = 4;
static const int oledSclPin = 15;
static const int oledRst = 16;

static const int loraCsPin = 18;     // LoRa radio chip select
static const int loraResetPin = 14;  // LoRa radio reset
static const int loraIrqPin = 26;    // change for your board; must be a hardware interrupt pin
static const int loraSckPin = 5;
static const int loraMisoPin = 19;
static const int loraMosiPin = 27;



static const int motorRunTime = 8000000; //Runtime in uS
static const uint8_t oledAddress = 0x3c;




enum blinds_state_t
{
  BLINDS_OFF,
  BLINDS_UP,
  BLINDS_DOWN,
  BLINDS_CMD_UP,
  BLINDS_CMD_DOWN,
  BLINDS_STOP
};


enum blinds_motcmd_t
{
  MOTCMD_IDLE,
  MOTCMD_UP,
  MOTCMD_DOWN
};

enum blinds_syscmd_t
{
  SYSCMD_UP = 1,
  SYSCMD_DOWN,
  SYSCMD_ENABLE_WIFI,
  SYSCMD_DISABLE_WIFI,
  SYSCMD_OTA,
  SYSCMD_STATUS
};

//B0 ... destination address
//B1 ... destination subnet
//B2 ... sender address
//B3 ... message ID
//B4 ... payload length
//B5..N ... payload

//P0 ... command
//  01 ... up
//  02 ... down
//  03 ... enable wifi
//  04 ... disable wifi
//  05 ... ota
//  06 ... status
//  07 ... status_reply

//P1 ... status field
//  01 ... battery voltage
//  02 ...


static const byte broadcastAddressing = 0xFF;
static const byte subnetAddressing = 0xFE;
static const char *filename = "/config.txt"; // <- SD library uses 8.3 filenames







struct Config
{
  char hostname[64];
  uint8_t address;
  uint8_t subnet;
};

struct Packet {
  int destAddress;       // destAddress address
  int destSubnet;        // destSubnet address
  byte senderAddress;  // senderAddress address
  byte msgId;  // incoming msg ID
  byte payloadLength;  // incoming msg length
  byte *payload;
  
  Packet() {
    payload = new byte[128];
  } 

  String toString() {
    return String((char *)payload);
  }

};

//Global variables

String outgoing;          // outgoing message
byte msgCount = 0;        // count of outgoing messages
byte myAddress = 0x17;    // address of this device
byte mySubnet = 0x01;     // address of this device
byte destAddress = 0xFF;  // destination to send to
byte destSubnet = 0x00;   // destination to send to
long lastSendTime = 0;    // last send time
int interval = 2000;      // interval between sends

int buttonUpState = 0;
int buttonDownState = 0;

std::queue<blinds_syscmd_t> rxCmdQueue;
std::queue<blinds_syscmd_t> txCmdQueue;
std::queue<blinds_motcmd_t> motorCmdQueue;


Config config;                        // <- global configuration object

SSD1306 display(oledAddress, oledSdaPin, oledSclPin);

blinds_state_t blindsCurState = BLINDS_OFF;
blinds_state_t blindsNextState = BLINDS_OFF;

int brightness = 50; // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by

int cur_time = 0;
int last_time = 0;
int buttonUpPressed = 0;
int buttonDownPressed = 0;

int motorUp = 0;
int motorDown = 0;

volatile int timerElapsed = 0;



hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

int buttonPressDuration = 2000;
EasyButton buttonUp(ctrlButtonUpPin, 35, false, false);
EasyButton buttonDown(ctrlButtonDownPin, 35, false, false);

/*
  * Prepare and set configuration of timers
  * that will be used by LED Controller
  */
ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HS_MODE,           // timer mode
    .duty_resolution = LEDC_TIMER_13_BIT, //, // resolution of PWM duty
    .timer_num = LEDC_HS_TIMER,           // timer index
    .freq_hz = LEDC_BASE_FREQ,            // frequency of PWM signal
    .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
};

// Prepare individual configuration
// for each channel of LED Controller
// by selecting:
// - controller's channel number
// - output duty cycle, set initially to 0
// - GPIO number where LED is connected to
// - speed mode, either high or low
// - timer servicing selected channel
//   Note: if different channels use one timer,
//         then frequency and bit_num of these channels
//         will be the same
ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {

    {.gpio_num = motorPwmUpPin,
     .speed_mode = LEDC_HS_MODE,
     .channel = ledc_channel_t(LEDC_HS_CH0_CHANNEL),
     .intr_type = ledc_intr_type_t(0),
     .timer_sel = ledc_timer_t(LEDC_HS_TIMER),
     .duty = 0,
     .hpoint = 0},
    {.gpio_num = motorPwmDownPin,
     .speed_mode = LEDC_HS_MODE,
     .channel = ledc_channel_t(LEDC_HS_CH1_CHANNEL),
     .intr_type = ledc_intr_type_t(0),
     .timer_sel = ledc_timer_t(LEDC_HS_TIMER),
     .duty = 0,
     .hpoint = 0}};


void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  timerElapsed = 1;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  Serial.println("Timer elapsed!");
}

// Callback.
void onPressedForDurationButtonUp()
{
  Serial.println("Button up has been pressed for the given duration!");
  portENTER_CRITICAL(&buttonMux);
  buttonUpPressed = 1;
  motorCmdQueue.push(MOTCMD_UP);
  portEXIT_CRITICAL(&buttonMux);
}

// Callback.
void onPressedForDurationButtonDown()
{
  Serial.println("Button down has been pressed for the given duration!");
  portENTER_CRITICAL(&buttonMux);
  buttonDownPressed = 1;
  motorCmdQueue.push(MOTCMD_DOWN);
  portEXIT_CRITICAL(&buttonMux);
}

void motorUpFade()
{
  Serial.println("Call: Motor up ");
  const int ch = 0;
  ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                          ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
  ledc_fade_start(ledc_channel[ch].speed_mode,
                  ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
  timerRestart(timer);
  timerAlarmWrite(timer, motorRunTime, false);
  timerAlarmEnable(timer);
}

void motorDownFade()
{
  Serial.println("Call: Motor down ");
  const int ch = 1;
  ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                          ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
  ledc_fade_start(ledc_channel[ch].speed_mode,
                  ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
  timerRestart(timer);
  timerAlarmWrite(timer, motorRunTime, false);
  timerAlarmEnable(timer);
}

void motorStop()
{
  Serial.println("Call: Motor stop ");
  for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
  {
    ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
  }
}

#if 0
// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}
#endif

void setupPwmChannels()
{

  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);

  // Set LED Controller with previously prepared configuration
  for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
  {
    ledc_channel_config(&ledc_channel[ch]);
  }

  // Initialize fade service.
  ledc_fade_func_install(0);
}






void loadConfiguration(const char *filename, Config &config)
{
  // Open file for reading
  File file = SPIFFS.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  config.address = doc["address"] | 0x00;
  config.subnet = doc["subnet"] | 0x00;
  strlcpy(config.hostname,                 // <- destination
          doc["hostname"] | "example.com", // <- source
          sizeof(config.hostname));        // <- destination's capacity

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}

void saveConfiguration(const char *filename, const Config &config)
{
  // Delete existing file, otherwise the configuration is appended to the file
  SPIFFS.remove(filename);

  // Open file for writing
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<256> doc;

  // Set the values in the document
  doc["hostname"] = config.hostname;
  doc["address"] = config.address;
  doc["subnet"] = config.subnet;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}



void sendPacket(const Packet &packet) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(packet.destAddress);       // add destination address
  LoRa.write(packet.destSubnet);        // add destination subnet
  LoRa.write(packet.senderAddress);     // add senderAddress address
  LoRa.write(packet.msgId);             // add message ID
  LoRa.write(packet.payloadLength); // add payload length
  LoRa.write(packet.payload, packet.payloadLength);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID

}
 

// void sendMessage(String outgoing)
// {
//   LoRa.beginPacket();            // start packet
//   LoRa.write(destAddress);       // add destination address
//   LoRa.write(destSubnet);      // add senderAddress address
//   LoRa.write(myAddress);      // add senderAddress address
//   LoRa.write(msgCount);          // add message ID
//   LoRa.write(outgoing.length()); // add payload length
//   LoRa.print(outgoing);          // add payload
//   LoRa.endPacket();              // finish packet and send it
//   msgCount++;                    // increment message ID
// }





Packet rxPacket;
Packet txPacket;

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return

  // read packet header bytes:
  rxPacket.destAddress = LoRa.read();       // destAddress address
  rxPacket.destSubnet = LoRa.read();        // destSubnet address
  rxPacket.senderAddress = LoRa.read();         // senderAddress address
  rxPacket.msgId = LoRa.read();  // incoming msg ID
  rxPacket.payloadLength = LoRa.read(); // incoming msg length
  
  int  idx = 0;
  while (LoRa.available())
  {                                // can't use readString() in callback, so
    rxPacket.payload[idx++] = LoRa.read(); // add bytes one by one
  }
  rxPacket.payload[idx] = '\0';


  String incoming; // payload of packet
  incoming = rxPacket.toString();

  Serial.print("Incoming length: ");
  Serial.println(rxPacket.payloadLength);
  Serial.print("Length incoming: ");
  Serial.println(incoming.length());
  Serial.print("Message: ");
  Serial.println(rxPacket.toString());
  if (rxPacket.payloadLength != incoming.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return; // skip rest of function
  }

  // if the destAddress isn't this device or broadcast,
  if ((rxPacket.destAddress != myAddress) && (rxPacket.destAddress != broadcastAddressing))
  {
    Serial.println("This message is not for me.");
    return; // skip rest of function
  } 

  if ((rxPacket.destSubnet != mySubnet) && (rxPacket.destAddress != subnetAddressing))
  {
    Serial.println("This message is not for me.");
    return; // skip rest of function
  } 


  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(rxPacket.senderAddress, HEX));
  Serial.println("Sent to: 0x" + String(rxPacket.destAddress, HEX));
  Serial.println("Message ID: " + String(rxPacket.msgId));
  Serial.println("Message length: " + String(rxPacket.payloadLength));
  Serial.println("Message: " + incoming);
  Serial.println(rxPacket.toString());
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  //Serial.println("Snr: " + String(LoRa.packetSnr()));
  

  Serial.print("Received packet. ");
  // String line = "Received packet ";// + String(packetSize);
  // display.clear();
  // display.setFont(ArialMT_Plain_10);
  // display.drawString(3, 0, line);
  // //display.drawString(20, 22, "S: " + incoming);
  // display.display();
  Serial.println("Pushing command!");
  rxCmdQueue.push(blinds_syscmd_t(rxPacket.payload[0]));
  txCmdQueue.push(blinds_syscmd_t(rxPacket.payload[0]));

}


void publish() {


}

void processTxCommand() {

  if (!txCmdQueue.empty()) {
    Serial.println("TX Command queue processing");
    blinds_syscmd_t cmd = txCmdQueue.front();
    txCmdQueue.pop();
    
    std::stringstream ss;
    ss << "CmdOk:" << int(cmd) << std::endl;

    std::string test = ss.str();
    String message = test.c_str(); // send a message
    

    txPacket.destAddress = destAddress;       // destAddress address
    txPacket.destSubnet = destSubnet;        // destSubnet address
    txPacket.senderAddress = myAddress;         // senderAddress address
    txPacket.msgId = msgCount;  // incoming msg ID
    txPacket.payloadLength = message.length(); // incoming msg length
    strcpy((char *)txPacket.payload, message.c_str());

    // sendMessage(message);
    sendPacket(txPacket);
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(3, 5, F("Rcvr: packet "));
    display.display();
    Serial.println("Sending " + message);
  }
}

void processRxCommand() {

  if (!rxCmdQueue.empty()) {
    Serial.println("RX Command queue processing");
    blinds_syscmd_t cmd = rxCmdQueue.front();
    rxCmdQueue.pop();
    String msg;
    switch (cmd)
    {
    case SYSCMD_UP:
      msg = ("Blinds up!");
      portENTER_CRITICAL(&buttonMux);
      motorCmdQueue.push(MOTCMD_UP);
      portEXIT_CRITICAL(&buttonMux);
      break;
    case SYSCMD_DOWN:
      msg = ("Blinds down!");
      portENTER_CRITICAL(&buttonMux);
      motorCmdQueue.push(MOTCMD_DOWN);
      portEXIT_CRITICAL(&buttonMux);
      break;
    case SYSCMD_ENABLE_WIFI:
      msg = ("Wifi on!");
      break;
    case SYSCMD_DISABLE_WIFI:
      msg = ("Wifi off!");
      break;
    case SYSCMD_OTA:
      msg = ("Ota enabled!");
      break;
    case SYSCMD_STATUS:
      msg = ("Status!");
      break;
    default:
      msg = ("Not a valid command!");
    }
    Serial.print("Command: "); 
    Serial.println(msg); 
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(28, 22, msg);
    display.display();
  }


}



void setupLora() {

  LoRa.setPins(loraCsPin, loraResetPin, loraIrqPin); // set CS, reset, IRQ pin

  if (!LoRa.begin(BAND))
  { // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true); // if failed, do nothing
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSignalBandwidth(125e3);
  LoRa.setSyncWord(0x12);

  LoRa.onReceive(onReceive);
  LoRa.receive();


}

void setupDisplay() {

  pinMode(oledRst,OUTPUT);
  digitalWrite(oledRst, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(oledRst, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Receiver"); 
  display.display();
}

void fsmProcess();
void processRxCommand();
void processTxCommand();

void taskMotorContol(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    buttonUp.read();
    buttonDown.read();
    fsmProcess();
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}


void taskLoraProcessing(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    if (millis() - lastSendTime > interval)
    {
      processTxCommand();
      lastSendTime = millis();        // timestamp the message
      interval = random(2000) + 1000; // 2-3 seconds
      LoRa.receive();                 // go back into receive mode
    }

    processRxCommand();
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}


void setup()
{
  Serial.begin(115200); // initialize serial
  while (!Serial);

  pinMode(motorEnPin, OUTPUT);
  digitalWrite(motorEnPin, HIGH);

  pinMode(ctrlButtonUpPin, INPUT);
  pinMode(ctrlButtonDownPin, INPUT);

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, 8000000, false);
  // timerAlarmEnable(timer);



  Serial.println("LoRa Duplex with callback");

  if (!SPIFFS.begin())
  {
    Serial.println("Cannot access SPIFFS");
    return;
  }

  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);
  myAddress = config.address;
  mySubnet = config.subnet;
  Serial.print(F("Set address to..."));
  Serial.println(myAddress);

  const int ledPin = 25;

  pinMode(ledPin,OUTPUT); //Send success, LED will bright 1 second

  // Initialize the button.
  buttonUp.begin();
  buttonDown.begin();

  // Attach callback.
  buttonUp.onPressedFor(buttonPressDuration, onPressedForDurationButtonUp);
  buttonDown.onPressedFor(buttonPressDuration, onPressedForDurationButtonDown);

  setupPwmChannels();

  Serial.println("End of setup!");

  setupDisplay();


  SPI.begin(loraSckPin, loraMisoPin, loraMosiPin, loraCsPin);
  // override the default CS, reset, and IRQ pins (optional)

  setupLora();



  Serial.println("LoRa Initial OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(2000);


  xTaskCreatePinnedToCore(
    taskMotorContol
    ,  "taskMotorControl"   // A name just for humans
    ,  8192  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);

  xTaskCreatePinnedToCore(
    taskLoraProcessing
    ,  "taskLoraProcessing"   // A name just for humans
    ,  8192  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1);
}

void fsmProcess()
{
  blinds_motcmd_t cmd = MOTCMD_IDLE;
  portENTER_CRITICAL(&buttonMux);
  if (!motorCmdQueue.empty()) {
    cmd = motorCmdQueue.front();
    motorCmdQueue.pop();
  }
  portEXIT_CRITICAL(&buttonMux);

 
  switch (blindsCurState)
  {
  case BLINDS_OFF:
    //if (buttonUpPressed == 1)
    if (cmd == MOTCMD_UP)
    {
      blindsNextState = BLINDS_CMD_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Up pressed; BlindsUp");
      }
      motorUpFade();
    }
    //else if (buttonDownPressed == 1)
    else if (cmd == MOTCMD_DOWN)
    {
      blindsNextState = BLINDS_CMD_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Down pressed; BlindsDown");
      }
      motorDownFade();
    }
    if (blindsCurState != blindsNextState)
    {
      //motorStop();
    }
    break;
  case BLINDS_CMD_UP:
    //if (buttonDownPressed == 1)
    if (cmd == MOTCMD_DOWN)
    {
      blindsNextState = BLINDS_OFF;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Down pressed during BlindsUp");
      }
      motorStop();
    }
    else
    {
      if (timerElapsed == 1)
      {
        portENTER_CRITICAL(&timerMux);
        timerElapsed = 0;
        portEXIT_CRITICAL(&timerMux);
        blindsNextState = BLINDS_UP;
        motorStop();
      }
    }
    if (blindsCurState != blindsNextState)
    {
      //motorUpFade();
    }

    break;

  case BLINDS_CMD_DOWN:
    //if (buttonUpPressed == 1)
    if (cmd == MOTCMD_UP)
    {
      blindsNextState = BLINDS_OFF;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Up pressed during BlindsDown");
      }
      motorStop();
    }
    else
    {
      if (timerElapsed == 1)
      {
        portENTER_CRITICAL(&timerMux);
        timerElapsed = 0;
        portEXIT_CRITICAL(&timerMux);
        blindsNextState = BLINDS_DOWN;
        motorStop();
      }
    }
    if (blindsCurState != blindsNextState)
    {
      //motorDownFade();
    }
    break;

  case BLINDS_DOWN:
    //if (buttonUpPressed == 1)
    if (cmd == MOTCMD_UP)
    {
      blindsNextState = BLINDS_CMD_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Blinds are down; button Up pressed");
      }
      motorUpFade();
    }
    if (blindsCurState != blindsNextState)
    {
      //motorStop();
    }
    break;

  case BLINDS_UP:
    //if (buttonDownPressed == 1)
    if (cmd == MOTCMD_DOWN)
    {
      blindsNextState = BLINDS_CMD_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Blinds are up; button Down pressed");
      }
      motorDownFade();
    }
    if (blindsCurState != blindsNextState)
    {
      //motorStop();
    }
    break;
  case BLINDS_STOP:
    blindsNextState = BLINDS_OFF;
    if (blindsCurState != blindsNextState)
    {
      motorStop();
    }
    break;
  }

  if (blindsCurState != blindsNextState)
  {

    Serial.print("blindsCurState: ");
    Serial.print(blindsCurState);
    Serial.print(" blindsNextState: ");
    Serial.println(blindsNextState);
  }

  // if (timerElapsed > 0)
  // {
  //   portENTER_CRITICAL(&timerMux);
  //   timerElapsed = 0;
  //   portEXIT_CRITICAL(&timerMux);
  // }

  portENTER_CRITICAL(&buttonMux);
  buttonDownPressed = 0;
  buttonUpPressed = 0;
  portEXIT_CRITICAL(&buttonMux);

  blindsCurState = blindsNextState;
}


void loop()
{

  // buttonUp.read();
  // buttonDown.read();
  // fsmProcess();

  // if (millis() - lastSendTime > interval)
  // {
  //   processTxCommand();
  //   lastSendTime = millis();        // timestamp the message
  //   interval = random(2000) + 1000; // 2-3 seconds
  //   LoRa.receive();                 // go back into receive mode
  // }

  // processRxCommand();

}

