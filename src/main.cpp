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
//#include "SSD1306.h"
#include "SSD1306Wire.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include <WiFiMulti.h> // Built-in
#include <ArduinoOTA.h>
//#include <string>
#include <iostream>
#include <sstream>

#ifdef DEBUG_PORT
#define DEBUG_MSG(...) DEBUG_PORT.printf(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

//#define T_BEAM_V10

// Pin assignments
#ifdef T_BEAM_V10
#define I2C_SDA 21 //FIXME
#define I2C_SCL 22 //FIXME
#else
#define I2C_SDA 4  //FIXME
#define I2C_SCL 15 //FIXME
#endif
#define SSD1306_ADDRESS 0x3C
#define SSD1306_RST_PIN 16

#define APP_NAME "TTN MAP-TRACK"
#define APP_VERSION "1.2.0"
#define LOGO_DELAY 5000 // Time to show logo on first boot
//SSD1306 display(oledAddress, oledSdaPin, oledSclPin);
//SSD1306Wire *display; moved to Screen class

#include "screen.h"
#include "sleep.h"

#include <driver/adc.h>
//#include "driver/ledc.h"
#include "C:\Users\Reinhold\.platformio\packages\framework-espidf\components\driver\include\driver\ledc.h"

//#ifdef T_BEAM_V10
#include "axp20x.h"

AXP20X_Class axp;
bool pmu_irq = false;
String baChStatus = "No charging";
//#endif

typedef WiFiMulti WiFiMulti_t;
WiFiMulti_t wifiMulti;

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
#define LEDC_START_DUTY (6000)
#define LEDC_FINAL_DUTY (8000)
#define LEDC_START_FADE_TIME (3000)
#define LEDC_STOP_FADE_TIME (1000)

#define GPS_POWER_CTRL_CH 3
#define LORA_POWER_CTRL_CH 2
#define PMU_IRQ 35

#define DIO0_GPIO 26
#define DIO1_GPIO 33 // Note: not really used on this board
#define DIO2_GPIO 32 // Note: not really used on this board

#ifdef NODEMCU_32S
static const int motorEnPin = 4;
static const int motorPwmUpPin = 5;
static const int motorPwmDownPin = 19;
static const int ctrlButtonUpPin = 25;
static const int ctrlButtonDownPin = 26;
static const int ctrlButtonStopPin = 26;
#else
static const int motorEnPin = 2;
static const int motorPwmUpPin = 21;
static const int motorPwmDownPin = 17;
static const int ctrlButtonUpPin = 39;
static const int ctrlButtonDownPin = 38;
static const int ctrlButtonStopPin = 37;
#endif

static const int oledSdaPin = 4;
static const int oledSclPin = 15;
static const int oledRst = 16;

static const int loraCsPin = 18;    // LoRa radio chip select
static const int loraResetPin = 14; // LoRa radio reset
static const int loraIrqPin = 26;   // change for your board; must be a hardware interrupt pin
static const int loraSckPin = 5;
static const int loraMisoPin = 19;
static const int loraMosiPin = 27;

static const int buttonPressDuration = 2000;
static const uint8_t oledAddress = 0x3c;

const char *ssid = "FRITZ!Box 7590 TD";
const char *password = "23461468396986328867";

enum blinds_state_t
{
  BLINDS_OFF,
  BLINDS_UP,
  BLINDS_DOWN,
  BLINDS_CMD_STEP_UP,
  BLINDS_CMD_STEP_DOWN,
  BLINDS_CMD_FULL_UP,
  BLINDS_CMD_FULL_DOWN,
  BLINDS_STOP
};

enum blinds_motcmd_t
{
  MOTCMD_IDLE,
  MOTCMD_FULL_UP,
  MOTCMD_FULL_DOWN,
  MOTCMD_STEP_UP,
  MOTCMD_STEP_DOWN,
  MOTCMD_STOP
};

// enum blinds_syscmd_t
// {
//   SYSCMD_UP = 1,
//   SYSCMD_DOWN,
//   SYSCMD_ENABLE_WIFI,
//   SYSCMD_DISABLE_WIFI,
//   SYSCMD_OTA,
//   SYSCMD_STATUS
// };

typedef uint8_t blinds_syscmd_base_t;

struct BlindsSysCmd
{
  static const blinds_syscmd_base_t SYSCMD_UP = '1';
  static const blinds_syscmd_base_t SYSCMD_DOWN = '2';
  static const blinds_syscmd_base_t SYSCMD_ENABLE_WIFI = '3';
  static const blinds_syscmd_base_t SYSCMD_DISABLE_WIFI = '4';
  static const blinds_syscmd_base_t SYSCMD_OTA = '5';
  static const blinds_syscmd_base_t SYSCMD_STATUS = '6';
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

const IPAddress local_IP(192, 168, 178, 151); // Set your server's fixed IP address here
const IPAddress gateway(192, 168, 178, 1);    // Set your network Gateway usually your Router base address
const IPAddress subnet(255, 255, 255, 0);     // Set your network sub-network mask here
const IPAddress dns(192, 168, 178, 1);        // Set your network DNS usually your Router base address

struct Config
{
  char hostname[64];
  uint8_t address;
  uint8_t subnet;
  uint32_t motorRuntime;
};

struct Packet
{
  int destAddress;    // destAddress address
  int destSubnet;     // destSubnet address
  byte senderAddress; // senderAddress address
  byte msgId;         // incoming msg ID
  byte payloadLength; // incoming msg length
  byte *payload;

  Packet()
  {
    payload = new byte[128];
  }

  String toString()
  {
    return String((char *)payload);
  }
};

//Global variables

String outgoing;         // outgoing message
byte msgCount = 0;       // count of outgoing messages
byte destAddress = 0xFF; // destination to send to
byte destSubnet = 0x00;  // destination to send to
long lastSendTime = 0;   // last send time
int interval = 2000;     // interval between sends

byte cfgAddress = 0x17;             // address of this device
byte cfgSubnet = 0x01;              // address of this device
uint32_t cfgMotorRunTime = 8000000; //Runtime in uS

bool ssd1306_found = false;
bool axp192_found = false;
bool arduinoOtaActive = false;

bool cadModeActive = false;

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause; // the reason we booted this time

std::queue<blinds_syscmd_base_t> rxCmdQueue;
std::queue<blinds_syscmd_base_t> txCmdQueue;
std::queue<blinds_motcmd_t> motorCmdQueue;

Config config; // <- global configuration object

blinds_state_t blindsCurState = BLINDS_OFF;
blinds_state_t blindsNextState = BLINDS_OFF;

int brightness = 50; // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by

int cur_time = 0;
int last_time = 0;

int motorUp = 0;
int motorDown = 0;

static volatile int timerElapsed = 0;
static volatile bool interruptHappenedDIO0 = false;
static volatile bool interruptHappenedDIO1 = false;

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
volatile SemaphoreHandle_t xBinarySemaphoreDIO0;
volatile SemaphoreHandle_t xBinarySemaphoreDIO1;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE interruptMuxDIO0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE interruptMuxDIO1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

EasyButton buttonUpHigh(ctrlButtonUpPin, 35, false, false);
EasyButton buttonDownHigh(ctrlButtonDownPin, 35, false, false);
EasyButton buttonStop(ctrlButtonStopPin, 35, false, false);
Screen *screen;
AsyncUDP udp;
#define CAD
//#ifndef CAD
LoRaClass LoRa;
//#endif

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

void taskLoraProcessing(void *pvParameters);
void taskOtaProcessing(void *pvParameters);
void taskCurrentSensing(void *pvParameters);

void fsmProcess();
void processRxCommand();
void processTxCommand();

void IRAM_ATTR isr_Timer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  timerElapsed = 1;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  //Serial.println("Timer elapsed!");
}

void IRAM_ATTR isr_pinLoraDIO0()
{
  portENTER_CRITICAL_ISR(&interruptMuxDIO0);
  interruptHappenedDIO0 = true;
  portEXIT_CRITICAL_ISR(&interruptMuxDIO0);
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xBinarySemaphoreDIO0, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken != pdFALSE)
  {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR isr_pinLoraDIO1()
{
  portENTER_CRITICAL_ISR(&interruptMuxDIO1);
  interruptHappenedDIO1 = true;
  portEXIT_CRITICAL_ISR(&interruptMuxDIO1);
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xBinarySemaphoreDIO1, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken != pdFALSE)
  {
    portYIELD_FROM_ISR();
  }
}

// Callback.
void onPressedForDurationButtonUp()
{
  Serial.println("Button up has been pressed for the given duration!");
  portENTER_CRITICAL(&buttonMux);
  motorCmdQueue.push(MOTCMD_FULL_UP);
  portEXIT_CRITICAL(&buttonMux);
}

void onPressedButtonUpHigh()
{
  Serial.println("Button up has been pressed!");
  portENTER_CRITICAL(&buttonMux);
  motorCmdQueue.push(MOTCMD_STEP_UP);
  portEXIT_CRITICAL(&buttonMux);
}

// Callback.
void onPressedForDurationButtonDown()
{
  Serial.println("Button down has been pressed for the given duration!");
  portENTER_CRITICAL(&buttonMux);
  motorCmdQueue.push(MOTCMD_FULL_DOWN);
  portEXIT_CRITICAL(&buttonMux);
}

void onPressedButtonDownHigh()
{
  Serial.println("Button down has been pressed!");
  portENTER_CRITICAL(&buttonMux);
  motorCmdQueue.push(MOTCMD_STEP_DOWN);
  portEXIT_CRITICAL(&buttonMux);
}

void onPressedForDurationButtonStop()
{
  Serial.println("Button stop has been pressed for the given duration!");
  portENTER_CRITICAL(&buttonMux);
  motorCmdQueue.push(MOTCMD_STOP);
  portEXIT_CRITICAL(&buttonMux);
}

void onPressedButtonStop()
{
  Serial.println("Button stop has been pressedn!");
  portENTER_CRITICAL(&buttonMux);
  motorCmdQueue.push(MOTCMD_STOP);
  portEXIT_CRITICAL(&buttonMux);
}

void motorUpFade(uint32_t motorRunTime)
{
  Serial.println("Call: Motor up ");
  const int ch = 0;
  digitalWrite(motorEnPin, HIGH);

  // ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_START_DUTY);
  // ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
  ledc_set_duty_and_update(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_START_DUTY, 0);

  ledc_set_fade_with_time(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_FINAL_DUTY, LEDC_START_FADE_TIME);
  ledc_fade_start(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);

  timerRestart(timer);
  timerAlarmWrite(timer, motorRunTime, false);
  timerAlarmEnable(timer);
}

void motorDownFade(uint32_t motorRunTime)
{
  Serial.println("Call: Motor down ");
  const int ch = 1;
  digitalWrite(motorEnPin, HIGH);

  // ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_START_DUTY);
  // ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
  ledc_set_duty_and_update(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_START_DUTY, 0);

  ledc_set_fade_with_time(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_FINAL_DUTY, LEDC_START_FADE_TIME);
  ledc_fade_start(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);

  timerRestart(timer);
  timerAlarmWrite(timer, motorRunTime, false);
  timerAlarmEnable(timer);
}

void motorStop()
{
  Serial.println("Call: Motor stop ");

  timerStop(timer);

  for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
  {
    // ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
    // ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
    // ledc_set_duty_and_update(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0, 0);

    ledc_set_fade_with_time(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0, LEDC_STOP_FADE_TIME);
    ledc_fade_start(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
  }
  vTaskDelay(LEDC_STOP_FADE_TIME); //FIXME: Check if we really can delay here...
  digitalWrite(motorEnPin, LOW);
}

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
  config.motorRuntime = doc["motorRuntime"] | 0x00000000;
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
  doc["motorRuntime"] = config.motorRuntime;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

void sendPacket(const Packet &packet)
{
  LoRa.beginPacket();                               // start packet
  LoRa.write(packet.destAddress);                   // add destination address
  LoRa.write(packet.destSubnet);                    // add destination subnet
  LoRa.write(packet.senderAddress);                 // add senderAddress address
  LoRa.write(packet.msgId);                         // add message ID
  LoRa.write(packet.payloadLength);                 // add payload length
  LoRa.write(packet.payload, packet.payloadLength); // add payload
  LoRa.endPacket();                                 // finish packet and send it
  msgCount++;                                       // increment message ID
}

// void sendMessage(String outgoing)
// {
//   LoRa.beginPacket();            // start packet
//   LoRa.write(destAddress);       // add destination address
//   LoRa.write(destSubnet);      // add senderAddress address
//   LoRa.write(cfgAddress);      // add senderAddress address
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
  {
    Serial.println("Packet is empty");
    return; // if there's no packet, return
  }

  // read packet header bytes:
  rxPacket.destAddress = LoRa.read();   // destAddress address
  rxPacket.destSubnet = LoRa.read();    // destSubnet address
  rxPacket.senderAddress = LoRa.read(); // senderAddress address
  rxPacket.msgId = LoRa.read();         // incoming msg ID
  rxPacket.payloadLength = LoRa.read(); // incoming msg length

  int idx = 0;
  while (LoRa.available())
  {                                        // can't use readString() in callback, so
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
  if ((rxPacket.destAddress != cfgAddress) && (rxPacket.destAddress != broadcastAddressing))
  {
    Serial.println("This message is not for me.");
    return; // skip rest of function
  }

  if ((rxPacket.destSubnet != cfgSubnet) && (rxPacket.destAddress != subnetAddressing))
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
  rxCmdQueue.push(blinds_syscmd_base_t(rxPacket.payload[0]));
  txCmdQueue.push(blinds_syscmd_base_t(rxPacket.payload[0]));
}

void setupWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    vTaskDelay(5000);
    ESP.restart();
  }
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupWifi(WiFiClass &WiFi, WiFiMulti_t &wifiMulti, const Config &cfg)
{
  // Setting up WiFi ************************************************************
  //*****************************************************************************
  if (!WiFi.config(local_IP, gateway, subnet, dns))
  { //WiFi.config(ip, gateway, subnet, dns1, dns2);
    Serial.println("WiFi STATION Failed to configure Correctly");
  }
  wifiMulti.addAP(ssid, password); // add Wi-Fi networks you want to connect to, it connects strongest to weakest
  //wifiMulti.addAP(ssid_2, password_2); // Adjust the values in the Network tab
  //wifiMulti.addAP(ssid_3, password_3);
  //wifiMulti.addAP(ssid_4, password_4); // You don't need 4 entries, this is for example!

  Serial.println("Connecting ...");
  while (wifiMulti.run() != WL_CONNECTED)
  { // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
    delay(250);
    Serial.print('.');
  }

  // Setting up Udp ************************************************************
  //*****************************************************************************
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  //udp.begin(localPort);

  // Setting up mDNS for the webserver ************************************************************
  //*****************************************************************************
  Serial.println("\nConnected to " + WiFi.SSID() + " Use IP address: " + WiFi.localIP().toString()); // Report which SSID and IP is in use
  // The logical name http://fileserver.local will also access the device if you have 'Bonjour' running or your system supports multicast dns
  if (!MDNS.begin(cfg.hostname))
  { // Set your preferred server name, if you use "myserver" the address would be http://myserver.local/
    Serial.println(F("Error setting up MDNS responder!"));
    ESP.restart();
  }
  MDNS.addService("http", "tcp", 80);
}

void setupUdp()
{
  if (udp.listen(1234))
  {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());
    });
  }
}

void shutdownWifi()
{
  MDNS.end();
  WiFi.mode(WIFI_OFF);

  btStop();
}
void setupOTA(const Config &cfg)
{
  setupWifi();
  //setupWifi(WiFi, wifiMulti, cfg);

  // Port defaults to 3232
  ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(cfg.hostname);

  // No authentication by default
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        SPIFFS.end();
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();
}

//Probably this is not needed as OTA will reboot anyway?
void shutdownOTA(const Config &cfg)
{
  ArduinoOTA.end();
  arduinoOtaActive = false;
}

void processTxCommand()
{

  if (!txCmdQueue.empty())
  {
    Serial.println("TX Command queue processing");
    blinds_syscmd_base_t cmd = txCmdQueue.front();
    txCmdQueue.pop();

    std::stringstream ss;
    ss << "CmdOk:" << char(cmd) << std::endl;

    std::string test = ss.str();
    String message = test.c_str(); // send a message

    txPacket.destAddress = destAddress;        // destAddress address
    txPacket.destSubnet = destSubnet;          // destSubnet address
    txPacket.senderAddress = cfgAddress;       // senderAddress address
    txPacket.msgId = msgCount;                 // incoming msg ID
    txPacket.payloadLength = message.length(); // incoming msg length
    strcpy((char *)txPacket.payload, message.c_str());

    // sendMessage(message);
    sendPacket(txPacket);
    screen->printAck();

    Serial.println("Sending " + message);
  }
}

void processRxCommand()
{

  if (!rxCmdQueue.empty())
  {
    Serial.println("RX Command queue processing");
    blinds_syscmd_base_t cmd = rxCmdQueue.front();
    rxCmdQueue.pop();
    String msg;
    switch (cmd)
    {
    case BlindsSysCmd::SYSCMD_UP:
      msg = ("Blinds up!");
      portENTER_CRITICAL(&buttonMux);
      motorCmdQueue.push(MOTCMD_FULL_UP);
      portEXIT_CRITICAL(&buttonMux);
      break;
    case BlindsSysCmd::SYSCMD_DOWN:
      msg = ("Blinds down!");
      portENTER_CRITICAL(&buttonMux);
      motorCmdQueue.push(MOTCMD_FULL_DOWN);
      portEXIT_CRITICAL(&buttonMux);
      break;
    case BlindsSysCmd::SYSCMD_ENABLE_WIFI:
      setupWifi();
      msg = ("Wifi on!");
      break;
    case BlindsSysCmd::SYSCMD_DISABLE_WIFI:
      shutdownWifi();
      msg = ("Wifi off!");
      break;
    case BlindsSysCmd::SYSCMD_OTA:
      setupOTA(config);
      xTaskCreate(taskOtaProcessing, "taskOtaProcessing", 1024, NULL, 2, NULL);
      msg = ("Ota enabled!");
      break;
    case BlindsSysCmd::SYSCMD_STATUS:
      setupWifi();
      setupUdp();
      xTaskCreate(taskCurrentSensing, "taskCurrentSensing", 1024, NULL, 2, NULL);
      msg = ("Status!");
      break;
    default:
      msg = ("Not a valid command!");
    }

    Serial.print("Command: ");
    Serial.println(msg);
    screen->printString(msg);
    // screen->_display->clear();
    // screen->_display->setFont(ArialMT_Plain_10);
    // screen->_display->setTextAlignment(TEXT_ALIGN_LEFT);
    // screen->_display->drawString(28, 22, msg);
    // screen->_display->display();
  }
}

void setupLora()
{

  LoRa.setPins(loraCsPin, loraResetPin, loraIrqPin); // set CS, reset, IRQ pin

  if (!LoRa.begin(BAND))
  { // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ; // if failed, do nothing
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(128);
  LoRa.setSignalBandwidth(125e3);
  LoRa.setSyncWord(0x12);

#ifndef CAD
  //Register receive callback
  LoRa.onReceive(onReceive);
  //Put receiver in continuous RX mode
  LoRa.receive();
#else
  // For other constants, like FHSS change channel, CRC error, or RX timeout, see the LoRa.h header file.
  // Choose from LORA_IRQ_DIOx_ variants and use this "x" number in place of the first parameter.
  // Not all DIOx and interrupt type mixes are possible.

  pinMode(DIO0_GPIO, INPUT);
  pinMode(DIO1_GPIO, INPUT);
  LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);
  LoRa.setInterruptMode(0 /* DIO0 */, LORA_IRQ_DIO0_CADDONE);
  LoRa.setInterruptMode(1 /* DIO1 */, LORA_IRQ_DIO1_RXTIMEOUT);
  // Launch our ISR function when LoRa interrupt happens
  attachInterrupt(digitalPinToInterrupt(DIO0_GPIO), isr_pinLoraDIO0, RISING);
  attachInterrupt(digitalPinToInterrupt(DIO1_GPIO), isr_pinLoraDIO1, RISING);

  // Start LoRa CAD process
  cadModeActive = true;
  LoRa.cad();
#endif
  Serial.println("Starting LoRa succeeded");
}

void setupDisplay()
{

  pinMode(oledRst, OUTPUT);
  digitalWrite(oledRst, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(oledRst, HIGH);
  screen->setupAndPrint();
}

void taskMotorContol(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    buttonUpHigh.read();
    buttonDownHigh.read();
    buttonStop.read();

    fsmProcess();
    vTaskDelay(100); // one tick delay (15ms) in between reads for stability
  }
}

void taskLoraProcessing(void *pvParameters) // This is a task.
{
  (void)pvParameters;

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
    vTaskDelay(1 / portTICK_PERIOD_MS); // one tick delay (15ms) in between reads for stability
  }
}

void taskLoraProcessingDIO0(void *pvParameters)
{
  (void)pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    xSemaphoreTake(xBinarySemaphoreDIO0, portMAX_DELAY);

    if (cadModeActive == true)
    {

      uint8_t loraInterrupts = LoRa.readInterrupts();
      //Serial.print("DIO0 (cadMode) interrups: ");
      //Serial.println(loraInterrupts);
      //CAD detected -> chnage to RX mode
      if (loraInterrupts & LORA_IRQ_FLAG_CAD_DETECTED)
      {
        // Here use LORA_IRQ_FLAG_* variants from LoRa.h
        LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE | LORA_IRQ_FLAG_RX_TIMEOUT);
        LoRa.clearInterrupts(0xff);
        LoRa.setInterruptMode(0 /* DIO0 */, LORA_IRQ_DIO0_RXDONE);
        LoRa.setSymbolTimeout(255);
        cadModeActive = false;
        //LoRa.rxSingle();
        LoRa.parsePacket();
        last_time = millis();
        Serial.println("CAD done, detected going into RX mode");
      }
      else //Only CAD done, restart CAD
      {
        //Serial.println("CAD done, restarting CAD!");
        LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);
        LoRa.clearInterrupts(0xff);
        cadModeActive = true;
        LoRa.cad();
      }
    }
    else //No CAD mode, RX mode
    {

      uint8_t loraInterrupts = LoRa.readInterrupts();
      Serial.print("DIO0 (RX mode) interrups: ");
      Serial.println(loraInterrupts);

      if (loraInterrupts & LORA_IRQ_FLAG_RX_DONE)
      {
        Serial.println("RX done interrupt");

        int packetSize = LoRa.parsePacket(); // Put into RXSINGLE mode
        onReceive(packetSize);
        if (packetSize > 0)
        {
          Serial.println("Packet available, setting back to CAD mode");
          LoRa.setInterruptMode(0 /* DIO0 */, LORA_IRQ_DIO0_CADDONE);
          cadModeActive = true;
          LoRa.cad();
        }

        LoRa.clearInterrupts(LORA_IRQ_FLAG_RX_DONE);
      }
    }
  }
}

void taskLoraProcessingDIO1(void *pvParameters)
{
  (void)pvParameters;
  for (;;) // A Task shall never return or exit.
  {

    xSemaphoreTake(xBinarySemaphoreDIO1, portMAX_DELAY);

    uint8_t loraInterrupts = LoRa.readInterrupts();
    Serial.print("DIO1 interrups: ");
    Serial.println(loraInterrupts);

    if (loraInterrupts & LORA_IRQ_FLAG_RX_TIMEOUT)
    {
      Serial.println("RX timeout happened, restarting CAD!");
      Serial.print("Timeout: ");
      Serial.println(LoRa.getSymbolTimeout());
      cur_time = millis();
      Serial.print("Time from RX:");
      Serial.println(last_time-cur_time);
    }

    LoRa.clearInterrupts(LORA_IRQ_FLAG_RX_TIMEOUT);
    LoRa.clearInterrupts(0xff);
    LoRa.setInterruptMode(0 /* DIO0 */, LORA_IRQ_DIO0_CADDONE);
    //TX timeout interrupt
    //No RX happened in time -> we should restart CAD
    cadModeActive = true;
    LoRa.cad();
  }
}

void taskOtaProcessing(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    if (arduinoOtaActive == true)
    {
      ArduinoOTA.handle();
    }

    vTaskDelay(20);
  }
}

void taskCurrentSensing(void *pvParameters)
{
  (void)pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    AsyncUDPMessage test;
    //uint16_t val = analogRead(36);
    int val = adc1_get_raw(ADC1_CHANNEL_0) + 1000;
    // esp_err_t r = adc2_get_raw( ADC2_CHANNEL_0, ADC_WIDTH_12Bit, &val);
    // if ( r == ESP_OK ) {
    //     Serial.println (val );
    // } else if ( r == ESP_ERR_TIMEOUT ) {
    //     Serial.println("ADC error: ");
    // }

    uint8_t *test1 = (uint8_t *)&val;

    test.write(test1, 2);

    udp.broadcast(test1, 2);
    //udp.broadcast("Anyone here?");
    Serial.print("Val: ");
    Serial.println(val);
    vTaskDelay(100);
  }
}

void setupSPIFFS()
{
  if (!SPIFFS.begin())
  {
    Serial.println("Cannot access SPIFFS");
    return;
  }
}

void doDeepSleep(uint64_t msecToWake)
{
  Serial.printf("Entering deep sleep for %llu seconds\n", msecToWake / 1000);

  // not using wifi yet, but once we are this is needed to shutoff the radio hw
  // esp_wifi_stop();

  screen->screenOff(); // datasheet says this will draw only 10ua
  //LMIC_shutdown(); // cleanly shutdown the radio
  LoRa.sleep();
  if (axp192_found)
  {
    // turn on after initial testing with real hardware
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS main power
  }

  // FIXME - use an external 10k pulldown so we can leave the RTC peripherals powered off
  // until then we need the following lines
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  // Only GPIOs which are have RTC functionality can be used in this bit map: 0,2,4,12-15,25-27,32-39.
  uint64_t gpioMask = (1ULL << ctrlButtonUpPin) | (1ULL << ctrlButtonDownPin) | (1ULL << ctrlButtonStopPin);

  // FIXME change polarity so we can wake on ANY_HIGH instead - that would allow us to use all three buttons (instead of just the first)
  //gpio_pullup_en((gpio_num_t) ctrlButtonUpPin); //FIXME external pulldowns there

  esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);

  esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL); // call expects usecs
  esp_deep_sleep_start();                              // TBD mA sleep current (battery)
}

void sleep()
{
#if SLEEP_BETWEEN_MESSAGES

  // If the user has a screen, tell them we are about to sleep
  if (ssd1306_found)
  {
    // Show the going to sleep message on the screen
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "Sleeping in %3.1fs\n", (MESSAGE_TO_SLEEP_DELAY / 1000.0));
    screen->printText(buffer);

    // Wait for MESSAGE_TO_SLEEP_DELAY millis to sleep
    delay(MESSAGE_TO_SLEEP_DELAY);

    // Turn off screen
    screen->screenOff();
  }

  // Set the user button to wake the board
  sleep_interrupt(ctrlButtonStopPin, HIGH);

  // We sleep for the interval between messages minus the current millis
  // this way we distribute the messages evenly every SEND_INTERVAL millis
  uint32_t sleep_for = (millis() < SEND_INTERVAL) ? SEND_INTERVAL - millis() : SEND_INTERVAL;
  doDeepSleep(sleep_for);

#endif
}

void scanI2Cdevice(void)
{
  byte err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println(" !");
      nDevices++;

      if (addr == SSD1306_ADDRESS)
      {
        ssd1306_found = true;
        Serial.println("ssd1306 display found");
      }
      if (addr == AXP192_SLAVE_ADDRESS)
      {
        axp192_found = true;
        Serial.println("axp192 PMU found");
      }
    }
    else if (err == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

/**
 * Init the power manager chip
 * 
 * axp192 power 
    DCDC1 0.7-3.5V @ 1200mA max -> OLED // If you turn this off you'll lose comms to the axp192 because the OLED and the axp192 share the same i2c bus, instead use ssd1306 sleep mode
    DCDC2 -> unused
    DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
    LDO1 30mA -> charges GPS backup battery // charges the tiny J13 battery by the GPS to power the GPS ram (for a couple of days), can not be turned off
    LDO2 200mA -> LORA
    LDO3 200mA -> GPS
 */

void axp192Init()
{
  if (axp192_found)
  {
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
    {
      Serial.println("AXP192 Begin PASS");
    }
    else
    {
      Serial.println("AXP192 Begin FAIL");
    }
    // axp.setChgLEDMode(LED_BLINK_4HZ);
    Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
    Serial.println("----------------------------------------");

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300); // for the OLED power

    Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(
        PMU_IRQ, [] { pmu_irq = true; }, FALLING);

    axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
    axp.clearIRQ();

    if (axp.isChargeing())
    {
      baChStatus = "Charging";
    }
  }
  else
  {
    Serial.println("AXP192 not found");
  }
}

// Perform power on init that we do on each wake from deep sleep
void initDeepSleep()
{
  bootCount++;
  wakeCause = esp_sleep_get_wakeup_cause();
  /* 
    Not using yet because we are using wake on all buttons being low
    wakeButtons = esp_sleep_get_ext1_wakeup_status();       // If one of these buttons is set it was the reason we woke
    if (wakeCause == ESP_SLEEP_WAKEUP_EXT1 && !wakeButtons) // we must have been using the 'all buttons rule for waking' to support busted boards, assume button one was pressed
        wakeButtons = ((uint64_t)1) << buttons.gpios[0];
    */

  Serial.printf("booted, wake cause %d (boot count %d)\n", wakeCause, bootCount);
}

void setup()
{
  Serial.begin(115200); // initialize serial
  while (!Serial)
    ;

  pinMode(motorEnPin, OUTPUT);
  digitalWrite(motorEnPin, LOW);

  pinMode(ctrlButtonUpPin, INPUT);
  pinMode(ctrlButtonDownPin, INPUT);
  pinMode(ctrlButtonStopPin, INPUT);

  initDeepSleep();

  //Reset the OLED display in order that the I2C scan finds the device
  pinMode(SSD1306_RST_PIN, OUTPUT);
  digitalWrite(SSD1306_RST_PIN, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(SSD1306_RST_PIN, HIGH);

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2Cdevice();

  axp192Init();
  // Hello
  DEBUG_MSG(APP_NAME " " APP_VERSION "\n");

  // Don't init display if we don't have one or we are waking headless due to a timer event
  if (wakeCause == ESP_SLEEP_WAKEUP_TIMER)
  {
    ssd1306_found = false; // forget we even have the hardware
  }

  screen = new Screen(axp, axp192_found);

  if (ssd1306_found)
  {
    screen->setup();
  }

  // Init GPS
  //gps_setup();

  // Show logo on first boot after removing battery
  if (bootCount == 0)
  {
    screen->printText(APP_NAME " " APP_VERSION, 0, 0);
    screen->showLogo();
    screen->update();
    delay(LOGO_DELAY);
  }

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  xBinarySemaphoreDIO0 = xSemaphoreCreateBinary();
  xBinarySemaphoreDIO1 = xSemaphoreCreateBinary();

  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &isr_Timer, true);
  // timerAlarmWrite(timer, 8000000, false);
  // timerAlarmEnable(timer);

  Serial.println("LoRa Duplex with callback");

  setupSPIFFS();

  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);
  cfgAddress = config.address;
  cfgSubnet = config.subnet;
  cfgMotorRunTime = config.motorRuntime * 1000;
  Serial.print(F("Set address to..."));
  Serial.println(cfgAddress);

  const int ledPin = 25;

  pinMode(ledPin, OUTPUT); //Send success, LED will bright 1 second

  // int val = analogRead(36);
  // analogReadResolution(resolution);
  // analogSetWidth(width);
  // analogSetCycles(cycles);
  // analogSetSamples(samples);
  // analogSetClockDiv(attenuation);
  // analogSetAttenuation(ADC_0db);
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  // analogSetPinAttenuation(pin, attenuation);
  // adcAttachPin(pin);
  // adcStart(pin);
  // adcBusy(pin);
  // resultadcEnd(pin);

  // Initialize the button.
  buttonUpHigh.begin();
  buttonDownHigh.begin();
  buttonStop.begin();

  // Attach callback.
  buttonUpHigh.onPressedFor(buttonPressDuration, onPressedForDurationButtonUp); //Full up
  buttonUpHigh.onPressed(onPressedButtonUpHigh);                                //Step up start

  buttonDownHigh.onPressedFor(buttonPressDuration, onPressedForDurationButtonDown); // Full down
  buttonDownHigh.onPressed(onPressedButtonDownHigh);

  buttonStop.onPressedFor(buttonPressDuration, onPressedForDurationButtonStop);
  buttonStop.onPressed(onPressedButtonStop);

  setupPwmChannels();

  Serial.println("End of button setup!");

  //setupDisplay();

  Serial.println("End of display setup!");

  SPI.begin(loraSckPin, loraMisoPin, loraMosiPin, loraCsPin);
  // override the default CS, reset, and IRQ pins (optional)

  setupLora();

  Serial.println("LoRa Initial OK!");
  screen->_display->drawString(5, 20, "LoRa Initializing OK!");
  screen->_display->display();
  delay(2000);

  xTaskCreatePinnedToCore(
      taskMotorContol,
      "taskMotorControl", // A name just for humans
      8192,               // This stack size can be checked & adjusted by reading the Stack Highwater
      NULL,
      2, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      NULL,
      0);

  // xTaskCreatePinnedToCore(
  //     taskLoraProcessing,
  //     "taskLoraProcessing", // A name just for humans
  //     8192,                 // This stack size can be checked & adjusted by reading the Stack Highwater
  //     NULL,
  //     10, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  //     NULL,
  //     1);
  xTaskCreatePinnedToCore(
      taskLoraProcessingDIO0,
      "taskLoraProcessingDIO0", // A name just for humans
      8192,                     // This stack size can be checked & adjusted by reading the Stack Highwater
      NULL,
      10, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      NULL,
      1);
  xTaskCreatePinnedToCore(
      taskLoraProcessingDIO1,
      "taskLoraProcessingDIO1", // A name just for humans
      8192,                     // This stack size can be checked & adjusted by reading the Stack Highwater
      NULL,
      10, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      NULL,
      1);
}

void fsmProcess()
{
  blinds_motcmd_t cmd = MOTCMD_IDLE;
  portENTER_CRITICAL(&buttonMux);
  if (!motorCmdQueue.empty())
  {
    cmd = motorCmdQueue.front();
    motorCmdQueue.pop();
  }
  portEXIT_CRITICAL(&buttonMux);

  switch (blindsCurState)
  {
  case BLINDS_OFF:
    if (cmd == MOTCMD_STEP_UP)
    {
      blindsNextState = BLINDS_CMD_STEP_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_STEP_UP_START");
      }
      motorUpFade(1000000);
    }
    else if (cmd == MOTCMD_FULL_UP)
    {
      blindsNextState = BLINDS_CMD_FULL_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_FULL_UP");
      }
      motorUpFade(cfgMotorRunTime);
    }
    else if (cmd == MOTCMD_STEP_DOWN)
    {
      blindsNextState = BLINDS_CMD_STEP_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_STEP_DOWN_START");
      }
      motorDownFade(1000000);
    }
    else if (cmd == MOTCMD_FULL_DOWN)
    {
      blindsNextState = BLINDS_CMD_FULL_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_FULL_DOWN");
      }
      motorDownFade(cfgMotorRunTime);
    }
    break;

  case BLINDS_CMD_STEP_UP:
    if (cmd == MOTCMD_STOP)
    {
      blindsNextState = BLINDS_STOP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Stop pressed");
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
        blindsNextState = BLINDS_STOP;
        motorStop();
      }
    }
    break;
  case BLINDS_CMD_FULL_UP:
    if (cmd == MOTCMD_STOP)
    {
      blindsNextState = BLINDS_STOP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Stop pressed");
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
    break;

  case BLINDS_CMD_STEP_DOWN:
    if (cmd == MOTCMD_STOP)
    {
      blindsNextState = BLINDS_STOP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Stop pressed");
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
        blindsNextState = BLINDS_STOP;
        motorStop();
      }
    }
    break;
  case BLINDS_CMD_FULL_DOWN:
    if (cmd == MOTCMD_STOP)
    {
      blindsNextState = BLINDS_STOP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Stop pressed");
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
    break;

  case BLINDS_DOWN:
    if (cmd == MOTCMD_STEP_UP)
    {
      blindsNextState = BLINDS_CMD_STEP_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_STEP_UP_START");
      }
      motorUpFade(1000000);
    }
    else if (cmd == MOTCMD_FULL_UP)
    {
      blindsNextState = BLINDS_CMD_FULL_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_FULL_UP");
      }
      motorUpFade(cfgMotorRunTime);
    }
    break;

  case BLINDS_UP:
    if (cmd == MOTCMD_STEP_DOWN)
    {
      blindsNextState = BLINDS_CMD_STEP_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_STEP_DOWN_START");
      }
      motorDownFade(1000000);
    }
    else if (cmd == MOTCMD_FULL_DOWN)
    {
      blindsNextState = BLINDS_CMD_FULL_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button MOTCMD_FULL_DOWN");
      }
      motorDownFade(cfgMotorRunTime);
    }
    break;
  case BLINDS_STOP:
    blindsNextState = BLINDS_OFF;
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

  blindsCurState = blindsNextState;
}

void loop()
{

  // buttonUpHigh.read();
  // buttonDownHigh.read();
  // fsmProcess();

  // if (millis() - lastSendTime > interval)
  // {
  //   processTxCommand();
  //   lastSendTime = millis();        // timestamp the message
  //   interval = random(2000) + 1000; // 2-3 seconds
  //   LoRa.receive();                 // go back into receive mode
  // }

  // processRxCommand();
  //screen->loop();
  if (0)
  {

    sleep();
  }

  vTaskDelay(1000);
}
