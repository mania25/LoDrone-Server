// rf95_server.cpp
//
// Example program showing how to use RH_RF95 on Raspberry Pi
// Uses the bcm2835 library to access the GPIO pins to drive the RFM95 module
// Requires bcm2835 library to be already installed
// http://www.airspayce.com/mikem/bcm2835/
// Use the Makefile in this directory:
// cd example/raspi/rf95
// make
// sudo ./rf95_server
//
// Contributed by Charles-Henri Hallard based on sample RH_NRF24 by Mike Poublon

#include "mqtt/client.h"
#include <bcm2835.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <stdlib.h>

#include <RH_RF69.h>
#include <RH_RF95.h>

// define hardware used change to fit your need
// Uncomment the board you have, if not listed
// uncommment custom board and set wiring tin custom section

// LoRasPi board
// see https://github.com/hallard/LoRasPI
// #define BOARD_LORASPI

// iC880A and LinkLab Lora Gateway Shield (if RF module plugged into)
// see https://github.com/ch2i/iC880A-Raspberry-PI
//#define BOARD_IC880A_PLATE

// Raspberri PI Lora Gateway for multiple modules
// see https://github.com/hallard/RPI-Lora-Gateway
//#define BOARD_PI_LORA_GATEWAY

// Dragino Raspberry PI hat
// see https://github.com/dragino/Lora
//#define BOARD_DRAGINO_PIHAT

// DycodeX LoRa Raspberry PI hat
// =========================================
// see https://github.com/dycodex/LoRa-Raspberry-Pi-Hat
// #define BOARD_DYCODEX

// DycodeX LoRa Raspberry PI hat V2
// =========================================
#define BOARD_DYCODEX_V2

// Now we include RasPi_Boards.h so this will expose defined
// constants with CS/IRQ/RESET/on board LED pins definition
#include "./RasPiBoards.h"

// Our RFM95 Configuration
#define RF_FREQUENCY 868.10
#define RF_NODE_ID 1

// Create an instance of a driver
RH_RF95 rf95(RF_CS_PIN, RF_IRQ_PIN);
// RH_RF95 rf95(RF_CS_PIN);

// Flag for Ctrl-C
volatile sig_atomic_t force_exit = false;

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID{"receiver"};
const std::string TOPIC{"/controlling-drone"};

const std::string user =
    "bbff39d0d3066758ffe55666762b3c8b150295b848cb6c871b79f2fff36c79fb";
const std::string password =
    "50acea3098359517297e08040dc6bfc371d044190be6527c1ac29e078cbe8313";

const int QOS = 0;

/////////////////////////////////////////////////////////////////////////////
// Class to receive callbacks

class user_callback : public virtual mqtt::callback {
  void connection_lost(const std::string &cause) override {
    std::cout << "\n[rf95_server][main][warning] Connection lost" << std::endl;
    if (!cause.empty())
      std::cout << "\tcause: " << cause << std::endl;
  }

public:
};

/*
std::string split implementation by using delimeter as a character.
*/
std::vector<std::string> split(std::string strToSplit, char delimeter)
{
  std::stringstream ss(strToSplit);
  std::string item;
	std::vector<std::string> splittedStrings;
  while (std::getline(ss, item, delimeter)) {
		splittedStrings.push_back(item);
  }
  return splittedStrings;
}

std::string getEnvVar( std::string const &key )
{
    char *val = getenv(key.c_str());
    return val == NULL ? std::string("") : std::string(val);
}

void logger( const std::string &text )
{
    std::ofstream log_file(
        "/home/pi/Documents/TA/LoraDroneControl/log_flight.log", std::ios_base::out | std::ios_base::app );
    log_file << text << std::endl;
    log_file.close();
}

void sig_handler(int sig) {
  printf("\n[rf95_server][main][info] %s Break received, exiting!\n", __BASEFILE__);
  force_exit = true;
}

// Main Function
int main(int argc, const char *argv[]) {
  unsigned long led_blink = 0;
  const std::string deviceID = getEnvVar("device_id");

  if (deviceID.empty()) {
    printf("Device ID Not Found. . .\n");
    return 1;
  }

  signal(SIGINT, sig_handler);
  printf("%s\n", __BASEFILE__);

  if (!bcm2835_init()) {
    fprintf(stderr, "[rf95_server][main][error] %s bcm2835_init() Failed\n\n", __BASEFILE__);
    return 1;
  }

  std::cout << "\n[rf95_server][main][info] Initializing Device ID: " << std::flush;
  std::cout << deviceID << std::endl;

  printf("\n");

  printf("[rf95_server][main][info] RF95 CS=GPIO%d", RF_CS_PIN);

#ifdef RF_LED_PIN
  pinMode(RF_LED_PIN, OUTPUT);
  digitalWrite(RF_LED_PIN, HIGH);
#endif

#ifdef RF_IRQ_PIN
  printf(", IRQ=GPIO%d", RF_IRQ_PIN);
  // IRQ Pin input/pull down
  pinMode(RF_IRQ_PIN, INPUT);
  bcm2835_gpio_set_pud(RF_IRQ_PIN, BCM2835_GPIO_PUD_DOWN);
  // Now we can enable Rising edge detection
  bcm2835_gpio_ren(RF_IRQ_PIN);
#endif

#ifdef RF_RST_PIN
  printf(", RST=GPIO%d", RF_RST_PIN);
  // Pulse a reset on module
  pinMode(RF_RST_PIN, OUTPUT);
  digitalWrite(RF_RST_PIN, LOW);
  bcm2835_delay(150);
  digitalWrite(RF_RST_PIN, HIGH);
  bcm2835_delay(100);
#endif

#ifdef RF_LED_PIN
  printf(", LED=GPIO%d", RF_LED_PIN);
  digitalWrite(RF_LED_PIN, LOW);
#endif

  if (!rf95.init()) {
    fprintf(stderr, "\n[rf95_server][main][error] RF95 module init failed, Please verify wiring/module\n");
  } else {
    printf("\n[rf95_server][main][info] RF95 module seen OK!\r\n");
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
    // 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST
    // transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
    //  driver.setTxPower(23, false);
    // If you are using Modtronix inAir4 or inAir9,or any other module which
    // uses the transmitter RFO pins and not the PA_BOOST pins then you can
    // configure the power transmitter power for -1 to 14 dBm and with useRFO
    // true. Failure to do that will result in extremely low transmit powers.
    // rf95.setTxPower(14, true);

    // RF95 Modules don't have RFO pin connected, so just use PA_BOOST
    // check your country max power useable, in EU it's +14dB
    rf95.setTxPower(23, false);

    // You can optionally require this module to wait until Channel Activity
    // Detection shows no activity on the channel before transmitting by setting
    // the CAD timeout to non-zero:
    // rf95.setCADTimeout(10000);

    // Adjust Frequency
    rf95.setFrequency(RF_FREQUENCY);

    // If we need to send something
    rf95.setThisAddress(RF_NODE_ID);
    rf95.setHeaderFrom(RF_NODE_ID);

    // Be sure to grab all node packet
    // we're sniffing to display, it's a demo
    rf95.setPromiscuous(false);

    // We're ready to listen for incoming message
    rf95.setModeRx();

    std::cout << "\n[rf95_server][main][info] Initialzing..." << std::flush;
    mqtt::client client(SERVER_ADDRESS, CLIENT_ID);

    user_callback cb;
    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    connOpts.set_connection_timeout(60);
    connOpts.set_connect_timeout(60);
    
    connOpts.set_automatic_reconnect(true);
    connOpts.set_automatic_reconnect(1, 10);

    connOpts.set_user_name(user.c_str());
    connOpts.set_password(password.c_str());

    std::cout << "OK" << std::endl;

    printf("[rf95_server][main][info] RF95 node #%d init OK @ %3.2fMHz\n", RF_NODE_ID, RF_FREQUENCY);
    
    bcm2835_delay(10000);

    // Begin the main body of code
    std::cout << "[rf95_server][main][info] Connecting to the MQTT server..." << std::flush;
    client.connect(connOpts);
    std::cout << "OK\n" << std::endl;

    std::cout << "[rf95_server][main][info] Listening packet...\n" << std::endl;

    while (!force_exit) {

#ifdef RF_IRQ_PIN
      // We have a IRQ pin ,pool it instead reading
      // Modules IRQ registers from SPI in each loop

      // Rising edge fired ?
      if (bcm2835_gpio_eds(RF_IRQ_PIN)) {
        // Now clear the eds flag by setting it to 1
        bcm2835_gpio_set_eds(RF_IRQ_PIN);
        // printf("Packet Received, Rising event detect for pin GPIO%d\n",
        // RF_IRQ_PIN);
#endif

        if (rf95.available()) {
#ifdef RF_LED_PIN
          led_blink = millis();
          digitalWrite(RF_LED_PIN, HIGH);
#endif
          // Should be a message for us now
          uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
          uint8_t len = sizeof(buf);
          uint8_t from = rf95.headerFrom();
          uint8_t to = rf95.headerTo();
          uint8_t id = rf95.headerId();
          uint8_t flags = rf95.headerFlags();
          int8_t rssi = rf95.lastRssi();

          if (rf95.recv(buf, &len)) {
            int64_t receivedMessageTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            
            printf("[rf95_server][main][info] Packet[%02d] #%d => #%d %ddB: ", len, from, to, rssi);
            printbuffer(buf, len);
            std::string message(buf, buf + len);
            std::cout << "\n[rf95_server][main][info] Got Message: " << message << '\n';
            try {
              std::vector<std::string> extractReceivedData = split(message, '|');

              if (extractReceivedData.size() > 1) {
                int64_t sendingTimeFromGS = strtoll(extractReceivedData[2].c_str(), NULL, 10);
                int64_t receiveTimeDelayInMillis = receivedMessageTime - sendingTimeFromGS;
                
                logger("Received at: " + std::to_string(receivedMessageTime) + ", Message: " + extractReceivedData[1] + ", Time Delay: " + std::to_string(receiveTimeDelayInMillis) + "ms, RSSI: " + std::to_string(rssi) + " dB");

                if (extractReceivedData[0] == deviceID) {
                  // First use a message pointer.
                  std::cout << "\n[rf95_server][main][info] Sending message..." << std::endl;
                  auto pubmsg = mqtt::make_message(TOPIC, extractReceivedData[1]);
                  pubmsg->set_qos(QOS);
                  client.publish(pubmsg);
                  std::cout << "...OK" << std::endl;
                }
              }
            } catch (const mqtt::exception &exc) {
              std::cerr << exc.what() << std::endl;
              return 1;
            }
          } else {
            Serial.print("[rf95_server][main][warning] receive failed");
          }
          printf("\n");
        } else {
          Serial.println("[rf95_server][main][info] Not Receiving Any Command. . .");
        }

#ifdef RF_IRQ_PIN
      }
#endif

#ifdef RF_LED_PIN
      // Led blink timer expiration ?
      if (led_blink && millis() - led_blink > 200) {
        led_blink = 0;
        digitalWrite(RF_LED_PIN, LOW);
      }
#endif
      // Let OS doing other tasks
      // For timed critical appliation you can reduce or delete
      // this delay, but this will charge CPU usage, take care and monitor
      bcm2835_delay(5);
    }

    // Disconnect
    std::cout << "\n[rf95_server][main][info] Disconnecting from the MQTT server..." << std::flush;
    client.disconnect();
    std::cout << "OK" << std::endl;
  }

#ifdef RF_LED_PIN
  digitalWrite(RF_LED_PIN, LOW);
#endif
  printf("\n%s Ending\n", __BASEFILE__);
  bcm2835_close();
  return 0;
}
