#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>

/************************* Ethernet Client Setup *****************************/
byte mac[] = {0xDE, 0xED, 0xBE, 0xEB, 0xFE, 0xEF};

unsigned long lastPing = 0;
unsigned long connectedSince = 0;
unsigned long now = 0;
unsigned long nextConnectionAttempt = 0; 
unsigned long failedConnectionAttempts = 0;

#define RELAY_0 2 // Unused
#define ROOM_LIGHT_PIN 3
#define STOVE_LIGHT_PIN 7
#define OUTSIDE_LIGHT_PIN 5

#define MQTT_PING_INTERVAL_MS 60000
#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5

int lastState[28] = {0};

#define VERSION_MESSAGE F("Front Console v0.15 27/07/18")

#define AIO_SERVER      "192.168.2.20"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
#define WILL_FEED AIO_USERNAME "/feeds/nodes.frontdoor"
Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish front = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.front");
Adafruit_MQTT_Publish side = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.side");

//Adafruit_MQTT_Subscribe tvtoggle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.tv");
Adafruit_MQTT_Subscribe livingroomlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.livingroomlight");
Adafruit_MQTT_Subscribe stovelight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.stovelight");
Adafruit_MQTT_Subscribe outsidelight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight");

Adafruit_MQTT_Publish livingroomlight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.livingroomlight");
Adafruit_MQTT_Publish stovelight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.stovelight");
Adafruit_MQTT_Publish outsidelight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight");

#define halt(s) { Serial.println(F( s )); while(1);  }

void(* __resetFunc) (void) = 0; //declare reset function @ address 0

void resetFunc(const __FlashStringHelper* msg, unsigned long delayMs) {
  Serial.println(msg);
  Serial.print(F("Resetting in "));
  Serial.print(delayMs / 1000);
  Serial.println(F("s"));
  delay(delayMs);
  __resetFunc();
}

/*
Measuring Current Using ACS712
*/
#define OUTSIDE_LIGHT_CIRCUIT A4
#define STOVE_LIGHT_CIRCUIT A3
#define LIVINGROOM_LIGHT_CIRCUIT A5

int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

void readAcs712(int sensorIn, Adafruit_MQTT_Publish* feed) {

 Voltage = getVPP(sensorIn);
 VRMS = (Voltage/2.0) *0.707;
 AmpsRMS = (VRMS * 1000)/mVperAmp;
 Serial.print(AmpsRMS);
 Serial.print(F(" Amps RMS on "));
 Serial.println(sensorIn);

 if (AmpsRMS > 0.15) {
   if (lastState[sensorIn] == LOW) {
     Serial.println(F("Current rising edge detected. Publishing feed status 1"));
     feed->publish("1");
   }

   lastState[sensorIn] = HIGH;
 } else {
   if (lastState[sensorIn] == HIGH) {
     Serial.println(F("Current falling edge detected. Publishing feed status 0"));
     feed->publish("0");
   }

   lastState[sensorIn] = LOW;
 }
}

float getVPP(int sensorIn)
{
  float result;

  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here

   uint32_t start_time = millis();
   while((millis() - start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(sensorIn);
       // see if you have a new maxValue
       if (readValue > maxValue)
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue)
       {
           /*record the maximum sensor value*/
           minValue = readValue;
       }
   }

   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;

   return result;
 }


void setup() {
  // Disable SD card
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(RELAY_0, OUTPUT);
  pinMode(ROOM_LIGHT_PIN, OUTPUT);
  pinMode(STOVE_LIGHT_PIN, OUTPUT);
  pinMode(OUTSIDE_LIGHT_PIN, OUTPUT);

  digitalWrite(RELAY_0, HIGH);
  digitalWrite(ROOM_LIGHT_PIN, HIGH);
  digitalWrite(STOVE_LIGHT_PIN, HIGH);
  digitalWrite(OUTSIDE_LIGHT_PIN, HIGH);

  Serial.begin(115200);
  Serial.println(VERSION_MESSAGE);

  // Initialise the Client
  Serial.println(F("Joining the network..."));
  Ethernet.begin(mac);
  delay(2000); //give the ethernet a second to initialize
  Serial.println(Ethernet.localIP());
  if (Ethernet.localIP() == IPAddress(0,0,0,0)) {
    resetFunc(F("DHCP resolution failed"), 30000);
  }
  
  delay(250);

  //MQTT_connect();
  Serial.println(F("MQTT subscribe"));

  //mqtt.subscribe(&tvtoggle);
  mqtt.subscribe(&livingroomlight);
  mqtt.subscribe(&stovelight);
  mqtt.subscribe(&outsidelight);
  mqtt.will(WILL_FEED, "0");

  server.begin();
}


void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  now = millis();
  Ethernet.maintain();
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &outsidelight) {

      // 4 possibilities exist: 
      //    The light is already on, 
      //    the light is already off, 
      //    the relay needs to be turned off, 
      //    the relay needs to be turned on

      Serial.println(F("msg: OUTSIDE LIGHT"));

      if (strcmp((char *)subscription->lastread, "1") == 0 && lastState[OUTSIDE_LIGHT_CIRCUIT] == LOW) {
        lastState[OUTSIDE_LIGHT_PIN] = lastState[OUTSIDE_LIGHT_PIN] == HIGH ? LOW : HIGH;
        digitalWrite(OUTSIDE_LIGHT_PIN, lastState[OUTSIDE_LIGHT_PIN]);
      }
      if (strcmp((char *)subscription->lastread, "0") == 0 && lastState[OUTSIDE_LIGHT_CIRCUIT] == HIGH) {
        lastState[OUTSIDE_LIGHT_PIN] = lastState[OUTSIDE_LIGHT_PIN] == HIGH ? LOW : HIGH;
        digitalWrite(OUTSIDE_LIGHT_PIN, lastState[OUTSIDE_LIGHT_PIN]);
      }

      //lastState[OUTSIDE_LIGHT_PIN] = (lastState[OUTSIDE_LIGHT_PIN] == HIGH ? LOW : HIGH);
      //digitalWrite(OUTSIDE_LIGHT_PIN, lastState[OUTSIDE_LIGHT_PIN]);

    } else if (subscription == &livingroomlight) {

      Serial.println(F("msg: LIVING ROOM LIGHT"));

      if (strcmp((char *)subscription->lastread, "1") == 0 && lastState[LIVINGROOM_LIGHT_CIRCUIT] == LOW) {
        lastState[ROOM_LIGHT_PIN] = lastState[ROOM_LIGHT_PIN] == HIGH ? LOW : HIGH;
        digitalWrite(ROOM_LIGHT_PIN, lastState[ROOM_LIGHT_PIN]);
      }
      if (strcmp((char *)subscription->lastread, "0") == 0 && lastState[LIVINGROOM_LIGHT_CIRCUIT] == HIGH) {
        lastState[ROOM_LIGHT_PIN] = lastState[ROOM_LIGHT_PIN] == HIGH ? LOW : HIGH;
        digitalWrite(ROOM_LIGHT_PIN, lastState[ROOM_LIGHT_PIN]);
      }


    } else if (subscription == &stovelight) {

      Serial.println(F("msg: STOVE LIGHT"));
      
      if (strcmp((char *)subscription->lastread, "1") == 0 && lastState[STOVE_LIGHT_CIRCUIT] == LOW) {
        lastState[STOVE_LIGHT_PIN] = lastState[STOVE_LIGHT_PIN] == HIGH ? LOW : HIGH;
        lastState[STOVE_LIGHT_CIRCUIT] = HIGH;
        digitalWrite(STOVE_LIGHT_PIN, lastState[STOVE_LIGHT_PIN]);
      }
      if (strcmp((char *)subscription->lastread, "0") == 0 && lastState[STOVE_LIGHT_CIRCUIT] == HIGH) {
        lastState[STOVE_LIGHT_PIN] = lastState[STOVE_LIGHT_PIN] == HIGH ? LOW : HIGH;
        lastState[STOVE_LIGHT_CIRCUIT] = LOW;
        digitalWrite(STOVE_LIGHT_PIN, lastState[STOVE_LIGHT_PIN]);
      }
    }
  }

  handleHttpClientRequest();

  readAcs712(OUTSIDE_LIGHT_CIRCUIT, &outsidelight_pub);
  readAcs712(STOVE_LIGHT_CIRCUIT, &stovelight_pub);
  readAcs712(LIVINGROOM_LIGHT_CIRCUIT, &livingroomlight_pub);

  MQTT_ping();
  delay(1000);
}

void detectEdge(int pin, Adafruit_MQTT_Publish* feed) {
  int state = digitalRead(pin);
  if (state != lastState[pin]) {
    Serial.print(F("Publishing state change on pin "));
    Serial.print(pin);
    if (state == HIGH) {
      Serial.println(F(", high"));
      feed->publish("1");
    } else {
      Serial.println(F(", low"));
      feed->publish("0");
    }
  }

  lastState[pin] = state;
}
/*
void IR_decode() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    switch (results.decode_type) {
        case NEC: Serial.println("NEC"); break ;
        case SONY: Serial.println("SONY"); break ;
        case RC5: Serial.println("RC5"); break ;
        case RC6: Serial.println("RC6"); break ;
        case DISH: Serial.println("DISH"); break ;
        case SHARP: Serial.println("SHARP"); break ;
        case JVC: Serial.println("JVC"); break ;
        case SANYO: Serial.println("SANYO"); break ;
        case MITSUBISHI: Serial.println("MITSUBISHI"); break ;
        case SAMSUNG: Serial.println("SAMSUNG"); break ;
        case LG: Serial.println("LG"); break ;
        case WHYNTER: Serial.println("WHYNTER"); break ;
        case AIWA_RC_T501: Serial.println("AIWA_RC_T501"); break ;
        case PANASONIC: Serial.println("PANASONIC"); break ;
        case DENON: Serial.println("DENON"); break ;
      default:
        case UNKNOWN: Serial.println("UNKNOWN"); break ;
    }

    irrecv.resume();
    irrecv.enableIRIn();
  }
}
*/
void MQTT_ping() {

  if (!mqtt.connected()) {
    return;
  }

  if (lastPing + MQTT_PING_INTERVAL_MS < now) {
    Serial.println(F("Ping"));
    lastPing = now;
    if (!mqtt.ping()) {
      Serial.println(F("Failed to ping"));
      mqtt.disconnect();
    } else {
      lastwill.publish(now);
    }
  }
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  if (nextConnectionAttempt < now) {
    Serial.print(F("Connecting to MQTT... "));

    int delaySecs = (2 << failedConnectionAttempts); // Delay for 2, 4, 8 .. seconds
    if (ret = mqtt.connect() != 0) {
      Serial.print(F("Failed: "));
      Serial.println(mqtt.connectErrorString(ret));
      //mqtt.disconnect();

      nextConnectionAttempt = now + delaySecs * 1000;
      ++failedConnectionAttempts;
    }
  
    if (0 == ret) {
      connectedSince = millis();
      failedConnectionAttempts = 0;
      Serial.println(F("Connected!"));
    } else if (failedConnectionAttempts > MQTT_CONNECT_RETRY_MAX) {
      connectedSince = 0;
      resetFunc(F("Max retries exhausted!"), 2000); // Reset and try again
    } else {
      Serial.print(F("Retrying in "));

      Serial.print(delaySecs);
      Serial.println(F("s"));
    }
  }
}

void handleHttpClientRequest() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println(F("New http client"));
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/html"));
          client.println(F("Connection: close"));  // the connection will be closed after completion of the response
          client.println(F("Refresh: 10"));  // refresh the page automatically every 5 sec
          client.println();
          client.println(F("<!DOCTYPE HTML>"));
          client.println(F("<html>"));
          // output the value of each analog input pin

          client.print(F("<h1>"));
          client.print(VERSION_MESSAGE);
          client.println(F("</h1>"));
          client.print(F("<br />Outside light is "));
          client.println(lastState[OUTSIDE_LIGHT_CIRCUIT]);
          client.print(F("<br />Stove light is "));
          client.println(lastState[STOVE_LIGHT_CIRCUIT]);
          client.print(F("<br />Living room light is "));
          client.println(lastState[LIVINGROOM_LIGHT_CIRCUIT]);
          client.print(F("<br />Last ping "));
          client.print(lastPing);
          client.print(F("<br />Uptime "));
          client.print(now);
          client.print(F("<br />Connected since "));
          client.print(connectedSince);
          client.println(F("<br />"));
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            int sensorReading = analogRead(analogChannel);
            client.print(F("analog input "));
            client.print(analogChannel);
            client.print(F(" is "));
            client.print(sensorReading);
            client.println(F("<br />"));
          }

          client.println(F("</html>"));
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println(F("Http client disconnected"));
  }
}
