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

#define RELAY_0 2
#define ROOM_LIGHT_PIN 3
#define STOVE_LIGHT_PIN 4
#define OUTSIDE_LIGHT_PIN 5

int outsideLightState = HIGH;
int livingRoomLightState = HIGH;
int stoveLightState = HIGH;

#define VERSION_MESSAGE F("Front Console v0.14 24/07/18")

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "abc"
#define AIO_KEY         "xyz"

#define SERVER_LISTEN_PORT 80

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

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

void(* __resetFunc) (void) = 0; //declare reset function @ address 0

void resetFunc(const __FlashStringHelper* msg) {
  Serial.println(msg);
  Serial.println(F("Resetting in 2 seconds"));
  delay(2000);
  __resetFunc();
}

void setup() {
  // put your setup code here, to run once:

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
  delay(1000); //give the ethernet a second to initialize
  Serial.print("HTTP server listening on ");
  Serial.println(Ethernet.localIP());

  //MQTT_connect();
  Serial.println("MQTT subscribe");

  //mqtt.subscribe(&tvtoggle);
  mqtt.subscribe(&livingroomlight);
  mqtt.subscribe(&stovelight);
  mqtt.subscribe(&outsidelight);
  mqtt.will(WILL_FEED, "0");
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

      Serial.println(F("msg: OUTSIDE LIGHT"));
      outsideLightState = (outsideLightState == HIGH ? LOW : HIGH);
      digitalWrite(OUTSIDE_LIGHT_PIN, outsideLightState);


    } else if (subscription == &livingroomlight) {

      Serial.println(F("msg: LIVING ROOM LIGHT"));
      livingRoomLightState = (livingRoomLightState == HIGH ? LOW : HIGH);
      digitalWrite(ROOM_LIGHT_PIN, livingRoomLightState);

    } else if (subscription == &stovelight) {

      Serial.println(F("msg: STOVE LIGHT"));
      stoveLightState = (stoveLightState == HIGH ? LOW : HIGH);
      digitalWrite(STOVE_LIGHT_PIN, stoveLightState);
    }
  }

  handleHttpClientRequest();

  MQTT_ping();
  delay(1000);
}


int lastState[25] = {0};

void detectEdge(int pin, Adafruit_MQTT_Publish* feed) {
  int state = digitalRead(pin);
  if (state != lastState[pin]) {
    Serial.print("Publishing state change on pin ");
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

  if (lastPing + 60000 < now) {
    Serial.println(F("Ping"));
    lastPing = now;
    if (!mqtt.ping()) {
      Serial.println(F("Failed to ping"));
      mqtt.disconnect();
    } else {
      lastwill.publish(String(now, DEC).c_str());
    }
  }
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(F("Connecting to MQTT... "));

  int attempts = 0;
  while (++attempts < 10 && (ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.print(F("Retrying MQTT connection in "));

    int delaySecs = (2 << attempts); // Delay for 2, 4, 8 .. seconds
    Serial.print(delaySecs);
    Serial.println(F(" seconds"));
    mqtt.disconnect();
    delay(delaySecs * 1000);
  }

  if (0 == ret) {
    connectedSince = millis();
    Serial.println(F("MQTT Connected!"));
  } else {
    connectedSince = 0;
    resetFunc(F("Failed connection!")); // Reset and try again
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
          client.println(outsideLightState);
          client.print(F("<br />Stove light is "));
          client.println(stoveLightState);
          client.print(F("<br />Living room light is "));
          client.println(livingRoomLightState);
          client.print(F("<br />Last ping "));
          client.print(lastPing);
          client.print(F("<br />Uptime "));
          client.print(now);
          client.print(F("<br />Connected since "));
          client.print(connectedSince);

          client.println(F("<br />"));

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
