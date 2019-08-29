#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>

#define VERSION_MESSAGE F("Front Console v0.20 28/08/19")

#define ROOM_LIGHT_PIN 4
#define STOVE_LIGHT_PIN 7
#define OUTSIDE_LIGHT_PIN 5
#define FRONT_DOOR_PIN 8

#define OUTSIDE_LIGHT_CIRCUIT A2
#define STOVE_LIGHT_CIRCUIT A4
#define LIVINGROOM_LIGHT_CIRCUIT A0

#define CURRENT_SENSOR_INTERVAL_MS 3000
#define MQTT_PING_INTERVAL_MS 60000
#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5

#define AIO_SERVER      "192.168.2.20"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"
#define WILL_FEED AIO_USERNAME "/feeds/nodes.frontdoor"

int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

byte mac[] = {0xDE, 0xED, 0xBE, 0xEB, 0xFE, 0xEF};

int lastState[20] = {0};

unsigned long lastPing = 0;
unsigned long connectedSince = 0;
unsigned long now = 0;
unsigned long nextConnectionAttempt = 0; 
unsigned long failedConnectionAttempts = 0;
unsigned long lastCurrentSensorRead = 0;
unsigned long sensorIndex = 0;

EthernetClient client;
EthernetServer server(SERVER_LISTEN_PORT);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Subscribe livingroomlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.livingroomlight");
Adafruit_MQTT_Subscribe stovelight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.stovelight");
Adafruit_MQTT_Subscribe outsidelight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight2");

Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);
Adafruit_MQTT_Publish front = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/doors.front");
Adafruit_MQTT_Publish livingroomlight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.livingroomlight");
Adafruit_MQTT_Publish stovelight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.stovelight");
Adafruit_MQTT_Publish outsidelight_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/toggle.outsidelight2");

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

void readAcs712(int sensorIn, Adafruit_MQTT_Publish* feed, bool forceInit = false) {

 Voltage = getVPP(sensorIn);
 VRMS = (Voltage/2.0) *0.707;
 AmpsRMS = (VRMS * 1000)/mVperAmp;
 Serial.print(AmpsRMS);
 Serial.print(F(" Amps RMS on "));
 Serial.println(sensorIn);

 if (AmpsRMS > 0.15) {
   if (forceInit || lastState[sensorIn] == LOW) {
     Serial.println(F("Current rising edge detected. Publishing feed status 1"));
     feed->publish("1");
   }

   lastState[sensorIn] = HIGH;
 } else {
   if (forceInit || lastState[sensorIn] == HIGH) {
     Serial.println(F("Current falling edge detected. Publishing feed status 0"));
     feed->publish("0");
   }

   lastState[sensorIn] = LOW;
 }
}

float getVPP(int sensorIn)
{
  float result;
  int readValue;
  int maxValue = 0; 
  int minValue = 1024;
  uint32_t start_time = millis();

  // 1 second sample
  while((millis() - start_time) < 1000) {
    
    readValue = analogRead(sensorIn);
    if (readValue > maxValue) {
      maxValue = readValue;
    }
       
    if (readValue < minValue) {
      minValue = readValue;
    }
  }

   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;
   return result;
 }

void readCurrentSensors() {
  if (now - lastCurrentSensorRead > CURRENT_SENSOR_INTERVAL_MS) {
    lastCurrentSensorRead = now;

    if (sensorIndex == 0) {
      Serial.println(F("Read living room light current"));
      readAcs712(LIVINGROOM_LIGHT_CIRCUIT, &livingroomlight_pub);
    }
    if (sensorIndex == 1) {
      Serial.println(F("Read stive light current"));
      readAcs712(STOVE_LIGHT_CIRCUIT, &stovelight_pub);
    }
    if (sensorIndex == 2) {
      Serial.println(F("Read outside light current"));
      readAcs712(OUTSIDE_LIGHT_CIRCUIT, &outsidelight_pub);
    }
    
    sensorIndex = (sensorIndex + 1) % 3;
  }
}

void setup() {

  pinMode(ROOM_LIGHT_PIN, OUTPUT);
  pinMode(STOVE_LIGHT_PIN, OUTPUT);
  pinMode(OUTSIDE_LIGHT_PIN, OUTPUT);
  pinMode(FRONT_DOOR_PIN, INPUT_PULLUP);

  delay(500);
  
  digitalWrite(ROOM_LIGHT_PIN, HIGH);
  lastState[ROOM_LIGHT_PIN] = HIGH;
  digitalWrite(STOVE_LIGHT_PIN, HIGH);
  lastState[STOVE_LIGHT_PIN] = HIGH;
  digitalWrite(OUTSIDE_LIGHT_PIN, HIGH);
  lastState[OUTSIDE_LIGHT_PIN] = HIGH;

  lastState[FRONT_DOOR_PIN] = digitalRead(FRONT_DOOR_PIN);

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

  Serial.println(F("MQTT subscribe"));

  mqtt.subscribe(&livingroomlight, &onSubscriptionEvent);
  mqtt.subscribe(&stovelight, &onSubscriptionEvent);
  mqtt.subscribe(&outsidelight, &onSubscriptionEvent);
  mqtt.will(WILL_FEED, "0");

  readAcs712(LIVINGROOM_LIGHT_CIRCUIT, &livingroomlight_pub, true);
  readAcs712(STOVE_LIGHT_CIRCUIT, &stovelight_pub, true);
  readAcs712(OUTSIDE_LIGHT_CIRCUIT, &outsidelight_pub, true);

  server.begin();
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the connectMqtt
  // function definition further below.
  now = millis();
  Ethernet.maintain();
  connectMqtt();

  mqtt.process(50);

  handleHttpClientRequest();
  readCurrentSensors();
  detectEdge(FRONT_DOOR_PIN, &front);

  pingMqtt();
  delay(10);
}

void onSubscriptionEvent(Adafruit_MQTT_Subscribe* subscription) {
  Serial.println(F("Incoming message"));
  char* command = (char*)subscription->lastread;
  command[0] = tolower(command[0]);

  if (subscription == &outsidelight) {

    // 4 possibilities exist: 
    //    The light is already on, 
    //    the light is already off, 
    //    the relay needs to be turned off, 
    //    the relay needs to be turned on

    Serial.println(F("msg: OUTSIDE LIGHT"));
    handleMqttToggleCommand((const char*)command, OUTSIDE_LIGHT_PIN, OUTSIDE_LIGHT_CIRCUIT);

  } else if (subscription == &livingroomlight) {

    Serial.println(F("msg: LIVING ROOM LIGHT"));
    handleMqttToggleCommand((const char*)command, ROOM_LIGHT_PIN, LIVINGROOM_LIGHT_CIRCUIT);

  } else if (subscription == &stovelight) {

    Serial.println(F("msg: STOVE LIGHT"));
    handleMqttToggleCommand((const char*)command, STOVE_LIGHT_PIN, STOVE_LIGHT_CIRCUIT);
  }
}

void onPing(bool result) {
  lastwill.publish(now);
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

void pingMqtt() {

  if (!mqtt.connected()) {
    return;
  }

  if (now - lastPing > MQTT_PING_INTERVAL_MS) {
    Serial.println(F("Ping"));
    lastPing = now;
    mqtt.pingAsync(onPing);
  }
}

void connectMqtt() {
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

void handleMqttToggleCommand(const char* command, int outputPin, int acsPin) {

  if (-1 == acsPin) {
    
    // For lights with no attached ACS sensr
    if (strcmp((char *)command, "1") == 0 && lastState[outputPin] == HIGH) {
      Serial.println(F("ACS-less relay is OFF (turn on)"));
      lastState[outputPin] = LOW;
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);

    } else if (strcmp((char *)command, "0") == 0 && lastState[outputPin] == LOW) {
      Serial.println(F("ACS-less relay is ON (turn off)"));
      lastState[outputPin] = HIGH;
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
    }
  } else {

    // For lights with attached ACS sensor
    if (strcmp((char *)command, "1") == 0 && lastState[acsPin] == LOW) {
      Serial.println(F("ACS relay is OFF (turn on)"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
      
    } else if (strcmp((char *)command, "0") == 0 && lastState[acsPin] == HIGH) {
      Serial.println(F("ACS relay is ON (turn off)"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
      Serial.print(F("Setting state to "));
      Serial.print(lastState[outputPin]);
      digitalWrite(outputPin, lastState[outputPin]);
    } else if (strcmp((char*)command, "2") == 0) {
      Serial.println(F("ACS relay toggle"));
      lastState[outputPin] = lastState[outputPin] == HIGH ? LOW : HIGH;
      digitalWrite(outputPin, lastState[outputPin]);
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
