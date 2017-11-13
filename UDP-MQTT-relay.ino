

#include <MD_KeySwitch.h>
const uint8_t SWITCH_PIN = 0;       // switch connected to this pin
const uint8_t SWITCH_ACTIVE = LOW;  // digital signal when switch is pressed 'on'
MD_KeySwitch S(SWITCH_PIN, SWITCH_ACTIVE);

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <Ticker.h>



#define WIFI_LED_PIN 2 //2=NodeMCU vagy ESP-12, 1=ESP-01 beépített LED
#define RETAINED true

const int FW_VERSION = 1001;
const char* fwUrlBase = "http://192.168.1.196/fwtest/fota/"; //FW files should be uploaded to this HTTP directory
// note: alias.bin and alias.version files should be there. Update will be performed if the version file contains bigger number than the FW_VERSION variable

#define ALIAS "udp-mqtt"
#define TOPIC_DEV_STATUS "/device/" ALIAS "/status"
#define TOPIC_DEV_COMMAND "/device/" ALIAS "/command"
#define TOPIC_DEV_TEST "/device/" ALIAS "/test"
#define TOPIC_DEV_SETURL "/device/" ALIAS "/seturl"
#define TOPIC_DEV_FASTLED "/device/" ALIAS "/fastled"
#define TOPIC_DEV_ALARM "/device/" ALIAS "/alarm"
#define TOPIC_DEV_FOTA "/device/" ALIAS "/fota"
#define TOPIC_DEV_RGB "/device/" ALIAS "/rgb"

#define TOPIC_ALL_ALARM "/all/alarm"
#define TOPIC_ALL_FOTA "/all/fota"
#define TOPIC_ALL_RGB "/all/rgb"


const char* ssid = "testm";
const char* password = "12345678";
const char* alias = ALIAS;


String topicTemp; //topic string used ofr various publishes
String publishTemp;
char msg[50];
char topic[50];
char recMsg[50];
char recTopic[50];
char recCommand[10];
char recValue[10];

unsigned int localUdpPort = 9900;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char  replyPacket[] = "Hi there! Got the message :-)";  // a reply string to send back

boolean msgReceived; //shows if message received
char digitTemp[3];
unsigned long wifiCheckTimer;


// Replace with the IP address of your MQTT server
IPAddress mqttServerIP(192, 168, 1, 1);

Ticker flipper; //for blinking built-in LED to show wifi and MQTT status



WiFiUDP UDPServer;
WiFiClient wclient;
PubSubClient client(wclient);


/*
  _____ ___ _____  _      _____ _   _ _   _  ____ _____ ___ ___  _   _ ____
  |  ___/ _ \_   _|/ \    |  ___| | | | \ | |/ ___|_   _|_ _/ _ \| \ | / ___|
  | |_ | | | || | / _ \   | |_  | | | |  \| | |     | |  | | | | |  \| \___ \
  |  _|| |_| || |/ ___ \  |  _| | |_| | |\  | |___  | |  | | |_| | |\  |___) |
  |_|   \___/ |_/_/   \_\ |_|    \___/|_| \_|\____| |_| |___\___/|_| \_|____/

*/
void checkForUpdates() {

  String fwURL = String( fwUrlBase );
  fwURL.concat( alias );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );

  Serial.println( "Checking for firmware updates." );
  Serial.print( "Alias: " );
  Serial.println( alias );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );
  //Blink three times with BLUE color to show the checking phase


  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if ( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();

    Serial.print( "Current firmware version: " );
    Serial.println( FW_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );

    int newVersion = newFWVersion.toInt();

    if ( newVersion > FW_VERSION ) {
      Serial.println( "Preparing to update" );
      //Changing all LEDs to static pattern to make the update status visible



      //Publish FW found status
      client.publish(TOPIC_DEV_STATUS, "FW found, downloading");

      String fwImageURL = fwURL;
      fwImageURL.concat( ".bin" );
      Serial.println( "Update started." );
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );

      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          //Publish failed status
          client.publish(TOPIC_DEV_STATUS, "HTTP update failed");
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          //Publish no HTTP update status
          client.publish(TOPIC_DEV_STATUS, "No HTTP updates");
          break;
      }
    }
    else {
      Serial.println( "Already on latest version" );
      //Publish FW found status
      client.publish(TOPIC_DEV_STATUS, "Already on latest version");
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
    //Publish failed status
    client.publish(TOPIC_DEV_STATUS, "Could not check available FW");
  }
  httpClient.end();
}


/**************FOTA FUNCTIONS END******************************/

/*
   __  __  ___ _____ _____   _____ _   _ _   _  ____ _____ ___ ___  _   _ ____
  |  \/  |/ _ \_   _|_   _| |  ___| | | | \ | |/ ___|_   _|_ _/ _ \| \ | / ___|
  | |\/| | | | || |   | |   | |_  | | | |  \| | |     | |  | | | | |  \| \___ \
  | |  | | |_| || |   | |   |  _| | |_| | |\  | |___  | |  | | |_| | |\  |___) |
  |_|  |_|\__\_\|_|   |_|   |_|    \___/|_| \_|\____| |_| |___\___/|_| \_|____/

*/
//Subscribe function
void subscribeToTopics() {

  //Note: multible subscription will not work without client.loop();

/*
  client.subscribe(TOPIC_DEV_COMMAND);
  Serial.println("Subscribed to [" TOPIC_DEV_COMMAND "] topic");
  client.loop();
  client.subscribe(TOPIC_DEV_TEST);
  Serial.println("Subscribed to [" TOPIC_DEV_TEST "] topic");
  client.loop();
  client.subscribe(TOPIC_DEV_SETURL);
  Serial.println("Subscribed to [" TOPIC_DEV_SETURL "] topic");
  client.loop();
  client.subscribe(TOPIC_DEV_FASTLED);
  Serial.println("Subscribed to [" TOPIC_DEV_FASTLED "] topic");
  client.loop();
  client.subscribe(TOPIC_DEV_ALARM);
  Serial.println("Subscribed to [" TOPIC_DEV_ALARM "] topic");
  client.loop();
  client.subscribe(TOPIC_DEV_FOTA);
  Serial.println("Subscribed to [" TOPIC_DEV_FOTA "] topic");
  client.loop();
  client.subscribe(TOPIC_DEV_RGB);
  Serial.println("Subscribed to [" TOPIC_DEV_RGB "] topic");
  client.loop();
  client.subscribe(TOPIC_ALL_ALARM);
  Serial.println("Subscribed to [" TOPIC_ALL_ALARM "] topic");
  client.loop();
  client.subscribe(TOPIC_ALL_FOTA);
  Serial.println("Subscribed to [" TOPIC_ALL_FOTA "] topic");
  client.loop();
  client.subscribe(TOPIC_ALL_RGB);
  Serial.println("Subscribed to [" TOPIC_ALL_RGB "] topic");
*/

  client.subscribe("#");  //SUBSCRIBE TO ALL TOPICS
  Serial.println("Subscribed to all topics (#)");


}
void publishMyIP() {
  //Publishing own IP on device/[alias]/ip/ topic
  IPAddress local = WiFi.localIP();
  strcpy(msg, "");
  sprintf(msg, "IP: %i.%i.%i.%i", local[0], local[1], local[2], local[3]);
  client.publish(TOPIC_DEV_STATUS, msg, RETAINED);
  Serial.printf("MQTT topic: %s, message: %s\n", TOPIC_DEV_STATUS, msg);

}

void publishMyFW() {

  //Publishing own FW version on device/[alias]/fw/ topic
  strcpy(msg, "");
  sprintf(msg, "FW: %i", FW_VERSION);
  client.publish(TOPIC_DEV_STATUS, msg, RETAINED);
  Serial.printf("MQTT topic: %s, message: %s\n", TOPIC_DEV_STATUS, msg);

}

void publishMyHeap() {

  //Publishing own FW version on device/[alias]/fw/ topic
  strcpy(msg, "");
  sprintf(msg, "HEAP: %i", ESP.getFreeHeap());
  client.publish(TOPIC_DEV_STATUS, msg);
  Serial.printf("MQTT topic: %s, message: %s\n", TOPIC_DEV_STATUS, msg);

}

void publishMySketchSize() {

  //Publishing own FW version on device/[alias]/fw/ topic
  strcpy(msg, "");
  sprintf(msg, "SKETCH: %i", ESP.getSketchSize());
  client.publish(TOPIC_DEV_STATUS, msg);
  Serial.printf("MQTT topic: %s, message: %s\n", TOPIC_DEV_STATUS, msg);

}

void publishMyFreeSize() {

  //Publishing own FW version on device/[alias]/fw/ topic
  strcpy(msg, "");
  sprintf(msg, "FREE: %i", ESP.getFreeSketchSpace());
  client.publish(TOPIC_DEV_STATUS, msg);
  Serial.printf("MQTT topic: %s, message: %s\n", TOPIC_DEV_STATUS, msg);

}

// Callback function
void callback(char* topic, byte * payload, unsigned int length) {
  digitalWrite(WIFI_LED_PIN, LOW);   // Turn the LED on (Note that LOW is the voltage level

  Serial.print("Message arrived [");
  Serial.print(topic);
  strcpy(recTopic, "");
  strcpy(recTopic, topic); //storing received topic for further processing
  Serial.print("] ");
  int i;
  for (i = 0; i < length; i++) {
    // Serial.print((char)payload[i]);
    recMsg[i] = (char)payload[i];
  }
  recMsg[i] = '\0'; //Closing the C string
  Serial.println(recMsg);
  msgReceived = true;

  digitalWrite(WIFI_LED_PIN, HIGH);  // Turn the LED off by making the voltage HIGH
}

/**************MQTT FUNCTIONS END******************************/

/*

  __  __ _____ ____ ____    _    ____ _____   ____  ____   ___   ____ _____ ____ ____
  |  \/  | ____/ ___/ ___|  / \  / ___| ____| |  _ \|  _ \ / _ \ / ___| ____/ ___/ ___|
  | |\/| |  _| \___ \___ \ / _ \| |  _|  _|   | |_) | |_) | | | | |   |  _| \___ \___ \
  | |  | | |___ ___) |__) / ___ \ |_| | |___  |  __/|  _ <| |_| | |___| |___ ___) |__) |
  |_|  |_|_____|____/____/_/   \_\____|_____| |_|   |_| \_\\___/ \____|_____|____/____/

*/


void processRecMessage() {

  bool validContent = false;


  // COMMAND BRANCH
  if (strcmp(recTopic, TOPIC_DEV_COMMAND) == 0) {
    validContent = true;
    Serial.println(TOPIC_DEV_COMMAND " branch");
    if (strcmp(recMsg, "getfw") == 0) {

      validContent = true;
      publishMyFW();
    }
    if (strcmp(recMsg, "getip") == 0) {
      validContent = true;
      publishMyIP();
    }
    if (strcmp(recMsg, "getsketchsize") == 0) {
      validContent = true;
      publishMySketchSize();
    }
    if (strcmp(recMsg, "getfreesize") == 0) {
      validContent = true;
      publishMyFreeSize();
    }
    if (strcmp(recMsg, "getfreeheap") == 0) {
      validContent = true;
      publishMyHeap();
    }


    if (strcmp(recMsg, "reboot") == 0) {
      Serial.println("Rebooting...");

      delay(500);
      ESP.restart();
    }

  }// END OF COMMAND BRANCH


  //FOTA BRANCH

  if (strcmp(recTopic, TOPIC_DEV_FOTA) == 0 || strcmp(recTopic, TOPIC_ALL_FOTA) == 0) {

    if (strcmp(recMsg, "checknew") == 0) { //command received for checking new FW
      validContent = true;
      checkForUpdates();
    }

  }//END OF FOTA BRANCH

  if (!validContent) Serial.println("Not a valid content");
  msgReceived = false;
}


/**************MESSAGE PROCESS FUNCTIONS END******************************/

/*

  _   _ _____ _     ____  _____ ____    _____ _   _ _   _  ____ _____ ___ ___  _   _ ____
  | | | | ____| |   |  _ \| ____|  _ \  |  ___| | | | \ | |/ ___|_   _|_ _/ _ \| \ | / ___|
  | |_| |  _| | |   | |_) |  _| | |_) | | |_  | | | |  \| | |     | |  | | | | |  \| \___ \
  |  _  | |___| |___|  __/| |___|  _ <  |  _| | |_| | |\  | |___  | |  | | |_| | |\  |___) |
  |_| |_|_____|_____|_|   |_____|_| \_\ |_|    \___/|_| \_|\____| |_| |___\___/|_| \_|____/

*/

boolean isTimeout(unsigned long checkTime, unsigned long timeWindow)
{
  unsigned long now = millis();
  if (checkTime <= now)
  {
    if ( (unsigned long)(now - checkTime )  < timeWindow )
      return false;
  }
  else
  {
    if ( (unsigned long)(checkTime - now) < timeWindow )
      return false;
  }

  return true;
}

void flip()
{
  int state = digitalRead(WIFI_LED_PIN);  // get the current state of GPIO2 pin
  digitalWrite(WIFI_LED_PIN, !state);     // set pin to the opposite state

}





/**************OTHER FUNCTIONS END******************************/

/*

  ____  _____ _____ _   _ ____
  / ___|| ____|_   _| | | |  _ \
  \___ \|  _|   | | | | | | |_) |
  ___) | |___  | | | |_| |  __/
  |____/|_____| |_|  \___/|_|

*/

void setup()
{
  Serial.begin(115200);
  pinMode(WIFI_LED_PIN, OUTPUT);     // Initialize the INDICATOR_PIN pin as an output
  Serial.println("Booting...");

  //KeySwitch init
  S.begin();
  S.enableDoublePress(true);
  S.enableLongPress(true);
  S.enableRepeat(false);
  S.enableRepeatResult(false);
  S.setLongPressTime(1000);
  S.setDoublePressTime(500);


  Serial.println(__TIMESTAMP__);
  Serial.printf("Sketch size: %u\n", ESP.getSketchSize());
  Serial.printf("Free size: %u\n", ESP.getFreeSketchSpace());
  Serial.print( "Current firmware version: " );
  Serial.println( FW_VERSION );
  Serial.println();

  //  connectWifi();

  UDPServer.begin(localUdpPort);
  


  client.setServer(mqttServerIP, 1883);
  client.setCallback(callback);

  // WiFi.setSleepMode(WIFI_NONE_SLEEP); //avoid wifi sleep to get rid of flickering AND wifi issues

  msgReceived = false;

  Serial.println();


}

/*

  __  __    _    ___ _   _   _     ___   ___  ____
  |  \/  |  / \  |_ _| \ | | | |   / _ \ / _ \|  _ \
  | |\/| | / _ \  | ||  \| | | |  | | | | | | | |_) |
  | |  | |/ ___ \ | || |\  | | |__| |_| | |_| |  __/
  |_|  |_/_/   \_\___|_| \_| |_____\___/ \___/|_|

*/

void loop() {

  /**************CONNECT/RECONNECT SEQUENCE ******************************/
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, password);
    flipper.attach(0.1, flip); //blink quickly during wifi connection
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
  }

  if (WiFi.status() == WL_CONNECTED) {

    if (!client.connected()) {
      Serial.println("Connecting mqtt client...");
      flipper.attach(0.25, flip); //blink slower during client connection

      //connect with Last Will message as "offline"/retained. This will be published when incorrectly disconnected
      strcpy(msg, "");
      if (client.connect(alias, TOPIC_DEV_STATUS, 1, 1, "offline")) { //boolean connect (clientID, willTopic, willQoS, willRetain, willMessage)
        Serial.println("Client connected");
        Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
        Serial.println();

        flipper.detach();
        digitalWrite(WIFI_LED_PIN, HIGH); //switch off the flashing LED when connected

        //Subscribe to topics
        subscribeToTopics();

        //Publish online status

        client.publish(TOPIC_DEV_STATUS, "online", RETAINED);

        publishMyIP();
        publishMyFW();
      }
      else
      {
        delay(3000); //if disconnected from mqtt, wait for a while
      }
    }
    /**************CONNECT/RECONNECT SEQUENCE END ******************************/


    /*

       ____ ___  ____  _____
      / ___/ _ \|  _ \| ____|
      | |  | | | | |_) |  _|
      | |__| |_| |  _ <| |___
      \____\___/|_| \_\_____|

    */

    if (client.connected()) client.loop();
    if (msgReceived) processRecMessage();

    //UDP PROCESSING START

    int packetSize = UDPServer.parsePacket();
    if (packetSize)
    {
      // receive incoming UDP packets
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, UDPServer.remoteIP().toString().c_str(), UDPServer.remotePort());
      int len = UDPServer.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      Serial.printf("UDP packet contents: %s\n", incomingPacket);

  strcpy(msg, "");
  strcpy(msg, incomingPacket);
  client.publish(TOPIC_DEV_STATUS, msg);
  Serial.printf("MQTT topic: %s, message: %s\n", TOPIC_DEV_STATUS, msg);
      

      // send back a reply, to the IP address and port we got the packet from
      //UDPServer.beginPacket(UDPServer.remoteIP(), UDPServer.remotePort());
      UDPServer.beginPacket(UDPServer.remoteIP(), localUdpPort);
      UDPServer.write(replyPacket);
      UDPServer.endPacket();
    }
    //UDP PROCESSING END

    //Handle keypresses
    switch (S.read())
    {
      case MD_KeySwitch::KS_NULL:       /* Serial.println("NULL"); */   break;
      case MD_KeySwitch::KS_PRESS:      Serial.print("\nSINGLE PRESS"); break;
      case MD_KeySwitch::KS_DPRESS:     Serial.print("\nDOUBLE PRESS"); break;
      case MD_KeySwitch::KS_LONGPRESS:  Serial.print("\nLONG PRESS");   break;
      case MD_KeySwitch::KS_RPTPRESS:   Serial.print("\nREPEAT PRESS"); break;
      default:                          Serial.print("\nUNKNOWN");      break;
    }


  } //WIFI STATUS == CONNECTED

} //MAIN LOOP


