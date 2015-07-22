/*
 Based on work from author:  Eric Tsai
 Gateway ncorporating both the RFM69 and the ethernet part
 Revised by Alexandre Bouillot
 
 License:  CC-BY-SA, https://creativecommons.org/licenses/by-sa/2.0/
 Date:  10-23-2014
 File: Gateway.ino
 This sketch receives RFM wireless data and forwards it to Mosquitto relay
 
 Modifications Needed:
 1)  Update encryption string "ENCRYPTKEY"
 2)  Adjust SS - Chip Select - for RFM69
 3)  Adjust MQTT server address
 */

/*
RFM69 Pinout:
 MOSI = 11
 MISO = 12
 SCK = 13
 SS = 8
 */

/*
Ethernet Pinout:
 MOSI = 11
 MISO = 12
 SCK = 13
 SS = 10
 */

//general --------------------------------
#define SERIAL_BAUD   115200
#if 0
#define DEBUG1(expression)  Serial.print(expression)
#define DEBUG2(expression, arg)  Serial.print(expression, arg)
#define DEBUGLN1(expression)  Serial.println(expression)
#else
#define DEBUG1(expression)
#define DEBUG2(expression, arg)
#define DEBUGLN1(expression)
#endif
//RFM69  ----------------------------------
#include <RFM69.h>
#include <SPI.h>
#define ACCOUNTID     1001
#define GATEWAYID     1
#define NODEID        1    //unique for each node on same network
#define NETWORKID     101  //the same on all nodes that talk to each other
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "xxxxxxxxxxxxxxxx" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME      30 // max # of ms to wait for an ack
#define RFM69_SS  8
RFM69 radio(RFM69_SS);
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

#include <EthernetV2_0.h>

//Ethernet
byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};
byte server[] = { 
  54, 173, 145, 41 };

//IPAddress ip(192,168,0,91);
EthernetClient ethClient;
#define DHCP_RETRY 5000
#define W5200_CS  10
#define SDCARD_CS 4

//use LED for indicating MQTT connection status.
int led = 7;
int radioLed = 6;
int radioACKLed = 5;
int mqttConnectActivityLed = 3;

// Mosquitto---------------
#include <PubSubClient.h>
PubSubClient client(server, 1883, callback, ethClient);
#define MQTT_CLIENT_ID "arduinoClient"
#define MQTT_RETRY 500
int sendMQTT = 0;

void MQTTSendInt(PubSubClient* _client, int node, int sensor, int var, int val);
void MQTTSendULong(PubSubClient* _client, int node, int sensor, int var, unsigned long val);
void MQTTSendFloat(PubSubClient* _client, int node, int sensor, int var, float val);
void MQTTSendInfo(PubSubClient* _client, int accountID, int gatewayID, int nodeID, int sensorID, float sensorValue);


typedef struct {		
  int                   nodeID; 
  int			sensorID;
  unsigned long         var1_usl; 
  float                 var2_float; 
  float			var3_float;	
} 
Payload;
Payload theData;

volatile struct 
{
  int                   nodeID;
  int			sensorID;		
  unsigned long         var1_usl;
  float                 var2_float;
  float			var3_float;		//
  int                   var4_int;
} 
SensorNode;

//Arduino reset function to work around MQTT/EThernet crashing after 23h of straight usage
void(* resetFunc) (void) = 0; //declare reset function @ address 0


void setup() 
{
  
  //Serial.begin(SERIAL_BAUD); 
DEBUGLN1("Setting Up");
  //Ethernet -------------------------
  //Ethernet.begin(mac, ip);
  
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); //Deselect the SD card

 pinMode(led, OUTPUT);
  pinMode(radioLed, OUTPUT);
  pinMode(radioACKLed,OUTPUT);
  pinMode(mqttConnectActivityLed,OUTPUT);

digitalWrite(led, HIGH);
  //wait for IP address
  while (Ethernet.begin(mac) != 1) {
    DEBUGLN1("Error getting IP address via DHCP, trying again...");
    delay(DHCP_RETRY);
  }
  
  DEBUGLN1("ethernet OK");
  digitalWrite(radioLed, HIGH);
  
  // print your local IP address:
  DEBUGLN1("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    DEBUG2(Ethernet.localIP()[thisByte], DEC);
    DEBUG1("."); 
  }
  DEBUGLN1();

  DEBUGLN1("Connecting to MQTT");
  // Mosquitto ------------------------------
  while (client.connect(MQTT_CLIENT_ID) != 1) {
    DEBUGLN1("Error connecting to MQTT");
    delay(MQTT_RETRY);
  }
    DEBUGLN1("MQTT OK");
      digitalWrite(mqttConnectActivityLed, HIGH);

  //RFM69 ---------------------------
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGLN1(buff);
  digitalWrite(radioACKLed, HIGH);

  DEBUGLN1("setup complete");
  digitalWrite(led, LOW);
  digitalWrite(radioLed, LOW);
  digitalWrite(radioACKLed, LOW);
  digitalWrite(mqttConnectActivityLed, LOW);
}  // end of setup

byte ackCount=0;
long watchdogInterval = 10000;
long watchdog = 0;
boolean ledon=false;

void loop() {

  // calling client.loop too often block the system at some point quite early (~up to 5 loop)
  // Here is a temporized call to it on a regular interval
  // This need to be as fast as the fastest sensor received
  if (millis() > watchdog) {
    //    Serial.print("loop "); 
    //    Serial.println(millis());

    if(millis() > 83000000) (resetFunc());  //call reset
    
    watchdog += watchdogInterval;
    char buf[50];
    ltoa(watchdog,buf,10);
    DEBUGLN1(watchdog);
    client.publish("$GWSYS/Watchdog/Value", buf);
    DEBUGLN1("Sent Watchdog value");//client.loop needs to run every iteration.  Previous version did not.  Big opps.
    client.loop();
    DEBUGLN1("Client Looped");
    if(ledon==false){
      digitalWrite(led, HIGH);
      ledon=true;
    }else{
       digitalWrite(led, LOW);
       ledon=false;
    }
  }

  if (radio.receiveDone()) {
    digitalWrite(radioLed, HIGH);
    DEBUG1('[');
    DEBUG2(radio.SENDERID, DEC);
    DEBUG1("] ");
    if (promiscuousMode) {
      DEBUG1("to [");
      DEBUG2(radio.TARGETID, DEC);
      DEBUG1("] ");
    }
    DEBUGLN1();

    if (radio.DATALEN != sizeof(Payload))
      Serial.println(F("Invalid payload received, not matching Payload struct!"));
    else {
      theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else

      //save it for i2c:
      SensorNode.nodeID = theData.nodeID;
      SensorNode.sensorID = theData.sensorID;
      SensorNode.var1_usl = theData.var1_usl;
      SensorNode.var2_float = theData.var2_float;
      SensorNode.var3_float = theData.var3_float;
      SensorNode.var4_int = radio.RSSI;

      DEBUG1("Received Device ID = ");
      DEBUGLN1(SensorNode.sensorID);  
      DEBUG1 ("    Time = ");
      DEBUGLN1 (SensorNode.var1_usl);
      DEBUG1 ("    var2_float ");
      DEBUGLN1 (SensorNode.var2_float);
      DEBUG1 ("    var3_float ");
      DEBUGLN1 (SensorNode.var3_float);
      DEBUG1 ("    RSSI ");
      DEBUGLN1 (SensorNode.var4_int);

      sendMQTT = 1;
      digitalWrite(radioLed, LOW);
    }


    if (radio.ACK_REQUESTED)
    {
      digitalWrite(radioACKLed, HIGH);
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      digitalWrite(radioACKLed, LOW);
      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        //Serial.print(" Pinging node ");
        //Serial.print(theNodeID);
        //Serial.print(" - ACK...");
        //delay(3); //need this when sending right after reception .. ?
        //if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
        //  Serial.print("ok!");
        //else Serial.print("nothing");
      }
    }//end if radio.ACK_REQESTED
  } //end if radio.receive

  if (sendMQTT == 1) { // Is == 1 but testing how is goes without MQTT
    DEBUGLN1("starting MQTT send");

    if (!client.connected()) {
      while (client.connect(MQTT_CLIENT_ID) != 1)
      {
        digitalWrite(mqttConnectActivityLed, HIGH);
        DEBUGLN1("Error connecting to MQTT");
        delay(500);
        digitalWrite(mqttConnectActivityLed, LOW);
      }
      client.publish("outTopic","hello world");
    } 

    digitalWrite(led, HIGH);

    int varnum;
    char buff_topic[6];
    char buff_message[12];      

    /*
      //send var1_usl
     varnum = 2;
     buff_topic[6];
     buff_message[12];
     sprintf(buff_topic, "%02d%01d%01d", SensorNode.nodeID, SensorNode.sensorID, varnum);
     Serial.println(buff_topic);
     dtostrf (SensorNode.var1_usl, 10, 1, buff_message);
     client.publish(buff_topic, buff_message);
     */

    MQTTSendInfo(&client, ACCOUNTID, GATEWAYID, SensorNode.nodeID, SensorNode.sensorID, SensorNode.var1_usl);
    MQTTSendInfo(&client, ACCOUNTID, GATEWAYID, SensorNode.nodeID, SensorNode.sensorID+1, SensorNode.var2_float);
    MQTTSendInfo(&client, ACCOUNTID, GATEWAYID, SensorNode.nodeID, 0, SensorNode.var3_float); //Communicating sensor "0" = battery level of the node

/*
    //send var1_usl
    MQTTSendULong(&client, SensorNode.nodeID, SensorNode.sensorID, 1, SensorNode.var1_usl);

    //send var2_float
    MQTTSendFloat(&client, SensorNode.nodeID, SensorNode.sensorID, 2, SensorNode.var2_float);

    //send var3_float
    MQTTSendFloat(&client, SensorNode.nodeID, SensorNode.sensorID, 3, SensorNode.var3_float);

    //send var4_int, RSSI
    MQTTSendInt(&client, SensorNode.nodeID, SensorNode.sensorID, 4, SensorNode.var4_int);
*/
    sendMQTT = 0;
    DEBUGLN1("finished MQTT send");
    digitalWrite(led, LOW);
  }//end if sendMQTT
  
}//end loop

void MQTTSendInt(PubSubClient* _client, int node, int sensor, int var, int val) {
  char buff_topic[6];
  char buff_message[7];

  sprintf(buff_topic, "%02d%01d%01d", node, sensor, var);
  sprintf(buff_message, "%04d%", val);
  _client->publish(buff_topic, buff_message);
}

void MQTTSendULong(PubSubClient* _client, int node, int sensor, int var, unsigned long val) {
  char buff_topic[6];
  char buff_message[12];

  sprintf(buff_topic, "%02d%01d%01d", node, sensor, var);
  sprintf(buff_message, "%u", val);
  _client->publish(buff_topic, buff_message);
}

void MQTTSendFloat(PubSubClient* _client, int node, int sensor, int var, float val) {
  char buff_topic[6];
  char buff_message[12];

  sprintf(buff_topic, "%02d%01d%01d", node, sensor, var);
  dtostrf (val, 2, 1, buff_message);
  _client->publish(buff_topic, buff_message);
}

void MQTTSendInfo(PubSubClient* _client, int accountID, int gatewayID, int nodeID, int sensorID, float sensorValue) {
  char buff_topic[15];
  char buff_message[12];

  sprintf(buff_topic, "%04d/%03d/%02d/%01d", accountID, gatewayID, nodeID, sensorID);
  dtostrf (sensorValue, 2, 1, buff_message);
  _client->publish(buff_topic, buff_message);
}


// Handing of Mosquitto messages
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  DEBUGLN1(F("Mosquitto Callback"));
}






