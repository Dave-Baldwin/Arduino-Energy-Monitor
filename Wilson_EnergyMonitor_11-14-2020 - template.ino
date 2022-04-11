/*
  EmonTx CT123 Voltage Serial Only example
  
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
  
  Author: Trystan Lea
*/

// libraries for power/temp measurement
#include "EmonLib.h"		//// will likely have to be downloaded and added via Library Manager
#include <OneWire.h>
#include <DallasTemperature.h>

// libraries for ethernet / MQTT
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xD0, 0xC0 };	//// may have to be identified based on router data - the ethernet shield may or may not have a sticker with MAC on it - and this MAC is likely from a different board/project

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 1, 177);		//// set as desired
IPAddress mqttServer(192, 168, 1, 100);		//// you need an MQTT broker configured

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish("initTopic","hello world");
      // ... and resubscribe
      mqttClient.subscribe("initTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Create  instances for each CT channel
EnergyMonitor ct1,ct2,ct3,ct4;

//#define ONE_WIRE_BUS 4                                                  // Data wire is plugged into port 2 on the Arduino
//OneWire oneWire(ONE_WIRE_BUS);                                          // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//DallasTemperature sensors(&oneWire);                                    // Pass our oneWire reference to Dallas Temperature.

//float garageTemp = 0;

long switchLedMillis = 0;
long ledMillis = 1000;

long lastDispMillis = 0;
long serialDispIntMillis = 2000;

long lastRptMillis = 0;
long reptIntMillis = 5000;

long stabilizeMillis = 20000;
boolean valsStable = false;

// By using direct addressing its possible to make sure that as you add temperature sensors, the temperature sensor to variable mapping will not change.
// The below address was found by using a different sketch to determine temperature sensor ID..
//DeviceAddress address_T1 = { 0x28, 0x79, 0x8A, 0x34, 0x05, 0x00, 0x00, 0x15 };

// On-board emonTx LED
const int LEDpin = 9;                                                    
boolean digPin = false;
  
void setup() 
{
  
  // Setup indicator LED
  pinMode(LEDpin, OUTPUT);
  // oscillate pin
  if (digPin == false) {
    digitalWrite(LEDpin, HIGH);
    digPin = true;
  } else {
    digitalWrite(LEDpin, LOW);
    digPin = false;
  }  

  // begin serial communications..
  Serial.begin(9600);

  // set MQTT server info
  mqttClient.setServer(mqttServer, 1883);	//// port 1883 by default for MQTT when not using secured comms
  mqttClient.setCallback(callback);
  
  // wait for serial port to connect
  delay(2000);

  // initial print to serial..
  Serial.println("EMon Voltage/Power/Temp"); 
  
  // start the Ethernet connection:
  Serial.println("Init Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Fail to config Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield not found.  Can't run without hardware.");
      while (true) {
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable not connected.");
    }
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  } else {
    Serial.print("DHCP assigned IP: ");
    Serial.println(Ethernet.localIP());
  }
  // give the Ethernet shield a second to initialize:
  delay(1000);

	//// looks like I had pre-filled in these calibration constants below - you should be able to simply ensure the right CTs are plugged into the related plugs per the comments below, and stuff should automagically work?
	//// for interest, this Arduino shield you are using is described here: https://shop.openenergymonitor.com/emontx-arduino-shield-smt/
	//// and how to build/use this system is documented here: https://learn.openenergymonitor.org/electricity-monitoring/ctac/how-to-build-an-arduino-energy-monitor

  // Calibration factor = CT ratio / burden resistance = (100A / 0.05A) / 33 Ohms = 60.606
  ct1.current(1, 181.836365);     // channel1 is a 200A max SCT-019 - Calibration factor = CT ratio / burden resistance = (200A / 0.033A) / 33 Ohms = 181.836
  ct2.current(2, 60.606060);         // channel2 is a 100A max SCT-013 - Calibration factor = CT ratio / burden resistance = (100A / 0.050A) / 33 Ohms = 60.60606060...                                
  ct3.current(3, 60.606060);         // channel3 is a 100A max SCT-013 - calib factor as above
  ct4.current(4, 181.836365);         // channel4 is a 200A max SCT-019 - Calibration factor = CT ratio / burden resistance = (200A / 0.033A) / 33 Ohms = 181.836
  
  //// Depending on what circuits you are going to measure, you may need to modify the below
  //// if measuring 240VAC circuits, You should be able to leave the 284.4 values below.
  //// if measuring 120VAC circuits, you should cut the calibration value below in half.
  
  // (ADC input, calibration, phase_shift)
  ct1.voltage(0, 284.4, 1.7);     // measured 11/8/2020 - whole house incoming voltage of 247.5 VAC / 0.873 VAC at ADC pin = 283.5052 voltage calibration constant
  ct2.voltage(0, 284.4, 1.7);     // dryer circuit; this is 240VAC 
  ct3.voltage(0, 284.4, 1.7);    // HVAC circuit; this is 240VAC
  ct4.voltage(0, 284.4, 1.7);  // oven circuit; this is 240VAC

  // for any 120VAC circuits, we'll have to cut the voltage constant above in half ...
  
  // start temperature sensing..
  //sensors.begin();
  
}

void loop() 
{ 

  // if stabilization time has passed, then flip the stabilization flag and allow reporting
  if (millis() > stabilizeMillis && !valsStable) {
    valsStable = true;
  }
  
  // Send the command to get temperatures
  //sensors.requestTemperatures();
  //garageTemp = sensors.getTempF(address_T1);
  
  // Calculate all. No.of crossings, time-out 
  ct1.calcVI(20,2000);                                                  
  ct2.calcVI(20,2000);
  ct3.calcVI(20,2000);
  ct4.calcVI(20,2000);

  // if the proper time has elapsed to report results via Serial, then do it ..
  if (millis() - lastDispMillis > serialDispIntMillis) {

	//// depending on what circuits you are measuring, you should modify the Serial print comments below accordingly.

    // Print summary
    Serial.println("-");
    Serial.print("Net power W: ");
    Serial.println(ct1.realPower);     
    Serial.print("Dryer power W: ");
    Serial.println(ct2.realPower);
    Serial.print("HVAC power W: ");
    Serial.println(ct3.realPower);
    Serial.print("Oven power W: ");
    Serial.println(ct4.realPower);        
    Serial.print("Mains volts V: ");
    Serial.println(ct1.Vrms);
    Serial.print("Garage temp: ");
    Serial.println(garageTemp, 1);
      
    // Available properties: ct1.realPower, ct1.apparentPower, ct1.powerFactor, ct1.Irms and ct1.Vrms

    // reset timer for periodic serial display
    lastDispMillis = millis();
    
  }

  // oscillate pin
  if (millis() - ledMillis > switchLedMillis && digPin) {
    digitalWrite(LEDpin, HIGH);
    digPin = false;
    switchLedMillis = millis();
  } else {
    digitalWrite(LEDpin, LOW);
    digPin = true;
    switchLedMillis = millis();
  }

  // if not connected to MQTT server, then reconnect?
  if (!mqttClient.connected()) {
    reconnect();
  }

  // if MQTT client is connected, and data has stabilized, then send latest values..
  if (mqttClient.connected() && (millis() - lastRptMillis > reptIntMillis) && valsStable) {

    Serial.println("Report vals to MQTT broker.");

    // define character array for sending string to MQTT broker
    char str[5];

	//// suffice to say, dealing with strings and character arrays in C is super annoying. The below should work without modification and without causing memory issues.
	//// it can be tempting to use the "String" class in C (capital-S); it is much easier to deal with, but in my experience inevitably ends up causing just as many problems as it solves.

    // net house power
    dtostrf(ct1.realPower,4,3,str);
    mqttClient.publish("Wilson/NetPower_W", str);

    // Dryer power
    dtostrf(ct2.realPower,4,3,str);    
    mqttClient.publish("Wilson/DryerPower_W", str);

    // HVAC power
    dtostrf(ct3.realPower,4,3,str);    
    mqttClient.publish("Wilson/HVACPower_W", str);

    // Oven power
    dtostrf(ct4.realPower,4,3,str);    
    mqttClient.publish("Wilson/OvenPower_W", str);

    // garage temperature
    dtostrf(garageTemp,4,3,str);    
    mqttClient.publish("Wilson/GarageTemp_degF", str);

    // mains voltage
    dtostrf(ct1.Vrms,4,3,str);    
    mqttClient.publish("Wilson/MainsVoltage_V", str);
    
    // update delay timer for reporting..
    lastRptMillis = millis();
    
  } // end if statement to report via MQTT

  // refresh MQTT client?
  mqttClient.loop();

  // comment out delay on 11/14/2020; not sure we need it ..
  //delay(100);
}
