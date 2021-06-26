
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

#include<dht.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

//Variables for Wi-Fi connection
#ifndef STASSID
#define STASSID "Malibongwe"
#define STAPSK  "Hephzibah"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = "http://192.168.43.210";
const uint16_t port = 80;

//Pins for Neo 6M GPS module
int RXPin = D3;
int TXPin = D4;

SoftwareSerial gpsSerial(RXPin, TXPin);

//Pin for DHT11 (Humidity cum temperature sensor)
int DHT_pin = D2;

//Pin for pulse rate sensor
int BPM_pin = A0;

TinyGPSPlus GPS, retGPS;

dht DHT, retDHT;

float latitude, longitude;

int BPM, retBPM; int intervalCounter = 0;

#define PN532_SCK  (D5)
#define PN532_MOSI (D6)
#define PN532_SS   (D7)
#define PN532_MISO (D8)

#define PN532_IRQ   (D5)
#define PN532_RESET (D6)

Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

#if defined(ARDUINO_ARCH_SAMD)
   #define Serial SerialUSB
#endif

void setup() {
  Serial.begin(9600);

  //Setting up the GPS module
  gpsSerial.begin(9600);

  //Setting up the Wi-Fi module
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }

  Serial.println("");
  Serial.print(" WiFi connected - IP address: " + WiFi.localIP());

  //Setting up the NFC reader module
  #ifndef ESP8266
    while (!Serial); // for Leonardo/Micro/Zero
  #endif

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    while (1); //Halt
  }
  
  nfc.SAMConfig(); //Configure board to read RFID tags

  if(WiFi.status() == WL_CONNECTED){
    Serial.println("Already connected to the network.");
  }else{
    Serial.println("Not connected to any network.");
  }
}

void loop() {
  //Counter to determine when data will be sent to server
  intervalCounter++;

  String UUID = readNFC();
  dht retDHT = readHumidityTemperature();
  retBPM = readBMP();
  retGPS = readGPS();

  if(retGPS.location.isValid()){
    latitude = retGPS.location.lat();
    longitude = retGPS.location.lng();
  }else{
    latitude = getLatitude();
    longitude = getLongitude();
  }

  Serial.println("Cattle UUID: " + String(UUID));
  Serial.println("Humidity: " + String(retDHT.humidity));
  Serial.println("Temperature: " + String(retDHT.temperature));
  Serial.println("BMP: " + String(retBPM));
  Serial.println("Latitude: " + String(latitude));
  Serial.println("Longitude: " + String(longitude));

  Serial.println();
  Serial.println();

  if(intervalCounter%4 == 0){
    sendLivestockData(String(UUID), String(retDHT.temperature), String(retDHT.humidity), String(retBPM), String(latitude), String(longitude));
    Serial.println("Sent to remote server...");
    Serial.println();
  }
  
  delay(5000);
}

dht readHumidityTemperature(){
  int chk = DHT.read11(DHT_pin);
  return DHT;
}

int readBMP(){
  BPM = analogRead(BPM_pin);
  BPM = (BPM / 10) * 1.2;
  return BPM;
}

TinyGPSPlus readGPS(){
  while(gpsSerial.available() > 0){
    if (GPS.encode(gpsSerial.read())){
      return GPS;
    }
  }
  return GPS;
}

float getLatitude(){
  return 5.4801;
}

float getLongitude(){
  return 7.5437;
}

String readNFC(){
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  uint8_t uidLength; 
    
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  
  if(success){
    //The commented code print the hexadecimal representation of UID bytes
    //nfc.PrintHex(uid, uidLength);

    //Prepare the UID as String
    uint32_t cardId = uid[0];
    cardId <<= 8;
    cardId |= uid[1];
    cardId <<= 8;
    cardId |= uid[2];  
    cardId <<= 8;
    cardId |= uid[3];

    String s(cardId);

    return s;
  }

  return "";
}

void sendLivestockData(String NFC_UUID, String temperature, String humidity, String pulseRate, String locLatitude, String locLongitude){
  //API end point
  String endPoint = "/API/public/api/remote_livestock_monitoring/livestock_data/" + NFC_UUID + "/" + temperature + "/" + humidity + "/" + pulseRate + "/" + locLatitude + "/" + locLongitude;

  //Prepare and start sending to API
  HTTPClient http;
  String link = String(host) + endPoint;
  http.begin(link);
  int httpCode = http.GET();
  String payload = http.getString();
  Serial.print("Status code: "); Serial.println(httpCode);
  Serial.print("Data: "); Serial.println(payload);
  http.end();
}
