#include <WiFi.h> // Include the WiFi library for WiFi functionality|
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MQ7.h>
#include <MQ131.h>
#include <Seeed_HM330X.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Create sensor objects
HM330X sensor;
MQ7 mq7(34, 5.0);

// Define buffer for input sensor
uint8_t buf[30];

// fungsi get untuk cari PM
uint16_t getPM25(uint8_t* buffer);
uint16_t getPM10(uint8_t* buffer);

float coConcentration = 0.0;
float pm25Concentration = 0.0;
float pm10Concentration = 0.0;
float ozoneConcentration = 0.0;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

// Define MOSFET gate pins
const int gatePin1 = 27; // CO sensor
const int gatePin2 = 26; // PM sensor
const int gatePin3 = 25; // O3 sensor

// Constants for AQI calculation
const float CO_8HR_LOWER_LIMITS[] = {0.0, 4.5, 9.5, 12.5, 15.5, 30.5};
const float CO_8HR_UPPER_LIMITS[] = {4.4, 9.4, 12.4, 15.4, 30.4, 50.4};
const float CO_8HR_AQI_LOWER_LIMITS[] = {0.0, 51.0, 101.0, 151.0, 201.0, 301.0};
const float CO_8HR_AQI_UPPER_LIMITS[] = {50.0, 100.0, 150.0, 200.0, 300.0, 500.0};

const float O3_LOWER_LIMITS[] = {0.0, 55.0, 71.0, 86.0, 106.0, 201.0};
const float O3_UPPER_LIMITS[] = {54.0, 70.0, 85.0, 105.0, 200.0, 604.0};
const float O3_AQI_LOWER_LIMITS[] = {0, 55, 71, 86, 106, 201};
const float O3_AQI_UPPER_LIMITS[] = {54, 70, 85, 105, 200, 500};

const float PM25_LOWER_LIMITS[] = {0.0, 12.1, 35.5, 55.5, 150.5, 250.5};
const float PM25_UPPER_LIMITS[] = {12.0, 35.4, 55.4, 150.4, 250.4, 500.4};
const float PM25_AQI_LOWER_LIMITS[] = {0, 51, 101, 151, 201, 301};
const float PM25_AQI_UPPER_LIMITS[] = {50, 100, 150, 200, 300, 500};

const float PM10_LOWER_LIMITS[] = {0.0, 55.0, 155.0, 255.0, 355.0, 425.0};
const float PM10_UPPER_LIMITS[] = {54.0, 154.0, 254.0, 354.0, 424.0, 604.0};
const float PM10_AQI_LOWER_LIMITS[] = {0, 51, 101, 151, 201, 301};
const float PM10_AQI_UPPER_LIMITS[] = {50, 100, 150, 200, 300, 500};

// WiFi credentials
const char* ssid = "demas";
const char* password = "12345678";

// MQTT broker details
const char* mqtt_server = "31be2165576e470d81995587fdc1821a.s2.eu.hivemq.cloud";
const int mqtt_port = 8883;  // Use 8883 for secure connection
String clientID = "esp32_client";

// MQTT credentials
const char* mqtt_username = "Demass";
const char* mqtt_password = "Demass123";

// WiFi Client and MQTT client
WiFiClientSecure espClient;
PubSubClient client(espClient);

// MQTT topics
const char* coTopic = "co";
const char* pm25Topic = "pm25";
const char* pm10Topic = "pm10";
const char* o3Topic = "o3";

// WiFi Cert
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXwT0URMaFZJ4TZ2+fU93nnwHgqZ+fvQrTDYZzzL7CAkAOE8vFbJfgjPyXM75rF
o5aXcQB2QK/YkD2RhukWFoA7p5Z+ho6gCmtnBfmnZgF+Tmvl9gIJAApQGE9AQtKU
HRcQs+vXhZE9DjZZ4zIRIP4geQ7Bdgal13UT9x6zQ0zS6pHdb8xD5z65JQXgQiVI
Qjze4kBf40wWT84P9dQJvZs67Sc/X+0PAwzJXn7iWv6fZRkuLXQ7TVHgrMWIoVY=
OE0/RGOV3coDrKO5FEXXI4BRSMvG8Ts5s1vgE+7DoBIwAWvkdMSoNpIXklNoM2xw
38gntbZVfw6mxD4rBlylD7DMKD5ac0NxNsl1xH6ywbM5mn/GHj2UVU4DpD0UqtAV
U2cDYzxS5Jm26tSTU3kA
-----END CERTIFICATE-----
)EOF";


// Function to reconnect to MQTT broker
void reconnect() {
    while (!client.connected()) {
        if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
            // Successfully connected
        } else {
            // Wait and try again
            delay(5000);
        }
    }
}

// Function to initialize MQTT
void setupMQTT() {
    espClient.setCACert(root_ca);
    client.setServer(mqtt_server, mqtt_port);
    reconnect();
}

// Function to get current time
void getTime() {
    while (!timeClient.update()) {
        timeClient.forceUpdate();
    }
    formattedDate = timeClient.getFormattedDate();
    int splitT = formattedDate.indexOf("T");
    dayStamp = formattedDate.substring(0, splitT);
    timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
}

float calculateAQI(float concentration, const float* lower_limits, const float* upper_limits, const float* aqi_lower_limits, const float* aqi_upper_limits) {
  for (int i = 0; i < 6; ++i) {
    if (concentration >= lower_limits[i] && concentration <= upper_limits[i]) {
      return ((aqi_upper_limits[i] - aqi_lower_limits[i]) / (upper_limits[i] - lower_limits[i])) * (concentration - lower_limits[i]) + aqi_lower_limits[i];
    }
  }
  return -1; // Invalid concentration
}

// Function to collect and publish sensor data
void collectAndPublishData() {

    // Enable sensors
    digitalWrite(gatePin1, HIGH); // CO sensor
    digitalWrite(gatePin2, HIGH); // PM sensor
    digitalWrite(gatePin3, HIGH); // O3 sensor
    delay(500); // Ensure sensors are stable

    // Collect O3 data
    analogReadResolution(10);
    MQ131.begin(2, 35, LOW_CONCENTRATION, 1000, (Stream *)&Serial);
    delay(1000);  
    
    while(sensor.init()){
      Serial.println("sensor failed");
    }

    Serial.println("Calibration in progress...");
      
    MQ131.calibrate();

    
    Serial.println("Calibration done!");
    Serial.print("R0 = ");
    Serial.print(MQ131.getR0());
    Serial.println(" Ohms");
    Serial.print("Time to heat = ");
    Serial.print(MQ131.getTimeToRead());
    Serial.println(" s");
    Serial.println("Sampling...");
    MQ131.sample();
    ozoneConcentration = MQ131.getO3(UG_M3);
    float ozCon = static_cast<float>(MQ131.getO3(UG_M3));
    Serial.print("analog of O3:");
    Serial.println(analogRead(35));
    Serial.print("Concentration O3 : ");
    Serial.print(ozoneConcentration);
    Serial.println(" ug/m3");
    // int aqiO3 = calculateAQI_O3(ozoneConcentration, O3_AQI_LOWER_LIMITS, O3_AQI_UPPER_LIMITS);
    // printAirQualityCategory(aqiO3);
    // //Publish O3 TOPIC
    // client.publish(o3Topic, String(o3AQI).c_str());

    // Collect CO data
    float coConcentration = mq7.getPPM();
    // int airQualityIndex = calculateAQI_CO(coConcentration);
    // printAQI(airQualityIndex);
    // //PUBLISH CO TOPIC
    // client.publish(coTopic, co_msg.c_str());

    delay(1000); // Wait for a second
    
    // Collect PM data
    while(sensor.init()){
    Serial.println("sensor failed");
  }
    if(sensor.read_sensor_value(buf, 29)){
    Serial.println("FAILED GETTING SENSOR DATA");
    } 
    // checksum dulu gaes
  uint8_t sum = 0;
  for (int i = 0; i < 28; i++) {
    sum += buf[i];
  }
  if (sum != buf[28]) {
    Serial.println("wrong checkSum!!");
  }

  // Mendapatkan nilai PM2.5 dan PM10 dari sensor
  float pm25Concentration = getPM25(buf);
  float pm10Concentration = getPM10(buf);

  // Menghitung AQI
  float coAQI = calculateAQI(coConcentration, CO_8HR_LOWER_LIMITS, CO_8HR_UPPER_LIMITS, CO_8HR_AQI_LOWER_LIMITS, CO_8HR_AQI_UPPER_LIMITS);
  float o3AQI = calculateAQI(ozoneConcentration, O3_LOWER_LIMITS, O3_UPPER_LIMITS, O3_AQI_LOWER_LIMITS, O3_AQI_UPPER_LIMITS);
  float pm25AQI = calculateAQI(pm25Concentration, PM25_LOWER_LIMITS, PM25_UPPER_LIMITS, PM25_AQI_LOWER_LIMITS, PM25_AQI_UPPER_LIMITS);
  float pm10AQI = calculateAQI(pm10Concentration, PM10_LOWER_LIMITS, PM10_UPPER_LIMITS, PM10_AQI_LOWER_LIMITS, PM10_AQI_UPPER_LIMITS);

  // Menampilkan nilai PM2.5 dan PM10 beserta kategori AQI
  Serial.print("Nilai dari PM2.5 dalam μg/m3 adalah: ");
  Serial.println(pm25Concentration);
  Serial.print("Nilai dari PM10 dalam μg/m3 adalah: ");
  Serial.println(pm10Concentration);
  Serial.println();

// Publish data to MQTT
  if (client.publish(coTopic, String(coAQI).c_str(), true)) {
    Serial.println("CO AQI published successfully");
  } else {
    Serial.println("CO AQI publish failed");
  }

  if (client.publish(pm25Topic, String(pm25AQI).c_str(), true)) {
    Serial.println("PM2.5 AQI published successfully");
  } else {
    Serial.println("PM2.5 AQI publish failed");
  }

  if (client.publish(pm10Topic, String(pm10AQI).c_str(), true)) {
    Serial.println("PM10 AQI published successfully");
  } else {
    Serial.println("PM10 AQI publish failed");
  }

  if (client.publish(o3Topic, String(o3AQI).c_str(), true)) {
    Serial.println("O3 AQI published successfully");
  } else {
    Serial.println("O3 AQI publish failed");
  }

  // // Menampilkan kategori AQI untuk PM2.5
  // Serial.print("Kategori AQI PM2.5: ");
  // printAirQualityCategory(aqiPM25);

  // // Menampilkan kategori AQI untuk PM10
  // Serial.print("Kategori AQI PM10: ");
  // printAirQualityCategory(aqiPM10);
  // Serial.println();

  
  // //publish PM TOPIC
  // client.publish(pm25Topic, String(pm25AQI).c_str());
  // client.publish(pm10Topic, String(pm10AQI).c_str());


  // Disable sensors
  digitalWrite(gatePin1, LOW); // CO sensor
  digitalWrite(gatePin2, LOW); // PM sensor
  digitalWrite(gatePin3, LOW); // O3 sensor
}

// Function to handle deep sleep
void go_sleep() {
  // Update NTP time
  timeClient.update();
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  if (currentHour == 7 && currentMinute <= 10) {
    run();
  } else if (currentHour == 12 && currentMinute <= 10) {
    run();
  } else if (currentHour == 17 && currentMinute <= 10) {
    run();
  } else {
    sleep_ass(currentHour, currentMinute);
  }
}

void sleep_ass(int currentHour, int currentMinute) {
  long sleepSeconds = 0;
  
  if ((currentHour < 7) || (currentHour >= 17 && currentHour < 24)) {
    // Sleep until 7:00 AM
    int hourToSleep = (currentHour < 7) ? (7 - currentHour - 1) : (24 + 7 - currentHour - 1);
    int minuteToSleep = 60 - currentMinute;
    minuteToSleep = (minuteToSleep == 60) ? 0 : minuteToSleep;
    sleepSeconds = hourToSleep * 3600 + minuteToSleep * 60;
  } else if (currentHour >= 7 && currentHour < 12) {
    // Sleep until 12:00 PM
    int hourToSleep = 12 - currentHour - 1;
    int minuteToSleep = 60 - currentMinute;
    minuteToSleep = (minuteToSleep == 60) ? 0 : minuteToSleep;
    sleepSeconds = hourToSleep * 3600 + minuteToSleep * 60;
  } else if (currentHour >= 12 && currentHour < 17) {
    // Sleep until 5:00 PM
    int hourToSleep = 17 - currentHour - 1;
    int minuteToSleep = 60 - currentMinute;
    minuteToSleep = (minuteToSleep == 60) ? 0 : minuteToSleep;
    sleepSeconds = hourToSleep * 3600 + minuteToSleep * 60;
  }

  // Convert sleep seconds to milliseconds
  sleepSeconds *= 1000;

  // Unit Test
  Serial.print("Sleeping for: ");
  Serial.print(sleepSeconds / 1000);
  Serial.println(" seconds");

  esp_sleep_enable_timer_wakeup(sleepSeconds * 1000);
  esp_deep_sleep_start();
}


void setup() {
    // Initialize serial communication
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
    Serial.println("Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Initialize pins
    pinMode(gatePin1, OUTPUT);
    pinMode(gatePin2, OUTPUT);
    pinMode(gatePin3, OUTPUT);

    // Initialize MQTT
    setupMQTT();

    // Initialize NTP client
    timeClient.begin();
    timeClient.setTimeOffset(3600); // Adjust time offset as needed
   
    run();
    go_sleep();
}

void loop() {
    // TIDAK DIPANGGIL
    }
  void run (){
    getTime();
    collectAndPublishData();
  }
uint16_t getPM25(uint8_t* buffer){
  return (uint16_t) buffer[6] << 8 | buffer[7];
}
  uint16_t getPM10(uint8_t* buffer){
  return (uint16_t) buffer[8] << 8 | buffer[9];
}
