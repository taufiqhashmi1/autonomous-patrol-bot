#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ----- WiFi Credentials -----
const char* ssid = "MyWiFiCar";
const char* password = "12345678";

// ----- Pin Definitions -----
#define TRIG_PIN 16
#define ECHO_PIN 0
#define DHT_PIN 14

#define I2C_SDA 15
#define I2C_SCL 2

// ----- Sensor & Server Objects -----
DHT dht(DHT_PIN, DHT11);
Adafruit_MPU6050 mpu;
AsyncWebServer server(80);

// ----- State Flag for MPU6050 -----
bool mpuAvailable = false; // This flag will track if the MPU is connected

// ----- HTML Page with CSS and JavaScript -----
const char* htmlPage PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
  <title>ESP32 Sensor Hub</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #f4f4f4; margin-top: 20px; }
    h2 { color: teal; }
    h3 { color: #444; margin-top: 25px; border-bottom: 1px solid #ccc; padding-bottom: 5px; }
    .container { display: flex; flex-wrap: wrap; justify-content: center; }
    .sensorBox { 
      margin: 8px; 
      padding: 12px; 
      background: white; 
      border-radius: 10px; 
      box-shadow: 2px 2px 8px rgba(0,0,0,0.2); 
      font-size: 22px;
      min-width: 160px;
    }
    .label { font-size: 14px; color: #555; }
  </style>
</head>
<body>
  <h2>ESP32 Real-Time Sensor Data</h2>
  
  <h3>Environment</h3>
  <div class="container">
    <div class="sensorBox"><span class="label">Distance</span><br><span id="distance">--</span> cm</div>
    <div class="sensorBox"><span class="label">Temperature</span><br><span id="temperature">--</span> &deg;C</div>
    <div class="sensorBox"><span class="label">Humidity</span><br><span id="humidity">--</span> %</div>
  </div>

  <h3>Inertial Measurement (MPU-6050)</h3>
  <div class="container">
    <div class="sensorBox"><span class="label">Accel X</span><br><span id="accelX">--</span> m/s²</div>
    <div class="sensorBox"><span class="label">Accel Y</span><br><span id="accelY">--</span> m/s²</div>
    <div class="sensorBox"><span class="label">Accel Z</span><br><span id="accelZ">--</span> m/s²</div>
  </div>
  <div class="container">
    <div class="sensorBox"><span class="label">Gyro X</span><br><span id="gyroX">--</span> rad/s</div>
    <div class="sensorBox"><span class="label">Gyro Y</span><br><span id="gyroY">--</span> rad/s</div>
    <div class="sensorBox"><span class="label">Gyro Z</span><br><span id="gyroZ">--</span> rad/s</div>
  </div>

<script>
function fetchSensorData() {
  fetch("/sensors")
    .then(response => response.json())
    .then(data => {
      document.getElementById("distance").innerText = data.distance;
      document.getElementById("temperature").innerText = data.temperature;
      document.getElementById("humidity").innerText = data.humidity;
      document.getElementById("accelX").innerText = data.accelX;
      document.getElementById("accelY").innerText = data.accelY;
      document.getElementById("accelZ").innerText = data.accelZ;
      document.getElementById("gyroX").innerText = data.gyroX;
      document.getElementById("gyroY").innerText = data.gyroY;
      document.getElementById("gyroZ").innerText = data.gyroZ;
    })
    .catch(err => console.error(err));
}

setInterval(fetchSensorData, 1000);
window.onload = fetchSensorData;
</script>
</body>
</html>
)rawliteral";

// ----- Function: Get Ultrasonic Distance -----
long getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  return (duration == 0) ? 0 : distance;
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); 

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  dht.begin();

  // --- MODIFIED MPU6050 INITIALIZATION ---
  // Try to initialize the MPU6050, but don't stop if it fails.
  if (mpu.begin()) {
    mpuAvailable = true;
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  } else {
    Serial.println("MPU6050 not found. Continuing without it.");
  }
  
  WiFi.softAP(ssid, password);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", htmlPage);
  });

  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request){
    long distance = getUltrasonicDistance();
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Create variables for MPU data with default values
    float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
    float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;

    // --- MODIFIED MPU6050 READING ---
    // Only read from the MPU if it was available during setup
    if (mpuAvailable) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      accelX = a.acceleration.x;
      accelY = a.acceleration.y;
      accelZ = a.acceleration.z;
      gyroX = g.gyro.x;
      gyroY = g.gyro.y;
      gyroZ = g.gyro.z;
    }

    if (isnan(temp)) temp = 0.0;
    if (isnan(humidity)) humidity = 0.0;
    
    String json = "{";
    json += "\"distance\":" + String(distance) + ",";
    json += "\"temperature\":" + String(temp, 1) + ",";
    json += "\"humidity\":" + String(humidity, 1) + ",";
    json += "\"accelX\":" + String(accelX, 2) + ",";
    json += "\"accelY\":" + String(accelY, 2) + ",";
    json += "\"accelZ\":" + String(accelZ, 2) + ",";
    json += "\"gyroX\":" + String(gyroX, 2) + ",";
    json += "\"gyroY\":" + String(gyroY, 2) + ",";
    json += "\"gyroZ\":" + String(gyroZ, 2);
    json += "}";
    
    request->send(200, "application/json", json);
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Nothing to do here
}