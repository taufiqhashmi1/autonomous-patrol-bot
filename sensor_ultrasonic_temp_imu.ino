#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <DHT.h>

// ----- WiFi Credentials -----
const char* ssid = "MyWiFiCar";
const char* password = "12345678";

// ----- Pin Definitions -----
#define TRIG_PIN 12  // Safe to use (was 16)
#define ECHO_PIN 13  // Safe to use (was 0)
#define DHT_PIN 2    // Safe to use (was 14)

#define I2C_SDA 15   // Safe to use
#define I2C_SCL 14   // Safe to use (was 2)

// ----- NEW MPU Logic -----
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
// -------------------------

// ----- Sensor & Server Objects -----
DHT dht(DHT_PIN, DHT11);
// Adafruit_MPU6050 mpu; // <-- REMOVED
AsyncWebServer server(80);

// ----- State Flag for MPU6050 -----
// bool mpuAvailable = false; // <-- REMOVED

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
  <p style="font-size: 12px; color: #888;">(Displaying raw sensor values)</p> <!-- Clarification added -->
  <div class="container">
    <div class="sensorBox"><span class="label">Accel X</span><br><span id="accelX">--</span></div>
    <div class="sensorBox"><span class="label">Accel Y</span><br><span id="accelY">--</span></div>
    <div class="sensorBox"><span class="label">Accel Z</span><br><span id="accelZ">--</span></div>
  </div>
  <div class="container">
    <div class="sensorBox"><span class="label">Gyro X</span><br><span id="gyroX">--</span></div>
    <div class="sensorBox"><span class="label">Gyro Y</span><br><span id="gyroY">--</span></div>
    <div class="sensorBox"><span class="label">Gyro Z</span><br><span id="gyroZ">--</span></div>
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

// --- NEW mpu_read() function from your other code ---
void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
// ----------------------------------------------------

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); 

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  dht.begin();

  // --- REPLACED MPU6050 INITIALIZATION ---
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Wrote to IMU");
  // ---------------------------------------
  
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
    // These (float) are kept to maintain compatibility with the JSON string
    float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
    float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;

    // --- REPLACED MPU6050 READING LOGIC ---
    mpu_read(); // This populates the global AcX, AcY... GyZ variables

    // Assign raw int16_t values to the existing float variables
    // This maintains compatibility with the JSON string and webpage
    accelX = AcX;
    accelY = AcY;
    accelZ = AcZ;
    gyroX = GyX;
    gyroY = GyY;
    gyroZ = GyZ;
    // --------------------------------------

    if (isnan(temp)) temp = 0.0;
    if (isnan(humidity)) humidity = 0.0;
    
    // This JSON structure is UNCHANGED and still works
    String json = "{";
    json += "\"distance\":" + String(distance) + ",";
    json += "\"temperature\":" + String(temp, 1) + ",";
    json += "\"humidity\":" + String(humidity, 1) + ",";
    json += "\"accelX\":" + String(accelX, 0) + ","; // Changed to 0 decimals
    json += "\"accelY\":" + String(accelY, 0) + ","; // Changed to 0 decimals
    json += "\"accelZ\":" + String(accelZ, 0) + ","; // Changed to 0 decimals
    json += "\"gyroX\":" + String(gyroX, 0) + ",";  // Changed to 0 decimals
    json += "\"gyroY\":" + String(gyroY, 0) + ",";  // Changed to 0 decimals
    json += "\"gyroZ\":" + String(gyroZ, 0);         // Changed to 0 decimals
    json += "}";
    
    request->send(200, "application/json", json);
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Nothing to do here
}
