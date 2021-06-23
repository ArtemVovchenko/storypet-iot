#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "HTTPClient.h"
#include "Arduino_JSON.h"
#include "WiFi.h"

#define BUFFER_SIZE 100
#define LED 2

class Vector {
  private:
    double components[3];
  public:
    Vector();
    Vector(double, double, double);
    Vector(Vector, Vector);
    double X(void);
    double Y(void);
    double Z(void);
    void mul(double coef);
    double module();
};

Vector::Vector() {
  this->components[0] = 0.0;
  this->components[1] = 0.0;
  this->components[2] = 0.0;
};

Vector::Vector(double x, double y, double z) {
  this->components[0] = x;
  this->components[1] = y;
  this->components[2] = z;
};

Vector::Vector(Vector startPoint, Vector endPoint) {
  this->components[0] = endPoint.X() - startPoint.X();
  this->components[1] = endPoint.Y() - startPoint.Y();
  this->components[2] = endPoint.Z() - startPoint.Z();
};

double Vector::X() {
  return this->components[0];
};

double Vector::Y() {
  return this->components[1];
};

double Vector::Z() {
  return this->components[2];
};

void Vector::mul(double coef) {
  this->components[0] *= coef;
  this->components[1] *= coef;
  this->components[2] *= coef;
};

double Vector::module() {
  return sqrt((this->components[0] * this->components[0]) + (this->components[1] * this->components[1]) + (this->components[2] * this->components[2]));
};


// Device 
MPU6050 mpu;

// Device Constants
double G = 9.80665;
int accelMax = 4096;

//Time
unsigned long lastRequestTime = 0;
unsigned long lastMeasureTime = 0;

//Start Values
Vector prevVelocityVector, prevDistanceVector;
double distanceSum = 0.0, velocityMean = 0.0;

// Server endpoints & vars
const String authenticationUrl = "https://storypet.herokuapp.com/api/session/iot/login";
const String dataSendingUrl = "https://storypet.herokuapp.com/api/session/iot/data";
String accessSecret = "8e8c0ea53df24139fcbe51d345e3ab7d67416133c7262cd51af4506081b30746";
String accessToken;

//WiFi credentials
const char *ssid = "USER-PC_Network";
const char *password = "ak1u-uu1d-csnc";
WiFiServer server(80);


double AccToCI(int accel) {
  return (accel / accelMax) * G / 1000;
}

void configureWiFiConnection() {
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  server.begin();
}

void athenticate() {
  digitalWrite(LED, HIGH);
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    Serial.println("\nIoT authentication...");
    http.begin(authenticationUrl);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(String(String("{\"access_secret\": \"") + accessSecret + String("\"}")));

    if(httpResponseCode == 200) {
      JSONVar jsonData = JSON.parse(http.getString());
      if (JSON.typeof(jsonData) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      accessToken = jsonData["access"];
    }

    Serial.println("HTTP Response code: ");
    Serial.println(httpResponseCode);
      
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  lastRequestTime = millis();
  delay(100);
  digitalWrite(LED, LOW);
}

void sendActivityData() {
    digitalWrite(LED, HIGH);
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    Serial.println("\nSending Activity Data...");
    http.begin(dataSendingUrl);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + accessToken);
    int httpResponseCode = http.POST(String(String("{\"distance\": ") + String(distanceSum) + ", \"mean_speed\": " + String(velocityMean) + String(" }")));

    if(httpResponseCode == 401) {
      http.end();
      athenticate();
      digitalWrite(LED, LOW);
      return;
    }

    Serial.println("HTTP Response code: ");
    Serial.println(httpResponseCode);
      
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  lastRequestTime = millis();
  delay(100);
  digitalWrite(LED, LOW);
}

void setup()
{
  pinMode(LED, OUTPUT);
  Wire.begin();
  Serial.begin(115200);
  configureWiFiConnection();
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  athenticate();
  delay(100);
}


void loop()
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  unsigned long long current = millis();
  double timeInterval = (current - lastMeasureTime) / 1000;
  lastMeasureTime = current;
  
  Vector currentAccelerationVector(AccToCI(ax), AccToCI(ay), AccToCI(az));
  Vector currentVelocityVector(
    prevVelocityVector.X() + currentAccelerationVector.X() * timeInterval,
    prevVelocityVector.Y() + currentAccelerationVector.Y() * timeInterval,
    prevVelocityVector.Z() + currentAccelerationVector.Z() * timeInterval
  );
  Vector currentDistanceVector(
    prevDistanceVector.X() + currentVelocityVector.X() * timeInterval + (currentAccelerationVector.X() * timeInterval) / 2,
    prevDistanceVector.Y() + currentVelocityVector.Y() * timeInterval + (currentAccelerationVector.Y() * timeInterval) / 2,
    prevDistanceVector.Z() + currentVelocityVector.Z() * timeInterval + (currentAccelerationVector.Z() * timeInterval) / 2
  );

  Vector pointsDistanceVector(prevDistanceVector, currentDistanceVector);

  prevVelocityVector = currentVelocityVector;
  prevDistanceVector = currentDistanceVector;

  distanceSum += prevVelocityVector.module();
  velocityMean = (currentVelocityVector.module() + velocityMean) / 2;

  Serial.print("Distance: "); Serial.print(distanceSum); Serial.print("\n");
  Serial.print("Speed: "); Serial.print(velocityMean); Serial.print("\n");

  if ((current - lastRequestTime) / 6000 > 2) {
    
    lastRequestTime = current;
    sendActivityData();
    distanceSum = 0.0;
    velocityMean = 0.0;
  }
  
  delay(5000);
}
