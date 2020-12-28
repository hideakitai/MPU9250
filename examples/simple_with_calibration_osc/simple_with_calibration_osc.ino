#include "MPU9250.h"
#include "ArduinoOSC.h"

MPU9250 mpu;

// WiFi stuff
const char* ssid = "your-ssid";
const char* pwd = "your-password";
const IPAddress ip(192, 168, 0, 201);
const IPAddress gateway(192, 168, 0, 1);
const IPAddress subnet(255, 255, 255, 0);

// for ArduinoOSC
const char* host = "192.168.0.8";
const int publish_port = 54445;

struct Quat {
    float x;
    float y;
    float z;
    float w;
} quat;
struct Rpy {
    float r;
    float p;
    float y;
} rpy;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    // WiFi stuff (no timeout setting for WiFi)
    WiFi.begin(ssid, pwd);
    WiFi.config(ip, gateway, subnet);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.print("WiFi connected, IP = ");
    Serial.println(WiFi.localIP());

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    mpu.printCalibration();
    mpu.verbose(false);

    OscWiFi.publish(host, publish_port, "/quat", quat.x, quat.y, quat.z, quat.w);
    OscWiFi.publish(host, publish_port, "/rpy", rpy.r, rpy.p, rpy.y);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            mpu.printRollPitchYaw();
            prev_ms = millis();
        }
        quat.x = mpu.getQuaternionX();
        quat.y = mpu.getQuaternionY();
        quat.z = mpu.getQuaternionZ();
        quat.w = mpu.getQuaternionW();
        rpy.r = mpu.getRoll();
        rpy.p = mpu.getPitch();
        rpy.y = mpu.getYaw();
    }
    OscWiFi.update();
}
