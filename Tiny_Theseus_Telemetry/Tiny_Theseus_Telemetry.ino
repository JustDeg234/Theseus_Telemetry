#include <Wire.h>
#include "bmp390.h"
#include "mpu6050.h"

#define TRANSMIT_INTERVAL_MS 100  // 10 Hz

static BMP390_Data_t  bmp_data;
static MPU6050_Data_t imu_data;
static uint32_t       last_tx = 0;

void setup() {
    Serial.begin(115200);   // debug
    Serial1.begin(9600);    // LoRa
    while (!Serial);

    Wire.begin();

    Serial.println("Theseus telemetry starting...");

    if (!BMP390_Init()) {
        Serial.println("ERROR: BMP390 init failed");
        while (1);
    }
    Serial.println("BMP390 OK");

    if (!MPU6050_Init()) {
        Serial.println("ERROR: MPU6050 init failed");
        while (1);
    }
    Serial.println("MPU6050 OK");

    Serial.println("alt_m,press_pa,temp_c,ax,ay,az,gx,gy,gz");
}

void loop() {
    uint32_t now = millis();

    if (now - last_tx >= TRANSMIT_INTERVAL_MS) {
        last_tx = now;

        bool bmp_ok = BMP390_Read(&bmp_data);
        bool imu_ok = MPU6050_Read(&imu_data);

        if (bmp_ok && imu_ok) {
            // debug to laptop
            Serial.print(bmp_data.altitude_m);    Serial.print(",");
            Serial.print(bmp_data.pressure_pa);   Serial.print(",");
            Serial.print(bmp_data.temperature_c); Serial.print(",");
            Serial.print(imu_data.ax);             Serial.print(",");
            Serial.print(imu_data.ay);             Serial.print(",");
            Serial.print(imu_data.az);             Serial.print(",");
            Serial.print(imu_data.gx);             Serial.print(",");
            Serial.print(imu_data.gy);             Serial.print(",");
            Serial.println(imu_data.gz);

            // LoRa packet to ground
            char packet[128];
            snprintf(packet, sizeof(packet),
                "THS,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                bmp_data.altitude_m,
                bmp_data.pressure_pa,
                bmp_data.temperature_c,
                imu_data.ax, imu_data.ay, imu_data.az,
                imu_data.gx, imu_data.gy, imu_data.gz);

            Serial1.print(packet);
        }
    }
}