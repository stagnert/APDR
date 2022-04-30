#include <Wire.h> //Needed for I2C
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>

#include <Adafruit_BNO08x.h>
#include <Adafruit_I2CDevice.h>

#define I2C_ADDR_IMU 0x4A
#define I2C_ADDR_GPS 0x42
#define BNO08X_RESET -1
#define LOCK 13
#define LID_OPEN 17
#define BUMP 16
#define BATT 34

struct web_data {
    double latitude = 0;
    double longitude = 0;
    double accel_x = 0;
    double accel_y = 0;
    double accel_z = 0;
    double gyro_x = 0;
    double gyro_y = 0;
    double gyro_z = 0;
    double quat_r = 0;
    double quat_i = 0;
    double quat_j = 0;
    double quat_k = 0;
    int batt = 0;
    bool bump = false;
    bool open = false;
    bool unlock = false;
};

Adafruit_BNO08x bno08x(BNO08X_RESET);           // The IMU sensor object
sh2_SensorValue_t sensorValue;                  // The GPS sensor object

SFE_UBLOX_GNSS myGPS;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool imu_conn;
bool gps_conn;

struct web_data dt;
size_t pi_buff[5];

// Update the battery voltage on the robot.
void checkBattV() {
    dt.batt = analogRead(BATT);
}

// Check for new data on the NEO M9N GPS Module. Updates latitude and longitude variables.
void checkGPSData() {
    myGPS.begin();

    myGPS.checkUblox(); // See if new data is available. Process bytes as they come in.

    if (nmea.isValid() == true) {
        long latitude_mdeg = nmea.getLatitude();
        long longitude_mdeg = nmea.getLongitude();

        dt.latitude = latitude_mdeg / 1000000.;
        dt.longitude = longitude_mdeg / 1000000.;

        nmea.clear(); // Clear the MicroNMEA storage to make sure we are getting fresh data
    }
}

// Check for new data on the BNO085 IMU module. Updates the Gyrocope and Accelerometer variables.
void checkIMUData_accel()
{
    if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
        //Serial.println("Could not enable accelerometer");
    }

    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;
    }
    else {
        // Save new values of IMU Accelerometer data if there was a change from previous data
        switch (sensorValue.sensorId) {

        case SH2_ACCELEROMETER:
            dt.accel_x = sensorValue.un.accelerometer.x;
            dt.accel_y = sensorValue.un.accelerometer.y;
            dt.accel_z = sensorValue.un.accelerometer.z;
            break;
        }
    }
}

void checkIMUData_gyro()
{
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
        //Serial.println("Could not enable accelerometer");
    }

    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;
    }
    else {
        // Save new values of IMU Gyroscope data if there was a change from previous data
        switch (sensorValue.sensorId) {
        
        case SH2_GYROSCOPE_CALIBRATED:
            dt.gyro_x = sensorValue.un.gyroscope.x;
            dt.gyro_y = sensorValue.un.gyroscope.y;
            dt.gyro_z = sensorValue.un.gyroscope.z;
            break;
        }
    }
}

void checkIMUData_quat()
{
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV)) {
        //Serial.println("Could not enable accelerometer");
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
        // Save new values of IMU Gyroscope data if there was a change from previous data
        switch (sensorValue.sensorId) {
        
        case SH2_GYROSCOPE_CALIBRATED:
            dt.quat_r = sensorValue.un.arvrStabilizedRV.real;
            dt.quat_i = sensorValue.un.arvrStabilizedRV.i;
            dt.quat_j = sensorValue.un.arvrStabilizedRV.j;
            dt.quat_k = sensorValue.un.arvrStabilizedRV.k;
            break;
        }
    }
}

// Unlock the lock box
void unlock_box() {
    delay(1000);
    digitalWrite(LOCK, 1);
    delay(200);
    digitalWrite(LOCK, 0);
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Wire.begin();
    if (!myGPS.begin()) {
        //Serial.println(F("GPS module not detected"));
        gps_conn = false;
    }
    else {
        myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); // Set the I2C port to output both NMEA and UBX messages
        myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR

        myGPS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL); // Make sure the library is passing all NMEA messages to processNMEA
        gps_conn = true;
    }

    if (!bno08x.begin_I2C()) {
        //Serial.println("Failed to find BNO085 chip");
        imu_conn = false;
    }
    else {
        imu_conn = true;
    }
    //attachInterrupt(BUMP, isr, HIGH);
    pinMode(BUMP, INPUT_PULLUP);
    pinMode(LOCK, OUTPUT);
    pinMode(LID_OPEN, INPUT_PULLUP);

}

void loop() {
    String pi_data = "";
    int pii_data = 5;
    bool lid;

    /* 
    Get data from Raspberry Pi about what to update. First, check if there was a collision and tell the Pi.
    Next, Pi will send a single number and that number will determine what action to take:
        0 = GPS
        1 = IMU Accelerometer
        2 = IMU Gyroscope
        3 = IMU Quaternion
        4 = Unlock
        5 = Battery Voltage
        Anything else = check for open lid
    */

    while (1) {
        if (Serial.available() > 0) {
            if (digitalRead(BUMP)) {
                dt.bump = true;
                Serial.print("BUMP ");
                Serial.println(dt.bump);
            }

            pi_data = Serial.readStringUntil('\n');
            pii_data = pi_data.toInt();

            if (gps_conn || imu_conn) {

                switch(pii_data) {
                    // Print GPS data
                    case (0):
                        if (!gps_conn) {
                            Serial.println("Error - GPS not connected");
                            break;
                        }
                        checkGPSData();
                        Serial.print("GPS_G,");
                        Serial.print(dt.latitude);
                        Serial.print(",");
                        Serial.println(dt.longitude);
                        break;
                    
                    // Print IMU accelerometer data
                    case (1):
                        if (!imu_conn) {
                            Serial.println("Error - IMU not connected");
                            break;
                        }
                        checkIMUData_accel();
                        Serial.print("IMU_A,");
                        Serial.print(dt.accel_x);
                        Serial.print(",");
                        Serial.print(dt.accel_y);
                        Serial.print(",");
                        Serial.println(dt.accel_z);
                        break;

                    // Print IMU gyroscope data
                    case (2):
                        if (!imu_conn) {
                            Serial.println("Error - IMU not connected");
                            break;
                        }
                        checkIMUData_gyro();
                        Serial.print("IMU_G,");
                        Serial.print(dt.gyro_x);
                        Serial.print(",");
                        Serial.print(dt.gyro_y);
                        Serial.print(",");
                        Serial.println(dt.gyro_z);
                        break;

                    // Print IMU quaternion data
                    case (3):
                        if (!imu_conn) {
                            Serial.println("Error - IMU not connected");
                            break;
                        }
                        checkIMUData_quat();
                        Serial.print("IMU_Q,");
                        Serial.print(dt.quat_r);
                        Serial.print(",");
                        Serial.print(dt.quat_i);
                        Serial.print(",");
                        Serial.print(dt.quat_j);
                        Serial.print(",");
                        Serial.println(dt.quat_k);
                        break;

                    // Unlock Box
                    case (4):
                        unlock_box();
                        break;

                    // Print battery voltage
                    case (5):
                        //checkBattV();
                        Serial.print("BATT,");
                        Serial.println(analogRead(BATT));
                        break;

                    // No indication, check for lid open and/or bump
                    default:
                        if (digitalRead(LID_OPEN)) { 
                            Serial.println("Lid is OPEN");
                        }
                        break;
                }
            }
        
            else {
                Serial.println("Error - No devices connected");
            }
        }
        if (gps_conn) {
            checkGPSData();
        }

        if (imu_conn) {
            checkIMUData_accel();
            checkIMUData_gyro();
            checkIMUData_quat();
        }
        delay(50); // Allow the I2C bus to update
    }
}

void SFE_UBLOX_GNSS::processNMEA(char incoming) {
    // Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
    // for sentence cracking
    nmea.process(incoming);
}
