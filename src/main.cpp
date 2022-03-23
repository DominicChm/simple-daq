#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include "print_mpu_settings.h"
#include "DebouncedButton.h"
#include <SdFat.h>
#include "util.h"

#define PIN_DAQ_TOGGLE 11
#define PIN_MODE_SWITCH 8

#define PIN_SHOCK_BL A14
#define PIN_SHOCK_FL A15
#define PIN_SHOCK_BR A16
#define PIN_SHOCK_FR A17

#define LCD_REFRESH_INTERVAL 250
#define SENSOR_REFRESH_INTERVAL 33

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))


struct {
    uint16_t BL;
    uint16_t FL;
    uint16_t BR;
    uint16_t FR;
} shock_data;

int NUM_SCREENS = 3;
enum LCDScreen {
    LOG_STATUS,
    ACCEL_MEASUREMENTS,
    SHOCKS,

    SD_ERROR,
    FILE_ERROR,
};

Adafruit_MPU6050 mpu;
LiquidCrystal lcd(32, 31, 30, 29, 28, 27);

sensors_event_t a, g, temp;
LCDScreen lcd_screen;
SdFs sd;
FsFile file;
unsigned long t_start = 0;
char filename[32];

bool is_logging;

DebouncedButton mode_switch(PIN_MODE_SWITCH, 500);

void setup() {
    Serial.begin(115200);
    pinMode(PIN_DAQ_TOGGLE, INPUT_PULLUP);
    pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);

    pinMode(PIN_SHOCK_BL, INPUT);
    pinMode(PIN_SHOCK_FL, INPUT);
    pinMode(PIN_SHOCK_BR, INPUT);
    pinMode(PIN_SHOCK_FR, INPUT);

    lcd.begin(16, 2);
    lcd.print("DAQ Starting...");

    Serial.println("DAQ Starting");

    // Try to initialize!
    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire2, 0)) {
        Serial.println("Failed to find MPU6050 chip");

        lcd.clear();
        lcd.print("MPU6050 Failure");
        lcd.setCursor(0, 1);
        lcd.print("Check wiring!");

        while (true);
    }

    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    print_mput_settings(mpu);

    while (!digitalRead(PIN_DAQ_TOGGLE)) {
        lcd.clear();
        lcd.print("Turn off DAQ");
        lcd.setCursor(0, 1);
        lcd.print("toggle!!!");
        delay(500);
        lcd.clear();
        delay(500);
    }

    delay(100);
    Serial.println("Setup done!");
}


void render_LCD() {
    lcd.clear();
    switch (lcd_screen) {
        case LOG_STATUS:
            if (is_logging) {
                lcd.printf("Logging to:");
                lcd.setCursor(0, 1);
                lcd.print(filename);
            } else {
                lcd.printf("Ready to log!");
            }
            break;

        case ACCEL_MEASUREMENTS:
            lcd.printf("X:%.2f Y:%.2f", a.acceleration.x, a.acceleration.y);
            lcd.setCursor(0, 1);
            lcd.printf("Z:%.2f ", a.acceleration.z);
            break;

        case SHOCKS:
            lcd.printf("FL:%d FR:%d", shock_data.FL, shock_data.FR);
            lcd.setCursor(0, 1);
            lcd.printf("BL:%d BR:%d", shock_data.BL, shock_data.BR);
            break;

        case SD_ERROR:
            lcd.printf("SD INIT ERROR!");
            break;

        case FILE_ERROR:
            lcd.printf("COULDN'T CREATE");
            lcd.setCursor(0, 1);
            lcd.printf("FILE!!!");
            break;

        default:
            lcd.printf("SCREEN NOT FOUND");
    }

}

unsigned long last_lcd_refresh = 0;
unsigned long last_sensor_refresh = 0;

void init_log() {

    if (!sd.begin(BUILTIN_SDCARD)) {
        lcd_screen = SD_ERROR;
        if (sd.sdErrorCode()) {
            if (sd.sdErrorCode() == SD_CARD_ERROR_ACMD41) {
                Serial.println("Try power cycling the SD card.");
            }
            sd.printSdError(&Serial);
        }
        return;
    }
    select_next_filename(filename, &sd);
    if (!file.open(filename, O_RDWR | O_CREAT)) {
        lcd_screen = FILE_ERROR;
        return;
    }

    file.printf(
            "Time (s), Xaccel (m/s^2), Yaccel (m/s^2), Zaccel (m/s^2), Xgyro (rad/s), Ygyro (rad/s), Zgyro (rad/s), FLsus, FRsus, BLsus, BRsus\n");
    t_start = millis();
    lcd_screen = LOG_STATUS;
    is_logging = true;
}

void loop() {
    //INIT timers
    if (last_lcd_refresh == 0) last_lcd_refresh = millis();
    if (last_sensor_refresh == 0) last_sensor_refresh = millis();

    //Input checks
    if (!digitalRead(PIN_DAQ_TOGGLE)) { // Switch is ON
        if (!is_logging) init_log(); //State changed, init file.

    } else { // Switch is OFF
        if (is_logging) { //State changed, finish up.
            Serial.println("Stopping log!");

            file.truncate();
            file.flush();
            file.sync();
            file.close();

            lcd_screen = LOG_STATUS;
            is_logging = false;
        }
    }


    if (mode_switch.isTriggered()) {
        int screen = static_cast<int>(lcd_screen);
        if (++screen >= NUM_SCREENS) screen = 0;
        lcd_screen = static_cast<LCDScreen>(screen);
    }

    if (millis() - last_sensor_refresh > SENSOR_REFRESH_INTERVAL) {
        last_sensor_refresh += SENSOR_REFRESH_INTERVAL;
        /* Get new sensor events with the readings */
//        Serial.println("GETTING DATA!");
        shock_data.BL = analogRead(PIN_SHOCK_BL);
        shock_data.FL = analogRead(PIN_SHOCK_FL);
        shock_data.BR = analogRead(PIN_SHOCK_BR);
        shock_data.FR = analogRead(PIN_SHOCK_FR);


        mpu.getEvent(&a, &g, &temp);
        //file.print("TEST TEST TEST");
        file.printf("%f, %f, %f, %f, %f, %f, %f, %d, %d, %d, %d\n", (millis() - t_start) / 1000.f,
                    a.acceleration.x, a.acceleration.y, a.acceleration.z,
                    g.gyro.x, g.gyro.y, g.gyro.z,
                    shock_data.FL, shock_data.FR, shock_data.BL, shock_data.BR);

//
//        /* Print out the values */
//        Serial.print("Acceleration X: ");
//        Serial.print(a.acceleration.x);
//        Serial.print(", Y: ");
//        Serial.print(a.acceleration.y);
//        Serial.print(", Z: ");
//        Serial.print(a.acceleration.z);
//        Serial.println(" m/s^2");
//
//        Serial.print("Rotation X: ");
//        Serial.print(g.gyro.x);
//        Serial.print(", Y: ");
//        Serial.print(g.gyro.y);
//        Serial.print(", Z: ");
//        Serial.print(g.gyro.z);
//        Serial.println(" rad/s");
//
//        Serial.print("Temperature: ");
//        Serial.print(temp.temperature);
//        Serial.println(" degC");
//
//        Serial.println("");
    }

    if (millis() - last_lcd_refresh > LCD_REFRESH_INTERVAL) {
        last_lcd_refresh = millis();

        render_LCD();
    }


}