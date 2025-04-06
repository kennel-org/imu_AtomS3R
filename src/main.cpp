#include <M5Unified.h>
#include "bmi2auxClass.h"
#include "utility/myMahonyAHRS.h"

bmi2auxClass bmi2aux;

auto &lcd = M5.Display;
static M5Canvas compass(&lcd);     // Off-screen drawing buffer
static M5Canvas base(&compass);    // Off-screen drawing buffer

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;
static uint32_t fsec, psec;
static size_t fps = 0, frame_count = 0;

void compassplot(float a);
void drawAirplane();

void setup() {
    auto cfg = M5.config();
#if defined ( ARDUINO )
    cfg.serial_baudrate = 115200;   // default=115200. if "Serial" is not needed, set it to 0.
#endif
    M5.begin(cfg);
    delay(200);                   // Delay 200ms

    bmi2aux.Init();                // Init IMU
    myKp=8.0f; myKi=0.0f;

    // Set color mode as needed (default is 16)
    // 16 is faster with less SPI communication, but red and blue have 5-bit gradation
    lcd.setColorDepth(16);
    lcd.clear();        // Fill with black

    // Create compass display
    compass.createSprite(128, 128);
    compass.fillScreen(TFT_BLACK);
    compass.setTextColor(TFT_WHITE);
    compass.setTextSize(1);

    // Create base display elements
    base.createSprite(128, 128);
    base.fillScreen(TFT_BLACK);
    base.setTextColor(TFT_WHITE);
    base.setTextSize(1);

    // Draw compass base elements
    base.fillCircle(64, 64, 60, 0x4208);
    base.fillCircle(64, 64, 58, TFT_BLACK);
    base.drawCircle(64, 64, 59, TFT_DARKGREY);
    base.setTextDatum(middle_center);
    base.setTextColor(TFT_WHITE);
    base.drawString("N", 64, 10);
    base.drawString("E", 118, 64);
    base.drawString("S", 64, 118);
    base.drawString("W", 10, 64);
    base.drawCircle(64, 64, 3, TFT_RED);
    base.pushSprite(0, 0);

    lcd.startWrite();
    lcd.setCursor(0, 0);
    lcd.println("M5AtomS3R Compass");
    lcd.println("Press button to calibrate");
    lcd.endWrite();
}

void loop() {
    M5.update();
    
    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);
    lastUpdate = Now;

    // Read IMU data
    bmi2aux.readData();

    // Update AHRS filter
    MahonyAHRSupdate(bmi2aux.gyro_data.x, bmi2aux.gyro_data.y, bmi2aux.gyro_data.z,
                    bmi2aux.accel_data.x, bmi2aux.accel_data.y, bmi2aux.accel_data.z,
                    bmi2aux.mag_data.x, -bmi2aux.mag_data.y, -bmi2aux.mag_data.z);

    // Get Euler angles
    getEuler(&pitch, &roll, &yaw);
    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;

    // Display compass
    compassplot(360.0-yaw);

    // Log data for debugging
    M5.Log.printf("Gyro %5.2f, %5.2f, %5.2f \r\n", bmi2aux.gyro_data.x, bmi2aux.gyro_data.y, bmi2aux.gyro_data.z);
    M5.Log.printf("Acc  %5.2f, %5.2f, %5.2f \r\n", bmi2aux.accel_data.x, bmi2aux.accel_data.y, bmi2aux.accel_data.z);
    M5.Log.printf("MAG  %5.2f, %5.2f, %5.2f \r\n", bmi2aux.mag_data.x, -bmi2aux.mag_data.y, -bmi2aux.mag_data.z);
    M5.Log.printf(" %5.2f, %5.2f, %5.2f \r\n", roll, pitch, yaw);

    // Draw cardinal direction markers
    lcd.startWrite();
    lcd.fillTriangle(64,    7, 64-2,  0, 64+2,  0, TFT_WHITE); //  0°
    lcd.fillTriangle(127-7, 64, 127, 64-2, 127, 64+2, TFT_WHITE); // 90°
    lcd.fillTriangle(64, 127-7, 64-2, 127, 64+2, 127, TFT_WHITE); // 180°
    lcd.fillTriangle(7,    64,  0, 64-2,  0, 64+2, TFT_WHITE); // 270°
    lcd.endWrite();

    // Calculate FPS
    ++frame_count;
    uint32_t fsec = m5gfx::millis() / 1000;
    if (psec != fsec) {
        psec = fsec;
        fps = frame_count;
        frame_count = 0;
    }

    // Handle button press for calibration
    if(M5.BtnA.wasPressed()) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Begin calibration in 3 seconds");
        delay(3000);
        lcd.setCursor(0, 10);
        lcd.print("Flip + rotate core calibration");
        bmi2aux.MagCalibration();
        delay(100);
        lcd.clear();
    }
}

// Draw compass with heading angle
void compassplot(float a) {
    float sx = cos(DEG_TO_RAD * (a - 90));
    float sy = sin(DEG_TO_RAD * (a - 90));
    
    compass.pushImage(0, 0, 128, 128, base.getBuffer());
    
    // Draw compass needle
    compass.fillTriangle(64, 64,
                        64 + 50 * sx - 8 * sy, 64 + 50 * sy + 8 * sx,
                        64 + 50 * sx + 8 * sy, 64 + 50 * sy - 8 * sx,
                        TFT_RED);
    
    // Draw heading text
    compass.setTextDatum(middle_center);
    compass.setTextColor(TFT_WHITE, TFT_BLACK);
    compass.drawString(String(int(a)) + "°", 64, 40);
    
    // Draw airplane indicator
    drawAirplane();
    
    // Push to display
    compass.pushSprite(0, 0);
}

// Draw airplane indicator in the center
void drawAirplane() {
    // Simple airplane shape
    compass.fillRect(62, 59, 5, 10, TFT_YELLOW);  // Body
    compass.fillRect(55, 64, 19, 2, TFT_YELLOW);  // Wings
}