/*
*******************************************************************************
* Copyright (c) 2023 by M5Stack
*                  Equipped with M5CoreS3 sample source code
*                          配套  M5CoreS3 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/CoreS3
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/CoreS3
*
* Describe: BMI270 example.  IMU
* Date: 2023/5/5
*******************************************************************************
*/

#include "bmi2auxClass.h"
#include <M5Unified.h>

bmi2auxClass bmi2aux;

auto &lcd = M5.Display; // 同一インスタンスを指し示す方法

void setup() {
    auto cfg = M5.config();

//#if defined ( ARDUINO )
//    cfg.serial_baudrate = 115200;   // default=115200. if "Serial" is not needed, set it to 0.
//#endif

    M5.begin(cfg);
    lcd.setTextSize(1);        // Set text size. 设置文字大小
    lcd.setCursor(0, 0);       // Set the cursor. 设置光标位置
    delay(200);                   // Delay 200ms.  延迟200ms

//    Wire1.begin(M5.In_I2C.getSDA(), M5.In_I2C.getSCL()); //GNSS or IMU pro

    bmi2aux.Init();                // Init IMU. 初始化IMU
}

void loop() {
    M5.update();
    bmi2aux.Update();  // Update data from IMU. 更新IMU数据

    // gyroscope output related.  陀螺仪输出相关
    lcd.setCursor(0, 0); // x=30 y=70
    lcd.printf("gyroX   gyroY   gyroZ");  // Screen printingformatted string.
    lcd.setCursor(0, 10); // y=92
//    lcd.fillRect(20, 22, 290, 20, BLACK);
    lcd.printf("%6.2f %6.2f %6.2f",
                     bmi2aux.gyro_data.x, bmi2aux.gyro_data.y,
                     bmi2aux.gyro_data.z);

    // Accelerometer output is related
    lcd.setCursor(0, 20); // y=120
    lcd.printf("accX    accY    accZ");
    lcd.setCursor(0, 30); // y=142
//    lcd.fillRect(20, 68, 290, 20, BLACK);
    lcd.printf("%6.2f %6.2f %6.2f",
                     bmi2aux.accel_data.x, bmi2aux.accel_data.y,
                     bmi2aux.accel_data.z);

    // mag Raw
    lcd.setCursor(0, 40); // y=170
    lcd.printf("Raw mX Raw mY Raw mZ");
    lcd.setCursor(0, 50); // y=192
//    lcd.fillRect(20, 114, 290, 20, BLACK);
    lcd.printf("%6.2f %6.2f %6.2f",
                     bmi2aux.mag_data.raw.x, bmi2aux.mag_data.raw.y,
                     bmi2aux.mag_data.raw.z);

    // mag offset compensated
    lcd.setCursor(0, 60); //
    lcd.printf("magX   magY   magZ");
    lcd.setCursor(0, 70); //
//    lcd.fillRect(20, 158, 290, 20, BLACK);
    lcd.printf("%6.2f %6.2f %6.2f",
                     bmi2aux.mag_data.x, bmi2aux.mag_data.y,
                     bmi2aux.mag_data.z);

    // Atan2 X,Y
    lcd.setCursor(0, 80); //
    lcd.printf("Heading: %5.1f deg",
                     bmi2aux.CalcuDir());

    M5.Log.printf("%6.2f %6.2f %6.2f\r\n",
                     bmi2aux.mag_data.x, bmi2aux.mag_data.y,
                     bmi2aux.mag_data.z);


    lcd.setCursor(20, 110);
    lcd.printf("CAL ");

    if(M5.BtnA.wasPressed())
    {
        lcd.clear();        // 黒で塗り潰し
        lcd.setCursor(0, 0);
        lcd.print("begin calibration in 3 seconds");
        delay(3000);
        lcd.setCursor(0, 10);
        lcd.print("Flip + rotate core calibration");
      //bmm150.bmm150_calibrate(10000);
        bmi2aux.MagCalibration();
        delay(100);

      //bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
        lcd.clear();        // 黒で塗り潰し
    }
    delay(20);
}