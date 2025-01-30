#include <M5Unified.h>
#include "bmi2auxClass.h"
#include "utility/myMahonyAHRS.h"
#include "ehou_roll.h"
#include "itako.h"

bmi2auxClass bmi2aux;

auto &lcd = M5.Display; // 同一インスタンスを指し示す方法
static M5Canvas compass(&lcd);     // オフスクリーン描画用バッファ
static M5Canvas base(&compass);    // オフスクリーン描画用バッファ

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;

void compassplot(float a);
void drawAirplane(); //プロトタイプ宣言

void setup() {
    auto cfg = M5.config();
#if defined ( ARDUINO )
    cfg.serial_baudrate = 115200;   // default=115200. if "Serial" is not needed, set it to 0.
#endif
    M5.begin(cfg);
    delay(200);                   // Delay 200ms.  延迟200ms

    bmi2aux.Init();                // Init IMU. 初始化IMU
    myKp=8.0f; myKi=0.0f;

// 必要に応じてカラーモードを設定します。（初期値は16）
// 16の方がSPI通信量が少なく高速に動作しますが、赤と青の諧調が5bitになります。
    lcd.setColorDepth(16);
// clearまたはfillScreenで画面全体を塗り潰します。
    lcd.clear();        // 黒で塗り潰し
// スプライト（オフスクリーン）への描画も同様の描画関数が使えます。
// 最初にスプライトの色深度をsetColorDepthで指定します。（省略した場合は16として扱われます。）
    compass.setColorDepth(16);
    base.setColorDepth(16);
// createSpriteで幅と高さを指定してメモリを確保します。
// 消費するメモリは色深度と面積に比例します。大きすぎるとメモリ確保に失敗しますので注意してください。
    compass.createSprite(112,112);
    base.createSprite(120,120);//120->

// base airplane
    drawAirplane();
  
// 作成したスプライトはpushSpriteで任意の座標に出力できます。
}

void compassplot(float a) {
    int ang;
    compass.setTextDatum(middle_center);
    compass.setFont(&fonts::Font2);
    compass.fillScreen(0); // fill black

    for (ang=0 ; ang<36 ; ang++) {
        //180/2->112/2, 80->48
        compass.drawLine(56+48*sin((a+ang*10)*DEG_TO_RAD),56-48*cos((a+ang*10)*DEG_TO_RAD),
  56+56*sin((a+ang*10)*DEG_TO_RAD), 56-56*cos((a+ang*10)*DEG_TO_RAD), TFT_WHITE); //0
        compass.setTextSize(1.2);
        compass.setFont(&fonts::Font0);
        if (ang==0) { //72->48
            compass.drawString("N", 56+48*sin((a+  0)*DEG_TO_RAD), 56-48*cos((a+  0)*DEG_TO_RAD)); //0
        } else if (ang==9) {
            compass.drawString("E", 56+48*sin((a+ 90)*DEG_TO_RAD), 56-48*cos((a+ 90)*DEG_TO_RAD)); //90
        } else if (ang==18) {
            compass.drawString("S", 56+48*sin((a+180)*DEG_TO_RAD), 56-48*cos((a+180)*DEG_TO_RAD)); //180
        } else if (ang==27) {
            compass.drawString("W", 56+48*sin((a+270)*DEG_TO_RAD), 56-48*cos((a+270)*DEG_TO_RAD)); //270
        } else if ((ang%3)==0) {
            compass.setTextSize(1);
            compass.setFont(&fonts::Font0);
            compass.drawNumber(ang, 56+48*sin((a+ang*10)*DEG_TO_RAD), 56-48*cos((a+ang*10)*DEG_TO_RAD));
        }
    }
    compass.pushImage(32+48*sin((a+255)*DEG_TO_RAD), 32-48*cos((a+255)*DEG_TO_RAD), 48, 48, ehou, 0); //0,5

    //56*.707=40. 8*.7.7=6
    compass.fillTriangle(56+40, 56-40, 96+5, 16-7, 96+7, 16-5, TFT_WHITE);// 45
    compass.fillTriangle(56+40, 56+40, 96+5, 96+7, 96+7, 96+5, TFT_WHITE);//135
    compass.fillTriangle(56-40, 56+40, 16-5, 96+7, 16-7, 96+5, TFT_WHITE);//225
    compass.fillTriangle(56-40, 56-40, 16-5, 16-7, 16-7, 16-5, TFT_WHITE);//315
  
// 作成したスプライトはpushSpriteで任意の座標に出力できます。
    base.pushSprite(16,16,0); // 24,24
    compass.pushSprite(8,8); // 70,30
//  compass.endWrite();

}

void drawAirplane() {
// base airplane
/*
  base.drawLine( 28, 19, 28,  7, TFT_WHITE); //front L
  base.drawLine(  1, 33, 28, 19, TFT_WHITE); //wing L
  base.drawLine(  2, 41,  1, 33, TFT_WHITE); //wing L
  base.drawLine( 28, 34,  2, 41, TFT_WHITE); //wing L
  base.drawLine( 28, 44, 28, 34, TFT_WHITE); //body L
  base.drawLine( 19, 55, 29, 51, TFT_WHITE); //
  base.drawLine( 19, 61, 19, 55, TFT_WHITE);
  base.drawLine( 31, 59, 19, 61, TFT_WHITE);
  base.drawLine( 32, 62, 31, 59, TFT_WHITE);
  base.drawLine( 33, 59, 32, 62, TFT_WHITE);
  base.drawLine( 45, 61, 33, 59, TFT_WHITE);
  base.drawLine( 45, 55, 45, 61, TFT_WHITE);
  base.drawLine( 35, 51, 45, 55, TFT_WHITE);
  base.drawLine( 36, 34, 36, 44, TFT_WHITE); //body R
  base.drawLine( 62, 41, 36, 34, TFT_WHITE); //wing R
  base.drawLine( 63, 33, 62, 41, TFT_WHITE); //wing R
  base.drawLine( 36, 19, 63, 33, TFT_WHITE); //wing R
  base.drawLine( 36,  7, 36, 19, TFT_WHITE);
  base.drawLine( 29, 51, 28, 44, TFT_WHITE);
  base.drawLine( 35, 51, 36, 44, TFT_WHITE);
  base.drawLine( 28,  7, 29,  2, TFT_WHITE);
  base.drawLine( 36,  7, 35,  2, TFT_WHITE);
  base.drawLine( 35,  2, 32,  0, TFT_WHITE); //head
  base.drawLine( 29,  2, 32,  0, TFT_WHITE); //head
*/
  base.pushImage( 0, 0, 80, 80, itako, 0); //0,5

}

void loop() {
    M5.update();
    bmi2aux.Update();  // Update data from IMU. 更新IMU数据

    float head_dir = bmi2aux.CalcuDir();

    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);//0.09
    lastUpdate = Now;

    myIMU::MahonyAHRSupdate(
                bmi2aux.gyro_data.y * DEG_TO_RAD, -bmi2aux.gyro_data.x * DEG_TO_RAD, bmi2aux.gyro_data.z * DEG_TO_RAD,
                bmi2aux.accel_data.y, -bmi2aux.accel_data.x, bmi2aux.accel_data.z,
                -bmi2aux.mag_data.x, bmi2aux.mag_data.y, -bmi2aux.mag_data.z, deltat);

    pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]); // pitch
    roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1); // roll
    yaw   = atan2(2*(q[1]*q[2] + q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);  //yaw
    yaw = -yaw-PI/2;
    if(yaw < 0)
        yaw += 2*PI;
    if(yaw > 2*PI)
        yaw -= 2*PI;

    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;

    compassplot(360.0-yaw);

    // for processing display
    M5.Log.printf("Gyro %5.2f,  %5.2f,  %5.2f  \r\n", bmi2aux.gyro_data.x, bmi2aux.gyro_data.y, bmi2aux.gyro_data.z); // to processing
    M5.Log.printf("Acc  %5.2f,  %5.2f,  %5.2f  \r\n", bmi2aux.accel_data.x, bmi2aux.accel_data.y, bmi2aux.accel_data.z); // to processing
    M5.Log.printf("MAG  %5.2f,  %5.2f,  %5.2f  \r\n", bmi2aux.mag_data.x, -bmi2aux.mag_data.y, -bmi2aux.mag_data.z); // to processing
    M5.Log.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", roll, pitch, yaw); // to processing

    lcd.startWrite();
    lcd.fillTriangle(64,    7,64-2,  0,64+2,  0, TFT_WHITE);//  0
    lcd.fillTriangle(127-7,64,127,64-2,127,64+2, TFT_WHITE);// 90
    lcd.fillTriangle(64,127-7,64-2,127,64+2,127, TFT_WHITE);//180
    lcd.fillTriangle(7,    64,  0,64-2,  0,64+2, TFT_WHITE);//270


    if(M5.BtnA.wasPressed())
    {
        lcd.clear();        // 黒で塗り潰し
        lcd.setCursor(0, 0);
        lcd.print("begin calibration in 3 seconds");
        delay(3000);
        lcd.setCursor(0, 10);
        lcd.print("Flip + rotate core calibration");
        bmi2aux.MagCalibration();
        delay(100);

        lcd.clear();        // 黒で塗り潰し
    }
}
