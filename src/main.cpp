#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN     38
#define NUM_LEDS    1
#define BRIGHTNESS  64
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define BOOT_PIN 0      // Boot键通常连接到GPIO0
CRGB leds[NUM_LEDS];

MPU6050 mpu;
Servo servoX; // X轴舵机
Servo servoY; // Y轴舵机

const int TABLESIZE = 2001; // 从500到2500共有2001个整数值
float mappingTable[TABLESIZE];

bool imuMode = true;

// ESP32-S3 NodeMCU的SCL和SDA引脚
const int sclPin = 19; // IO5
const int sdaPin = 18; // IO4

// 舵机控制引脚
const int servoXPin = 4; // 舵机X控制引脚
const int servoYPin = 5; // 舵机Y控制引脚

float angleX = 0.0F;
float angleY = 0.0F;
int XpulseWidth = 0; // 映射角度到脉冲宽度
int YpulseWidth = 0; // 映射角度到脉冲宽度

void createMappingTable() 
{
  for (int i = 0; i < TABLESIZE; i++) 
  {
    // 手动计算映射值
    float value = (float)(i + 500 - 500) * (135.0 + 135.0) / (2500 - 500) - 135.0;
    mappingTable[i] = value;
  }
}

void setup() 
{
  pinMode(BOOT_PIN, INPUT_PULLUP); // 设置Boot键为输入模式，并启用内部上拉电阻

  createMappingTable();

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  Serial.begin(115200);
  Wire.begin(sdaPin, sclPin);
  mpu.initialize();
  if (!mpu.testConnection()) 
  {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  servoX.attach(servoXPin, 500, 2500); // 将舵机X附加到控制引脚
  servoY.attach(servoYPin, 500, 2500); // 将舵机Y附加到控制引脚

  leds[0] = CRGB::White;
  FastLED.show();
}

void loop() 
{
  int bootState = digitalRead(BOOT_PIN); // 读取Boot键的状态
  if (bootState == LOW)
  {
    imuMode = !imuMode;
  }

  if (imuMode)
  {
    leds[0] = CRGB::Green;
    FastLED.show();

    // 读取加速度和陀螺仪值
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 计算姿态
    angleX = atan2(ay, az) * 180 / PI;
    angleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

    // 调整angleX的值域到[-180,180]
    if (angleX > 180) 
    {
      angleX -= 360;
    }

    // 调整angleY的值域到[-180,180]
    if (angleY > 180) 
    {
      angleY -= 360;
    }

    angleX = -angleX;
    angleY = -angleY;
  }
  else
  {
    leds[0] = CRGB::Yellow;
    FastLED.show();
    // 读取串口数据并解析
    if (Serial.available() > 0) 
    {
      String message = Serial.readStringUntil('!');
      if (sscanf(message.c_str(), "$(%f, %f)", &angleX, &angleY) == 2) 
      {
        // 成功解析到两个浮点数
        Serial.print("received angleX: ");
        Serial.println(angleX);
        Serial.print("received angleY: ");
        Serial.println(angleY);
      } 
      else 
      {
        Serial.println("Failed to parse message");
      }
    }
  }

  // 控制舵机转动到对应角度
  for (int i = 0; i < TABLESIZE; i++) 
  {
    if (angleX >= mappingTable[i] && angleX < mappingTable[i + 1]) 
    {
      XpulseWidth = i + 500;
      break;
    }
  }
  for (int i = 0; i < TABLESIZE; i++) 
  {
    if (angleY >= mappingTable[i] && angleY < mappingTable[i + 1]) 
    {
      YpulseWidth = i + 500;
      break;
    }
  }
  servoX.writeMicroseconds(XpulseWidth);
  servoY.writeMicroseconds(YpulseWidth);

  // 发送姿态数据
  Serial.print("imu h pose: ");
  Serial.print(angleX);
  Serial.print("|");
  Serial.print("\timu v pose: ");
  Serial.print(angleY);
  Serial.print("\tservo h pose: ");
  Serial.print(angleX);
  Serial.print("\ttservo v pose: ");
  Serial.print(angleY);
  Serial.println("|");

  delay(150);
}