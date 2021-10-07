/*
  Vi      +3.3V（请勿接3.3V以上电源，否则烧毁模块！）
  TX      2
  RX      3
  GND    GND
*/

#include <Adafruit_Fingerprint.h>
#include <Servo.h>

#define mySerial Serial2
#define ANGLE_ON 90 //开门时的舵机角度
#define ANGLE_OFF 0 //关门时的舵机角度
#define SEVER_PIN 5 //舵机引脚

// uint8_t getFingerprintID();
uint8_t getFingerprintIDez();
void Touch1_Event();
void Touch2_Event();
void Touch3_Event();
void PinIntEvent();

Servo myservo;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
uint8_t tourch;
uint8_t fingTourch;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");
  myservo.attach(SEVER_PIN);                  //初始化舵机连接
  touchAttachInterrupt(T0, Touch1_Event, 50); //初始化触摸引脚T0
  touchAttachInterrupt(T7, Touch2_Event, 50); //初始化触摸引脚T2
  touchAttachInterrupt(T3, Touch3_Event, 50); //初始化触摸引脚T3
  pinMode(26, INPUT_PULLDOWN);                //初始化外部中斷引脚 26
  attachInterrupt(26, PinIntEvent, RISING);

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword())
  {
    Serial.println("Found fingerprint sensor!");
  }
  else
  {
    Serial.println("Did not find fingerprint sensor ");
    while (1)
    {
      delay(1);
    }
  }

  finger.getTemplateCount();
  Serial.print("Sensor contains ");
  Serial.print(finger.templateCount);
  Serial.println(" templates");
  Serial.println("Waiting for valid finger...");
  finger.LEDcontrol(false); //
  myservo.write(ANGLE_OFF);
}

void loop()
{
  uint8_t id = 255;
  uint8_t wakeFlag = 0;
  // uint8_t wakeFlag = 0;
  if (fingTourch == 1)
  {
    id = getFingerprintIDez();
    wakeFlag = 1;
  }
  if (id != 255) //指纹匹配，外侧开门
  {
    myservo.attach(SEVER_PIN); //连接舵机
    myservo.write(ANGLE_ON);   //开门
    Serial.printf("%d\r\n", id);
    delay(2000); //延时2s，等待执行到位
    id = 255;    //清空指纹匹配标志位
    fingTourch = 0;
    finger.LEDcontrol(false); //Light Off
    myservo.detach();         //执行完毕后将舵机失能，防止受力损坏
  }
  else if (id == 255 && wakeFlag)
  {
    finger.LEDcontrol(false); //Light Off
    fingTourch = 0;
    myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
  }
  else if (tourch == 1) //外侧关门事件
  {
    myservo.attach(SEVER_PIN);
    myservo.write(ANGLE_OFF);
    delay(2000);
    tourch = 0;
    myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
  }
  else if (tourch == 2) //内侧开门事件
  {
    myservo.attach(SEVER_PIN);
    myservo.write(ANGLE_ON);
    delay(2000);
    tourch = 0;
    myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
  }
  else if (tourch == 3) //内侧关门事件
  {
    myservo.attach(SEVER_PIN);
    myservo.write(ANGLE_OFF);
    delay(2000);
    tourch = 0;
    myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
  }

  // delay(50);
}
/**
 * @description: 
 * @param {*}
 * @return {*}
 */
void PinIntEvent()
{
  fingTourch = 1;
  Serial.printf("PinInt Event.\r\n");
}
/**
 * @description: T1触摸事件 外侧关门
 * @param {*}
 * @return {*}
 */
void Touch1_Event()
{
  tourch = 1;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T0);
  Serial.printf("Touch Event 1 OutDoor Close %d\r\n", tourchvalue);
}
/**
 * @description: T2触摸事件 内侧开门
 * @param {*}
 * @return {*}
 */
void Touch2_Event()
{
  tourch = 2;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T7);
  Serial.printf("Touch Event 2 InDoor Open %d\r\n", tourchvalue);
}
/**
 * @description: T3触摸事件 内侧关门
 * @param {*}
 * @return {*}
 */
void Touch3_Event()
{
  tourch = 3;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T3);
  Serial.printf("Touch Event 3 InDoor Close %d\r\n", tourchvalue);
}

uint8_t getFingerprintIDez()
{
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)
    return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)
    return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)
    return -1;

  // found a match!
  Serial.print("Found ID #");
  Serial.print(finger.fingerID);
  Serial.print(" with confidence of ");
  Serial.println(finger.confidence);
  return finger.fingerID;
}
