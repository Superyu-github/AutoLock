/*
  Vi      +3.3V（请勿接3.3V以上电源，否则烧毁模块！）
  TX      2
  RX      3
  GND    GND
*/
#define BLINKER_WIFI
#include <Adafruit_Fingerprint.h>
#include <Servo.h>
#include <Blinker.h>


#define mySerial Serial2
#define ANGLE_ON 90 //开门时的舵机角度
#define ANGLE_OFF 0 //关门时的舵机角度
#define SEVER_PIN 5 //舵机引脚




char auth[] = "117293f4821c";
char ssid[] = "PDCN";
char pswd[] = "niuzi203203";

// RTC_DATA_ATTR int bootCount = 0;
int bootCount = 0;
// uint8_t getFingerprintID();
uint8_t getFingerprintIDez();
void Touch1_Event();
void Touch2_Event();
void Touch3_Event();
void PinIntEvent();
void button1_callback(const String & state);

Servo myservo;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
uint8_t tourch;
uint8_t fingTourch;
uint16_t timeCount;
// 新建组件对象
BlinkerButton Button1("btn-afz");

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");
  myservo.attach(SEVER_PIN);                  //初始化舵机连接
  touchAttachInterrupt(T0, Touch1_Event, 40); //初始化触摸引脚T0
  touchAttachInterrupt(T7, Touch2_Event, 40); //初始化触摸引脚T2
  touchAttachInterrupt(T3, Touch3_Event, 40); //初始化触摸引脚T3
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
  // esp_sleep_enable_touchpad_wakeup();
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_26, 1);
  Serial.printf("bootCount:%d ",bootCount);
  // if(bootCount == 0)
    // esp_deep_sleep_start();
  
  // esp_light_sleep_start();
  BLINKER_DEBUG.stream(Serial);
  Blinker.begin(auth, ssid, pswd);  
  Button1.attach(button1_callback);
}

void loop()
{
  Blinker.run();
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
  if(timeCount>3000)
  {
    timeCount = 0;
    // Serial.printf("sleep\n");

    // esp_deep_sleep_start();
  }
  timeCount++;
  Serial.printf("timeCount:%d\r\n",timeCount);

  // delay(50);
}
/**
 * @description: 
 * @param {*}
 * @return {*}
 */
void PinIntEvent()
{
  // bootCount = 0;
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
  // bootCount++;
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
// 按下按键即会执行该函数
void button1_callback(const String & state) {
    BLINKER_LOG("get button state: ", state);
    if (state=="on") {
        // digitalWrite(LED_BUILTIN, LOW);
        // 反馈开关状态
        Button1.print("on");
    } else if(state=="off"){
        // digitalWrite(LED_BUILTIN, HIGH);
        // 反馈开关状态
        Button1.print("off");
    }
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
