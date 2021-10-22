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

/*WIFI连接信息*/
char auth[] = "";
char ssid[] = "";
char pswd[] = "";

/*门锁状态枚举*/
typedef enum
{
  LOCK_OUTDOOL_TOUCH = 0,    //外侧关门
  LOCK_INDOOL_TOUCH = 1,     //内侧关门
  LOCK_VIA_NET = 2,          //网络关门
  UNLOCK_OUTDOOL_FINGER = 3, //外侧开门
  UNLOCK_INDOOL_TOUCH = 4,   //外侧开门
  UNLOCK_VIA_NET = 5,        //网络开门
} dool_status_e;
/*门锁状态记录结构体*/
typedef struct
{
  dool_status_e dool_state;
  dool_status_e dool_last_state;
  uint8_t fing_tourch;
  uint8_t fing_id;
  uint8_t fing_wake_flag;
} auto_lock_t;

/*********************************************************
 * 函数声明
 * *******************************************************/
void AutoLockInit(auto_lock_t *auto_lock);
void LockControl(auto_lock_t *auto_lock);
void FingerInit();
void FingDetector(auto_lock_t *auto_lock);
uint8_t getFingerprintIDez();
void UnLock();
void Lock();
void FingerTouchEvent();
void OutdoorTouchOffEvent();
void IndoorTouchOnEvent();
void IndoorTouchOffEvent();
void Button1Callback(const String &state);

Servo myservo;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
/*定义门锁状态记录*/
auto_lock_t auto_lock;
/*新建组件对象*/
BlinkerButton Button1("btn-afz");

/*********************************************************
 * 默认函数
 * *******************************************************/
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(100);
  myservo.attach(SEVER_PIN);                          //初始化舵机连接
  touchAttachInterrupt(T0, OutdoorTouchOffEvent, 40); //触摸引脚T0回调函数绑定
  touchAttachInterrupt(T7, IndoorTouchOnEvent, 40);   //触摸引脚T7回调函数绑定
  touchAttachInterrupt(T3, IndoorTouchOffEvent, 40);  //触摸引脚T3回调函数绑定
  pinMode(26, INPUT_PULLDOWN);                        //初始化外部中斷引脚 26
  attachInterrupt(26, FingerTouchEvent, RISING);      //外部中断回调函数绑定
  AutoLockInit(&auto_lock);                           //初始化门锁状态结构体
  FingerInit();                                       //指纹模块初始化
  BLINKER_DEBUG.stream(Serial);                       //将BLINKER调试信息输出至串口
  Blinker.begin(auth, ssid, pswd);                    //初始化BLINKER连接
  Button1.attach(Button1Callback);                    // BLINKER按钮绑定
}

void loop()
{
  Blinker.run();
  FingDetector(&auto_lock);
  LockControl(&auto_lock);
}
/*********************************************************
 * 用户函数定义
 * *******************************************************/
/**
 * @description: 门锁结构体初始化函数
 * @param {auto_lock_t} *auto_lock
 * @return {*}
 */
void AutoLockInit(auto_lock_t *auto_lock)
{
  auto_lock->dool_state = LOCK_OUTDOOL_TOUCH;
  auto_lock->fing_id = 255;
  auto_lock->fing_tourch = 0;
  auto_lock->fing_wake_flag = 0;
}
/**
 * @description: 门锁控制函数
 * @param {auto_lock_t} *auto_lock
 * @return {*}
 */
#define LOCK < UNLOCK_OUTDOOL_FINGER //按照枚举编号，小于此的为上锁
#define UNLOCK > LOCK_VIA_NET        //按照枚举编号，大于此的为解锁
void LockControl(auto_lock_t *auto_lock)
{
  /*本次锁定状态和上次锁定状态不同时才执行动作，防止执行器产生振荡*/
  if (auto_lock->dool_state LOCK && auto_lock->dool_last_state UNLOCK)
  {
    Lock();
  }
  else if (auto_lock->dool_state UNLOCK && auto_lock->dool_last_state LOCK)
  {
    UnLock();
  }
  auto_lock->dool_last_state = auto_lock->dool_state;
}
/**
 * @description: 指纹模块初始化函数
 * @param {*}
 * @return {*}
 */
void FingerInit()
{
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
  finger.LEDcontrol(false);
}
/**
 * @description: 指纹探测函数
 * @param {auto_lock_t} *auto_lock
 * @return {*}
 */
void FingDetector(auto_lock_t *auto_lock)
{
  if (auto_lock->fing_tourch == 1)
  {
    auto_lock->fing_id = getFingerprintIDez(); //扫描指纹ID
    auto_lock->fing_wake_flag = 1;
  }
  if (auto_lock->fing_id != 255) //指纹匹配，返回ID号，不是默认值255
  {
    auto_lock->dool_state = UNLOCK_OUTDOOL_FINGER;
    Serial.printf("%d\r\n", auto_lock->fing_id);
    auto_lock->fing_id = 255;   //清空指纹匹配标志位
    auto_lock->fing_tourch = 0; //清空触摸唤醒标志位
    finger.LEDcontrol(false);   // Light Off
  }
  else if (auto_lock->fing_id == 255 && auto_lock->fing_wake_flag) //指纹不匹配且处于唤醒状态
  {
    finger.LEDcontrol(false);   // Light Off
    auto_lock->fing_tourch = 0; //清空触摸唤醒标志位
    // myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
  }
}
/**
 * @description: 指纹ID读取函数(取自arduino论坛)
 * @param {*}
 * @return {*}
 */
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
/**
 * @description: 解锁执行函数
 * @param {*}
 * @return {*}
 */
void UnLock()
{
  myservo.attach(SEVER_PIN);
  myservo.write(ANGLE_ON);
  delay(2000);
  myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
}
/**
 * @description: 上锁执行函数
 * @param {*}
 * @return {*}
 */
void Lock()
{
  myservo.attach(SEVER_PIN);
  myservo.write(ANGLE_OFF);
  delay(2000);
  myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
}

/*********************************************************
 * 回调函数
 * *******************************************************/

/**
 * @description: 指纹触摸事件
 * @param {*}
 * @return {*}
 */
void FingerTouchEvent()
{
  auto_lock.fing_tourch = 1;
  Serial.printf("PinInt Event.\r\n");
}
/**
 * @description: T1触摸事件 外侧关门
 * @param {*}
 * @return {*}
 */
void OutdoorTouchOffEvent()
{
  auto_lock.dool_state = LOCK_OUTDOOL_TOUCH;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T0);
  Serial.printf("Touch Event 1 OutDoor Close %d\r\n", tourchvalue);
}
/**
 * @description: T2触摸事件 内侧开门
 * @param {*}
 * @return {*}
 */
void IndoorTouchOnEvent()
{
  auto_lock.dool_state = UNLOCK_INDOOL_TOUCH;
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
void IndoorTouchOffEvent()
{
  auto_lock.dool_state = LOCK_INDOOL_TOUCH;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T3);
  Serial.printf("Touch Event 3 InDoor Close %d\r\n", tourchvalue);
}
// 按下按键即会执行该函数
/**
 * @description: BLINKER按钮回调函数
 * @param {String} &state
 * @return {*}
 */
void Button1Callback(const String &state)
{
  BLINKER_LOG("get button state: ", state);
  if (state == "on")
  {
    auto_lock.dool_state = UNLOCK_VIA_NET;
    // 反馈开关状态
    Button1.print("on");
  }
  else if (state == "off")
  {
    auto_lock.dool_state = LOCK_VIA_NET;
    // 反馈开关状态
    Button1.print("off");
  }
}
