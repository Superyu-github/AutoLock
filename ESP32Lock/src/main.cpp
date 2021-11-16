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
#include <DHT.h>

#define DHTPIN 18
#define DHTTYPE DHT11 // DHT 11
#define mySerial Serial2
#define ANGLE_ON 90 //开门时的舵机角度
#define ANGLE_OFF 0 //关门时的舵机角度
#define SEVER_PIN 5 //舵机引脚
#define DETACH_TIME 4000000

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
  UNLOCK_INDOOL_TOUCH = 4,   //内侧开门
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
  uint8_t completed_flag;
} auto_lock_t;
/*DHT传感器数据结构体*/
typedef struct
{
  float humi;
  float temp;
} dht_t;
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
void GetDhtData();
void HeartBeat();
void IRAM_ATTR TimerEvent();
void FingerTouchEvent();
void OutdoorTouchOffEvent();
void IndoorTouchOnEvent();
void IndoorTouchOffEvent();
void Button1Callback(const String &state);
void Button2Callback(const String &state);

hw_timer_t *timer = NULL;
Servo myservo;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
DHT dht(DHTPIN, DHTTYPE);
/*定义DHT传感器数据*/
dht_t dht_data;
/*定义门锁状态记录*/
auto_lock_t auto_lock;
/*新建组件对象*/
BlinkerButton Button1("btn-afz");
BlinkerButton Button2("btn-refresh");
BlinkerNumber HUMI("humi");
BlinkerNumber TEMP("temp");

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
  timer = timerBegin(1, 80, true);                    //分频系数设定
  timerAlarmWrite(timer, DETACH_TIME, true);          //定时参数设定
  timerAttachInterrupt(timer, &TimerEvent, true);     //定时器回调函数绑定
  touchAttachInterrupt(T0, OutdoorTouchOffEvent, 35); //触摸引脚T0回调函数绑定
  touchAttachInterrupt(T7, IndoorTouchOnEvent, 35);   //触摸引脚T7回调函数绑定
  touchAttachInterrupt(T3, IndoorTouchOffEvent, 35);  //触摸引脚T3回调函数绑定
  pinMode(26, INPUT_PULLDOWN);                        //初始化外部中斷引脚 26
  attachInterrupt(26, FingerTouchEvent, RISING);      //外部中断回调函数绑定
  AutoLockInit(&auto_lock);                           //初始化门锁状态结构体
  FingerInit();                                       //指纹模块初始化
  // BLINKER_DEBUG.stream(Serial);                       //将BLINKER调试信息输出至串口
  // BLINKER_DEBUG.debugAll();                           //输出详细信息
  Blinker.begin(auth, ssid, pswd);                    //初始化BLINKER连接
  Blinker.attachHeartbeat(HeartBeat);                 //心跳包函数绑定
  Button1.attach(Button1Callback);                    // BLINKER按钮绑定
  Button2.attach(Button2Callback);                    // BLINKER按钮绑定
  dht.begin();

}

void loop()
{
  Blinker.run();
  FingDetector(&auto_lock);
  LockControl(&auto_lock);
  GetDhtData();
  // BLINKER_LOG(auto_lock.dool_state);
  // delay(1);
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
  auto_lock->completed_flag = 1;
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
  // if (auto_lock->dool_state LOCK && auto_lock->dool_last_state UNLOCK && auto_lock->completed_flag)
  if (auto_lock->dool_state LOCK)
  {
    Lock();
  }
  else if (auto_lock->dool_state UNLOCK)
  {
    UnLock();
  }
  // auto_lock->dool_last_state = auto_lock->dool_state;
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
  // timerAlarmEnable(timer); //使能定时器
  // timerRestart(timer);     //重启定时器
  // myservo.attach(SEVER_PIN);
  myservo.write(ANGLE_ON);
  // Button1.print("on");
  // myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
}
/**
 * @description: 上锁执行函数
 * @param {*}
 * @return {*}
 */
void Lock()
{
  // timerAlarmEnable(timer); //使能定时器
  // timerRestart(timer);     //重启定时器
  // myservo.attach(SEVER_PIN);
  myservo.write(ANGLE_OFF);
  // Button1.print("off");

  // myservo.detach(); //执行完毕后将舵机失能，防止受力损坏
}
/**
 * @description: 从DHT传感器获取温湿度信息
 * @param {*}
 * @return {*}
 */
void GetDhtData()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    BLINKER_LOG("Failed to read from DHT sensor!");
  }
  else
  {
    BLINKER_LOG("Humidity: ", h, " %");
    BLINKER_LOG("Temperature: ", t, " *C");
    dht_data.humi = h;
    dht_data.temp = t;
  }
  // Blinker.delay(2000);
}
/*********************************************************
 * 回调函数
 * *******************************************************/
/**
 * @description: 心跳包
 * @param {*}
 * @return {*}
 */
void HeartBeat()
{
  HUMI.print(dht_data.humi);
  TEMP.print(dht_data.temp);
  if (auto_lock.dool_state LOCK)
    Button1.print("off");
  else
    Button1.print("on");
}
/**
 * @description: 定时器中断回调函数
 * @param {*}
 * @return {*}
 */
void IRAM_ATTR TimerEvent()
{
  myservo.detach();         //执行完毕后将舵机失能，防止受力损坏
  if (auto_lock.dool_state LOCK)
    Button1.print("off");
  else
    Button1.print("on");
  // myservo.detach();         //执行完毕后将舵机失能，防止受力损坏
  timerAlarmDisable(timer); //失能定时器
  // timerEnd(timer);
}
/**
 * @description: 指纹触摸事件
 * @param {*}
 * @return {*}
 */
void FingerTouchEvent()
{
  timerRestart(timer);     //重启定时器
  timerAlarmEnable(timer); //使能定时器
  myservo.attach(SEVER_PIN);
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
  timerRestart(timer);     //重启定时器
  timerAlarmEnable(timer); //使能定时器
  myservo.attach(SEVER_PIN);
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
  timerRestart(timer);     //重启定时器
  timerAlarmEnable(timer); //使能定时器
  myservo.attach(SEVER_PIN);
  auto_lock.dool_state = UNLOCK_INDOOL_TOUCH;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T7);
  Serial.printf("Touch Event 2 InDoor Open %d\r\n", tourchvalue);
}
/**
 * @description: T3触摸事件 内侧关门
 * @param {*}
 * @return {*}
 */
void IndoorTouchOffEvent()
{
  timerRestart(timer);     //重启定时器
  timerAlarmEnable(timer); //使能定时器
  myservo.attach(SEVER_PIN);
  auto_lock.dool_state = LOCK_INDOOL_TOUCH;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T3);
  Serial.printf("Touch Event 3 InDoor Close %d\r\n", tourchvalue);
}
// 按下按键即会执行该函数
/**
 * @description: BLINKER按钮回调函数，开关锁
 * @param {String} &state
 * @return {*}
 */
void Button1Callback(const String &state)
{
  timerRestart(timer);     //重启定时器
  timerAlarmEnable(timer); //使能定时器
  myservo.attach(SEVER_PIN);
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
  /**
   * @description: BLINKER按钮回回调函数，更新状态
   * @param {*}
   * @return {*}
   */
}
void Button2Callback(const String &state)
{
  BLINKER_LOG("get button state: ", state);
  HUMI.print(dht_data.humi);
  TEMP.print(dht_data.temp);
  if (auto_lock.dool_state LOCK)
    Button1.print("off");
  else
    Button1.print("on");
}