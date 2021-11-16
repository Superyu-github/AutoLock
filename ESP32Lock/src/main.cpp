/*
  Vi      +3.3V�������3.3V���ϵ�Դ�������ջ�ģ�飡��
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
#define ANGLE_ON 90 //����ʱ�Ķ���Ƕ�
#define ANGLE_OFF 0 //����ʱ�Ķ���Ƕ�
#define SEVER_PIN 5 //�������
#define DETACH_TIME 4000000

/*WIFI������Ϣ*/
char auth[] = "";
char ssid[] = "";
char pswd[] = "";

/*����״̬ö��*/
typedef enum
{
  LOCK_OUTDOOL_TOUCH = 0,    //������
  LOCK_INDOOL_TOUCH = 1,     //�ڲ����
  LOCK_VIA_NET = 2,          //�������
  UNLOCK_OUTDOOL_FINGER = 3, //��࿪��
  UNLOCK_INDOOL_TOUCH = 4,   //�ڲ࿪��
  UNLOCK_VIA_NET = 5,        //���翪��
} dool_status_e;
/*����״̬��¼�ṹ��*/
typedef struct
{
  dool_status_e dool_state;
  dool_status_e dool_last_state;
  uint8_t fing_tourch;
  uint8_t fing_id;
  uint8_t fing_wake_flag;
  uint8_t completed_flag;
} auto_lock_t;
/*DHT���������ݽṹ��*/
typedef struct
{
  float humi;
  float temp;
} dht_t;
/*********************************************************
 * ��������
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
/*����DHT����������*/
dht_t dht_data;
/*��������״̬��¼*/
auto_lock_t auto_lock;
/*�½��������*/
BlinkerButton Button1("btn-afz");
BlinkerButton Button2("btn-refresh");
BlinkerNumber HUMI("humi");
BlinkerNumber TEMP("temp");

/*********************************************************
 * Ĭ�Ϻ���
 * *******************************************************/
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(100);
  myservo.attach(SEVER_PIN);                          //��ʼ���������
  timer = timerBegin(1, 80, true);                    //��Ƶϵ���趨
  timerAlarmWrite(timer, DETACH_TIME, true);          //��ʱ�����趨
  timerAttachInterrupt(timer, &TimerEvent, true);     //��ʱ���ص�������
  touchAttachInterrupt(T0, OutdoorTouchOffEvent, 35); //��������T0�ص�������
  touchAttachInterrupt(T7, IndoorTouchOnEvent, 35);   //��������T7�ص�������
  touchAttachInterrupt(T3, IndoorTouchOffEvent, 35);  //��������T3�ص�������
  pinMode(26, INPUT_PULLDOWN);                        //��ʼ���ⲿ�Д����� 26
  attachInterrupt(26, FingerTouchEvent, RISING);      //�ⲿ�жϻص�������
  AutoLockInit(&auto_lock);                           //��ʼ������״̬�ṹ��
  FingerInit();                                       //ָ��ģ���ʼ��
  // BLINKER_DEBUG.stream(Serial);                       //��BLINKER������Ϣ���������
  // BLINKER_DEBUG.debugAll();                           //�����ϸ��Ϣ
  Blinker.begin(auth, ssid, pswd);                    //��ʼ��BLINKER����
  Blinker.attachHeartbeat(HeartBeat);                 //������������
  Button1.attach(Button1Callback);                    // BLINKER��ť��
  Button2.attach(Button2Callback);                    // BLINKER��ť��
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
 * �û���������
 * *******************************************************/
/**
 * @description: �����ṹ���ʼ������
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
 * @description: �������ƺ���
 * @param {auto_lock_t} *auto_lock
 * @return {*}
 */
#define LOCK < UNLOCK_OUTDOOL_FINGER //����ö�ٱ�ţ�С�ڴ˵�Ϊ����
#define UNLOCK > LOCK_VIA_NET        //����ö�ٱ�ţ����ڴ˵�Ϊ����
void LockControl(auto_lock_t *auto_lock)
{
  /*��������״̬���ϴ�����״̬��ͬʱ��ִ�ж�������ִֹ����������*/
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
 * @description: ָ��ģ���ʼ������
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
 * @description: ָ��̽�⺯��
 * @param {auto_lock_t} *auto_lock
 * @return {*}
 */
void FingDetector(auto_lock_t *auto_lock)
{
  if (auto_lock->fing_tourch == 1)
  {
    auto_lock->fing_id = getFingerprintIDez(); //ɨ��ָ��ID
    auto_lock->fing_wake_flag = 1;
  }
  if (auto_lock->fing_id != 255) //ָ��ƥ�䣬����ID�ţ�����Ĭ��ֵ255
  {
    auto_lock->dool_state = UNLOCK_OUTDOOL_FINGER;
    Serial.printf("%d\r\n", auto_lock->fing_id);
    auto_lock->fing_id = 255;   //���ָ��ƥ���־λ
    auto_lock->fing_tourch = 0; //��մ������ѱ�־λ
    finger.LEDcontrol(false);   // Light Off
  }
  else if (auto_lock->fing_id == 255 && auto_lock->fing_wake_flag) //ָ�Ʋ�ƥ���Ҵ��ڻ���״̬
  {
    finger.LEDcontrol(false);   // Light Off
    auto_lock->fing_tourch = 0; //��մ������ѱ�־λ
    // myservo.detach(); //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
  }
}
/**
 * @description: ָ��ID��ȡ����(ȡ��arduino��̳)
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
 * @description: ����ִ�к���
 * @param {*}
 * @return {*}
 */
void UnLock()
{
  // timerAlarmEnable(timer); //ʹ�ܶ�ʱ��
  // timerRestart(timer);     //������ʱ��
  // myservo.attach(SEVER_PIN);
  myservo.write(ANGLE_ON);
  // Button1.print("on");
  // myservo.detach(); //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
}
/**
 * @description: ����ִ�к���
 * @param {*}
 * @return {*}
 */
void Lock()
{
  // timerAlarmEnable(timer); //ʹ�ܶ�ʱ��
  // timerRestart(timer);     //������ʱ��
  // myservo.attach(SEVER_PIN);
  myservo.write(ANGLE_OFF);
  // Button1.print("off");

  // myservo.detach(); //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
}
/**
 * @description: ��DHT��������ȡ��ʪ����Ϣ
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
 * �ص�����
 * *******************************************************/
/**
 * @description: ������
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
 * @description: ��ʱ���жϻص�����
 * @param {*}
 * @return {*}
 */
void IRAM_ATTR TimerEvent()
{
  myservo.detach();         //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
  if (auto_lock.dool_state LOCK)
    Button1.print("off");
  else
    Button1.print("on");
  // myservo.detach();         //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
  timerAlarmDisable(timer); //ʧ�ܶ�ʱ��
  // timerEnd(timer);
}
/**
 * @description: ָ�ƴ����¼�
 * @param {*}
 * @return {*}
 */
void FingerTouchEvent()
{
  timerRestart(timer);     //������ʱ��
  timerAlarmEnable(timer); //ʹ�ܶ�ʱ��
  myservo.attach(SEVER_PIN);
  auto_lock.fing_tourch = 1;
  Serial.printf("PinInt Event.\r\n");
}
/**
 * @description: T1�����¼� ������
 * @param {*}
 * @return {*}
 */
void OutdoorTouchOffEvent()
{
  timerRestart(timer);     //������ʱ��
  timerAlarmEnable(timer); //ʹ�ܶ�ʱ��
  myservo.attach(SEVER_PIN);
  auto_lock.dool_state = LOCK_OUTDOOL_TOUCH;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T0);
  Serial.printf("Touch Event 1 OutDoor Close %d\r\n", tourchvalue);
}
/**
 * @description: T2�����¼� �ڲ࿪��
 * @param {*}
 * @return {*}
 */
void IndoorTouchOnEvent()
{
  timerRestart(timer);     //������ʱ��
  timerAlarmEnable(timer); //ʹ�ܶ�ʱ��
  myservo.attach(SEVER_PIN);
  auto_lock.dool_state = UNLOCK_INDOOL_TOUCH;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T7);
  Serial.printf("Touch Event 2 InDoor Open %d\r\n", tourchvalue);
}
/**
 * @description: T3�����¼� �ڲ����
 * @param {*}
 * @return {*}
 */
void IndoorTouchOffEvent()
{
  timerRestart(timer);     //������ʱ��
  timerAlarmEnable(timer); //ʹ�ܶ�ʱ��
  myservo.attach(SEVER_PIN);
  auto_lock.dool_state = LOCK_INDOOL_TOUCH;
  uint16_t tourchvalue;
  tourchvalue = touchRead(T3);
  Serial.printf("Touch Event 3 InDoor Close %d\r\n", tourchvalue);
}
// ���°�������ִ�иú���
/**
 * @description: BLINKER��ť�ص�������������
 * @param {String} &state
 * @return {*}
 */
void Button1Callback(const String &state)
{
  timerRestart(timer);     //������ʱ��
  timerAlarmEnable(timer); //ʹ�ܶ�ʱ��
  myservo.attach(SEVER_PIN);
  BLINKER_LOG("get button state: ", state);
  if (state == "on")
  {
    auto_lock.dool_state = UNLOCK_VIA_NET;
    // ��������״̬
    Button1.print("on");
  }
  else if (state == "off")
  {
    auto_lock.dool_state = LOCK_VIA_NET;
    // ��������״̬
    Button1.print("off");
  }
  /**
   * @description: BLINKER��ť�ػص�����������״̬
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