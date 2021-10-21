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


#define mySerial Serial2
#define ANGLE_ON 90 //����ʱ�Ķ���Ƕ�
#define ANGLE_OFF 0 //����ʱ�Ķ���Ƕ�
#define SEVER_PIN 5 //�������




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
// �½��������
BlinkerButton Button1("btn-afz");

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");
  myservo.attach(SEVER_PIN);                  //��ʼ���������
  touchAttachInterrupt(T0, Touch1_Event, 40); //��ʼ����������T0
  touchAttachInterrupt(T7, Touch2_Event, 40); //��ʼ����������T2
  touchAttachInterrupt(T3, Touch3_Event, 40); //��ʼ����������T3
  pinMode(26, INPUT_PULLDOWN);                //��ʼ���ⲿ�Д����� 26
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
  if (id != 255) //ָ��ƥ�䣬��࿪��
  {
    myservo.attach(SEVER_PIN); //���Ӷ��
    myservo.write(ANGLE_ON);   //����
    Serial.printf("%d\r\n", id);
    delay(2000); //��ʱ2s���ȴ�ִ�е�λ
    id = 255;    //���ָ��ƥ���־λ
    fingTourch = 0;
    finger.LEDcontrol(false); //Light Off
    myservo.detach();         //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
  }
  else if (id == 255 && wakeFlag)
  {
    finger.LEDcontrol(false); //Light Off
    fingTourch = 0;
    myservo.detach(); //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
  }
  else if (tourch == 1) //�������¼�
  {
    myservo.attach(SEVER_PIN);
    myservo.write(ANGLE_OFF);
    delay(2000);
    tourch = 0;
    myservo.detach(); //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
  }
  else if (tourch == 2) //�ڲ࿪���¼�
  {
    myservo.attach(SEVER_PIN);
    myservo.write(ANGLE_ON);
    delay(2000);
    tourch = 0;
    myservo.detach(); //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
  }
  else if (tourch == 3) //�ڲ�����¼�
  {
    myservo.attach(SEVER_PIN);
    myservo.write(ANGLE_OFF);
    delay(2000);
    tourch = 0;
    myservo.detach(); //ִ����Ϻ󽫶��ʧ�ܣ���ֹ������
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
 * @description: T1�����¼� ������
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
 * @description: T2�����¼� �ڲ࿪��
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
 * @description: T3�����¼� �ڲ����
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
// ���°�������ִ�иú���
void button1_callback(const String & state) {
    BLINKER_LOG("get button state: ", state);
    if (state=="on") {
        // digitalWrite(LED_BUILTIN, LOW);
        // ��������״̬
        Button1.print("on");
    } else if(state=="off"){
        // digitalWrite(LED_BUILTIN, HIGH);
        // ��������״̬
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
