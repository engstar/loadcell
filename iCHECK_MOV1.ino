//#define SHINSUNG
//https://github.com/espressif/arduino-esp32/releases/download/1.0.0/package_esp32_dev_index.json
//arduino_json 5.13.5 version
//const char* ssid     = "shinsunglab2G";
//const char* password = "sstk0824";
//const char* ssid     = "friendclinic_2";
//const char* password = "58005800@@";
const char* ssid     = "jong";
const char* password = "sstk0824";

#include <ArduinoJson.h>
#include <Arduino.h>
#include <Wire.h>
#include <Ticker.h>
#include <WiFi.h>
#define I2C_ADDR 0x70
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "HX711.h" //로드셀
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
//https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM
#include <EEPROM.h>
Ticker timer;

//define pins of TFT screen
#define TFT_CS 12
#define TFT_RST 14
#define TFT_DC 13
#define TFT_SCLK 22
#define TFT_MOSI 21
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
float p = 3.1415926;

#define FIRST_X 4
#define FIRST_Y 2

#define SOUND 5

#define SN_SIZE 20

#define HOST_URL_LENGTH 150 //http://211.194.14.9:41254/IringerVer2/
char host_url[HOST_URL_LENGTH] = "https://iringer.kr:8443/";
HTTPClient http;

char base64[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
char serial_num[SN_SIZE + 1] = {0, };

#define DOUT 4  // 로드셀 데이터핀
#define CLK 16   // 로드셀 클럭핀

HX711 scale;

float calibration_factor = 6100;

#define boot 0

//#define ir_tx 16
#define led_power 23
#define usb_detect 34
#define ESP_OUT 15

#define tft_on 17 // tft screen on

uint32_t old_bat = 0;
uint32_t bat = 0;
uint32_t rest = 0;
#define adcpin 39


int adcValue = 0;
int temp = 0;
int temp1 = 0;

uint8_t mtag = 0;//tjio  display control
u16_t sec_tmr = 0;
//u8_t sec_flag = 0;
uint32_t tmr = 0;

unsigned long current_sec = 0;
unsigned long prev_sec = 0;
bool cali_set = 0;//tjio rtn
bool zero = 0;
float cali_factor = 600.0f;
float oldcali_factor = 0.0f;

float fweight[10] = {0,};
float fave_value = 0;
float value = 0.0f;
float oldave_value = 999.0f;
float ave_value = 0.0f; // current Measure

struct Button
{
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

//Button wps_key = {32, 0, false};
Button power_key = {26, 0, false};
//Button ir_rx = {17, 0, false};
Button zero_key = {25 , 0 , false};


//int pnIrRx = 17;

#include <Fonts/FreeSerifItalic9pt7b.h>
#include <Fonts/FreeSerifItalic12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>

void _printf(const char *s, ...)
{
  va_list args;
  va_start(args, s);
  int n = vsnprintf(NULL, 0, s, args);
  char *str = new char[n + 1];
  vsprintf(str, s, args);
  va_end(args);
  Serial.print(str);
  delete[] str;
}


void read_sn()
{
  //printf("read_sn() start\n");
  int i;
  char c[SN_SIZE + 1] = {0};
  for (i = 0; i < SN_SIZE; i++)
  {
    if (c[0] == 0xff)
      break;
    if (c[i] == 0x0)
      break;
  }
  _printf("serial num %s with length %d\n", c, i);
  if (i == 0) {
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print(mac[3], HEX);
    Serial.print(":");
    Serial.print(mac[4], HEX);
    Serial.print(":");
    Serial.println(mac[5], HEX);
    int num = int((unsigned char)(mac[3]) << 16 |
                  (unsigned char)(mac[4]) << 8 |
                  (unsigned char)(mac[5]));
    char new_serial[13] = "iRinger-1234";
    new_serial[8] = base64[(num >> 18) % 64];
    new_serial[9] = base64[(num >> 12) % 64];
    new_serial[10] = base64[(num >> 6) % 64];
    new_serial[11] = base64[num % 64];
    memcpy(serial_num, new_serial, 13);
    Serial.println(serial_num);
    return;
  }
  memcpy(serial_num, c, sizeof(c));
  //printf("read_sn() serial_num1=%s\n",serial_num);
}

//tjio modify rtn

float fsave_value = 0;
float AUTO_OFFSET;
float LCELL_INIT_VALUE = -330000.;  //무부하 값으로 기기별로 매우 다를 수 있으며, 공장값을 입력해 줘야 합니다.
float LCELL_DIVIDER = 596.;         //기기별로 약간의 차이가 있으며, 공장값을 입력해 줘야 합니다.
float setup_offset = 176629;
int zero_exe=0;
void zero_set() {
  // Serial.println("Readings:");
  //결론 1) _scale 입력 2)_offset_ 설정 3)측정갑 표시  : set_scale(742.0f)->tare(20)->get_units(5)  5번 평균한값

  AUTO_OFFSET = scale.read_average(50);
  //  setup_offset = AUTO_OFFSET;
  fsave_value = (AUTO_OFFSET - setup_offset) / LCELL_DIVIDER;//setup_offset = 176629;

  Serial.print("Zero set  fsave_value  : \t");
  Serial.println(fsave_value);
  Serial.print("Zero set setup_offset:");
  Serial.println(setup_offset);
  setup_offset = setup_offset +(fsave_value * LCELL_DIVIDER);
                 Serial.print("After Zero set  fsave_value  : \t");
  Serial.println(fsave_value);
  Serial.print("After Zero set setup_offset:");
  Serial.println(setup_offset);
  for (int i = 0; i < 10; i++) {
    fweight[i] = fsave_value;
    ave_value = 0;
  }
  zero_exe++;
}



void alarmsound()
{
  uint8_t octave = 5;
  ledcWriteNote(0, NOTE_D, octave);
  delay(1000);
  ledcWriteTone(0, 0);
  delay(500);
}

void init_tft()
{
  tft.initR(INITR_144GREENTAB); //initialize a ST7735S chip, black tab
  tft.setRotation(0);
  tft.fillScreen(ST7735_BLACK); //large block of text
  tft.setTextWrap(false);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
}

void display_main()
{
  tft.setFont(&FreeSans12pt7b);
  tft.setCursor(FIRST_X + 95, FIRST_Y + 95);
  tft.setTextColor(ST7735_WHITE);
  tft.print("ml");
}


void print_data()
{

  tft.setFont(&FreeSans12pt7b);
  tft.setCursor(FIRST_X + 95, FIRST_Y + 95);
  tft.setTextColor(ST7735_WHITE);
  tft.print("ml");

  if (((int)ave_value < (int)oldave_value) || ((int)ave_value > (oldave_value + 2))) {

    tft.setCursor(45, 95);
    tft.setFont(&FreeSans12pt7b);
    tft.setTextColor(ST77XX_BLACK);
    tft.printf("%d", (int)oldave_value);

    tft.setCursor(45, 95);
    tft.setFont(&FreeSans12pt7b);
    tft.setTextColor(ST77XX_WHITE);
    tft.printf("%d", (int)ave_value);
    oldave_value = ave_value;

  }

  if (cali_set) {//calibration  display
    //   float cali_factor
    // if ((int)cali_factor != (int)oldcali_factor) {
    tft.setCursor(FIRST_X + 20, FIRST_Y + 70);
    tft.setTextColor(ST77XX_BLACK);
    tft.printf("%d", (int)oldcali_factor);

    tft.setCursor(FIRST_X + 20, FIRST_Y + 70);
    tft.setTextColor(ST77XX_WHITE);
    tft.printf("%d", (int)cali_factor);
    oldcali_factor = cali_factor;
    //  }
  }
  else {//All clear
    tft.setCursor(FIRST_X + 20, FIRST_Y + 70);
    tft.setTextColor(ST77XX_BLACK);
    tft.printf("%d", (int)oldcali_factor);
  }
}

int post_graphql_query(char* body, char* query) {
  char graphql_url[HOST_URL_LENGTH + 8] = { 0, };
  sprintf(graphql_url, "%smonitoring", host_url);

  http.begin(graphql_url);
  http.addHeader("Content-Type", "application/json");

  sprintf(body, "{ \"query\": \"%s\" }", query);

  _printf("POST %s\n %s\n", graphql_url, body);
  return http.POST(body);
}

byte send_data() //sd
{
  byte x = 1;
  char query[200] = { 0,};
  char body[220] = { 0, };

  String test;

  sprintf(
    query,
    "{ \\\"v2Monitoring\\\" : { \\\"sn\\\": \\\"%s\\\", \\\"weight\\\": %f, \\\"battery\\\": %d } }",
    serial_num, ave_value - 40 , bat
  );
  Serial.print("query 값 : ");
  Serial.println(query);

  int httpCode = post_graphql_query(body, query);

  if (httpCode == 200)
  {
    const size_t capacity = 2 * JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(6) + 120;
    DynamicJsonBuffer jsonBuffer(capacity);
    String str = http.getString();
    JsonObject &root = jsonBuffer.parseObject(str);


    x = 1;
    Serial.print("Str값:");
    Serial.println(str);
  }
  // else  {
  //    Serial.print("Error");

  //   x = 0;
  // }
  else
  {
    Serial.print("send_data error with code ");
    Serial.println(httpCode);

    x = 0;
  }

  http.end();
  return x;
  //return 1;
}

//tjio rtn modify
int adjust_value = 0; //calibration factor
bool cali_exe_flag = 0;
void IRAM_ATTR isr1()
{
  delay(5);// 채터링방지
  if ((digitalRead(power_key.PIN) == 0)) {

    power_key.pressed = 1; // press flag set
    power_key.numberKeyPresses = 0;//

    if (cali_set && (!cali_exe_flag)) {
      adjust_value++;
      prev_sec = current_sec;//키가 눌러 지면 완료 시기 연장 하기위해
      cali_exe_flag = 1;//cali_exe_flag는  1초이전에  재진입 방지 하기 위해
    }
  }
  //change interrupt use !
  else if ((digitalRead(power_key.PIN) != 0)) { // power_key 가 off 상태이면

    power_key.pressed = 0;
    //   power_key.numberKeyPresses = 0; // COUNT 0 set

  }
}

void IRAM_ATTR zero_sw()
{
  //  Serial.println(" 인터럽트 루틴 Zero sw 입니다");
  delay(5);// 채터링방지

  if ((digitalRead(zero_key.PIN) == 0)) {
    if (!cali_set) {
      zero_key.pressed = 1; // for zero adjust
      zero_key.numberKeyPresses = 0; // COUNT 0 set

    }
    else {
      adjust_value--;
      prev_sec = current_sec;
    }
  }

}

//2023 6 13
void batLevel(int batStep) { //green 폭은 20
  tft.fillRoundRect(94, 32, 20, 10, 2, ST77XX_BLACK);//x y w=20 h=10 round->r t color    지우고
  tft.fillRoundRect(94, 32, batStep, 10, 2, ST77XX_GREEN);//x y w=20 h=10 round->r t color
  tft.drawRect(92, 30, 24, 14, ST77XX_WHITE);//x-2 y-2 w=24 h=14  white
  tft.fillRoundRect(115, 34, 3, 5, 1, ST77XX_WHITE);//x=92+24-1,y=32+2, y=14/2=7-2(양쪽 1씩 ) h=14/2=5-2 round->r t color
}

#define ANALOGIC_RESOLUTION 4096 // The esp32 board resolution, for example, Arduino Uno is 10 bit (from 0 to 1023=1024)
#define REFERENCE_VOLTAGE 3.3 //(mcu 인가전압 3.3v)The reference voltage, for example, Arduino Uno works in 5 V reference voltage by default
#define R1 100000.0 // resistance of R1 (100k)
#define R2 100000.0 // resistance of R2 (100k)
#define EXPECTED_V_OUT 4.2 // The intented voltage, 4.2 V  충전전압

float bat_old_digit; // batterry 이전값

#define size 11
float fSvalue[11];

float FindMedianValue() {

  // 배열을 오름차순으로 정렬
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (fSvalue[j] > fSvalue[j + 1]) {
        float temp = fSvalue[j];
        fSvalue[j] = fSvalue[j + 1];
        fSvalue[j + 1] = temp;
      }
    }
  }//for end

  return fSvalue[5];//median out

}

int change_bat = 0;
void bat_measure() {
  int value = analogRead(adcpin); // Read value at analog input
  // value = analogRead(adcpin); // Read value at analog input  (오차를 줄이기 위하여  한번 더 읽어봄
  //median value find rtn  10ms 마다 battery check
  for (int i = 0; i < size; i++) {

    fSvalue[i] = analogRead(adcpin);

    //    Serial.println(fSvalue[i]);
    delay(10);
  }
  value = (int)FindMedianValue(); // median find
  value = constrain ( value, 1260, 2400 );    // 아날로그 value 값을 1260에서 2400으로 제한합니다.

  float vOut = (value * REFERENCE_VOLTAGE) / ANALOGIC_RESOLUTION; // Calculate output voltage with rule of three formula
  float vIn = vOut / (R2 / (R1 + R2)) * 1.10; // Calculate input voltage with voltage divider formula
  int percent = vIn * 100 / EXPECTED_V_OUT; // Calculating baterry level percentage with rule of three formula

  //map display2400

  if (value >= 2320) percent = 20;//100->2314

  else if (value >= 2200)    // 90-100% range
    percent = map(value, 2200, 2320, 18, 20);//2200 2314
  else if (value >= 1970)    // 10-90% range
    percent = map(value, 1970, 2200, 2, 18);//2050 2200
  else if (value >= 1260)    // 0-10% range
    percent = map(value, 1260, 1970, 0, 2);// 1260  2050
  else percent = 0;

  Serial.print("Baterry value is: ");
  Serial.println(value);
  Serial.print("Baterry level is: ");
  Serial.print(vIn);
  Serial.print("V (");
  Serial.print(percent * 5); // 5 step   100%일때 20이므로
  Serial.println("%);\n");

  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(14, 120);      // Start at top-left corner
  tft.setTextColor(ST7735_BLACK);  // Draw white text
  tft.printf("%4.2f", bat_old_digit); // 4자리를 확보하고 왼쪽 마춤
  tft.setCursor(14, 120);      // Start at top-left corner
  tft.setTextColor(ST7735_WHITE);  // Draw white text
  tft.printf("%4.2f", vIn); // 4자리를 확보하고 왼쪽 마춤
  tft.print("  V");
  bat_old_digit = vIn;


  int imsi = percent * 5; // = bat
  if (!change_bat)change_bat = imsi;// just display
  if (change_bat != imsi) { //현재값이 토글 현상이 발생하면  대피하고 다음에 같으면  값 반영
    change_bat = imsi;//percent:현재측정 값
  }
  else {
    bat = imsi;
    batLevel(percent);
  }
}

void timer_isr()
{
  tmr++;
  // if (++sec_tmr == 1000)
  if (++sec_tmr == 100)//10ms*100=1 초
  {
    sec_tmr = 0;

    current_sec++;
    cali_exe_flag = 0; //??? 1초 마다 지움

    if (power_key.pressed) { // power_key isr 루틴에서 set

      power_key.numberKeyPresses++;// for tjio power key


      if (power_key.numberKeyPresses > 2) { //power off
        // Serial.print("Power key Press sec");
        mtag = 1; //tjio
        digitalWrite(ESP_OUT, 0);
        digitalWrite(tft_on, LOW);
        init_tft();
        alarmsound();
        delay(50);
        alarmsound();
        while (1); // off 할때까지 기다림    물론 인터럽트는 가능
      }
    }

    if (zero_key.pressed) { // zero sw 루틴에서 set

      zero_key.numberKeyPresses++;// for tjio zero key


      if (zero_key.numberKeyPresses > 5) { //zero key 5초 이상이면
        Serial.println("Calibration start");
        //zero_key.pressed = 0;
        alarmsound();
        //      delay(1);
        alarmsound();
        cali_set = 1;

        prev_sec = current_sec;//calibration  시간을 연장 하기위여  current_sec 저장
        zero_key.numberKeyPresses = 0; // COUNT 0 set
      }
    }
    //Serial.print("zero_key.numberKeyPresses : ");
    //Serial.println(zero_key.numberKeyPresses);
    if ((digitalRead(zero_key.PIN) != 0)) {//zero sw 해제   loop rtn 에서 빨리 스위치 상태를 얻기 위해 옮김
      zero = 0;
      zero_key.pressed = 0;

    }

  }//1 sec end

}

#define EEPROM_SIZE 12
void setup() {
  Serial.begin(115200);
  Serial.println("INIT");

  EEPROM.begin(EEPROM_SIZE);

  //Write data into eeprom
  int address = 0;


  //Read data from eeprom
  address = 0;
  int readId;
  readId = EEPROM.read(address); //EEPROM.get(address,readId);
  Serial.print("Read Id = ");
  Serial.println(readId);
  address += sizeof(readId); //update address value

  float readParam;
  EEPROM.get(address, readParam); // 인덱스 address 에서 자료형 크기 4바이트 만큼 읽고 변수에 저장
  Serial.print("Read param = ");
  Serial.println(readParam);

  // 초기 설정 범위 확인
  if ((readParam < 100.0) && (readParam > -100)) {
    adjust_value = readParam;
  }
  else adjust_value = 0;

  cali_factor = 600 + adjust_value;

  // EEPROM.end();
  /*
     pinMode(ledPin, OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
  */
  pinMode(power_key.PIN, INPUT_PULLUP);
  // attachInterrupt(power_key.PIN, isr1, FALLING);
  attachInterrupt(digitalPinToInterrupt(power_key.PIN), isr1, CHANGE);
  pinMode(zero_key.PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zero_key.PIN), zero_sw, FALLING);//digitalPinToInterrupt(zero_key.PIN)


  pinMode(boot, INPUT_PULLUP);
  pinMode(ESP_OUT, OUTPUT);

  digitalWrite(ESP_OUT, 1);//power on
  timer.attach(0.01, timer_isr);

  ledcAttachPin(led_power , 1);
  ledcSetup(1, 5000, 8);

  ledcAttachPin(tft_on, 2);
  ledcSetup(2, 5000, 8);

  ledcWrite(1, 85);
  delay(10);
  ledcWrite(2 , 85);
  delay(10);

  pinMode(35 , INPUT_PULLUP);

  // digitalWrite(tft_on, LOW);
  ledcSetup(0, 1E5, 12); //PWM채널,주파수,분해능
  ledcAttachPin(5, 0); //포트번호,PWM채널
  alarmsound();

  init_tft();
  display_main();
  //로드셀 초기화
  scale.begin(DOUT, CLK); //로드셀 시작

  zero_set();


  connectToWiFi();
  read_sn();

  //  tft.setFont(&FreeSans9pt7b);
  //  tft.setTextColor(ST7735_GREEN);
  //  tft.setCursor(FIRST_X + 5, FIRST_Y + 40);
  //  tft.printf("Connect");

  //  Serial.println("WIFI CONNECT");

  bat_measure();
}//setup end

//tjio rtn modify
void calibration_adjust() {//loop 에서 검사 하기때문에    시간지연 발생
  if (current_sec - prev_sec > 10) { //10초이상이면 calibration 종료
    cali_set = 0;
    // 한번만 수행 하여 함
    //Write data into eeprom
    int address = 0;
    int boardId = 18;
    EEPROM.write(address, boardId);//EEPROM.put(address, boardId);
    address += sizeof(boardId); //update address value
    float param = adjust_value;
    EEPROM.writeFloat(address, param);//EEPROM.put(address, param);
    EEPROM.commit();
    Serial.println("Write complete");
    //EEPROM.end();
  }
  cali_factor = 600 + adjust_value;//default =600+변화값 을 표시 합니다.
  Serial.print("Calibration value:");
  Serial.println(cali_factor);
  delay(5);
}




void connectToWiFi() {
  Serial.println("Wi-Fi 연결을 시도합니다...");

  WiFi.begin(ssid, password);
  unsigned long check_time = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if ((millis() - check_time) >= 5000) break;
  }
  if (WiFi.status() == WL_CONNECTED) {//5초 이상이어도 연결되지 않은지
    Serial.println("Wi-Fi 연결 성공!");
    Serial.print("IP 주소: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(ST7735_GREEN);
    tft.setCursor(FIRST_X + 5, FIRST_Y + 40);
    tft.printf("Connect");
  }

}
float moveAverage(float *arr, float inData, int len) {
  int i;
  float sum = 0;
  for (i = 0; i < (len - 1); i++) arr[i] = arr[i + 1];
  arr[len - 1] = inData;
  for (i = 0; i < len; i++) sum += arr[i];
  return float(sum / len);
}
float crtWeight = 0;
void loop()
{
  if (!mtag) {//power off 시 진행 방지


    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi 연결이 끊어졌습니다. 다시 연결을 시도합니다...");

      tft.setFont(&FreeSans9pt7b);
      tft.setTextColor(ST7735_BLACK);
      tft.setCursor(FIRST_X + 5, FIRST_Y + 40);
      tft.printf("Connect");// 검정색으로 다시 쓴다.

      connectToWiFi();

    }


    static unsigned long ckeck_weight = millis();
    static int count = 0;

    if ((millis() - ckeck_weight) >= 200 ) { // 2초 마다 체크
      ckeck_weight = millis();
      if (count++ < 10) {
        ckeck_weight = millis();
        //set_scale로 설정된 SCALE 매개변수에 의해
        //ADC에서 용기 무게를 뺀(1)개 판독값의 평균을 나눈 값을 출력합니다.
        //       ave_value =  scale.get_units(1);
        crtWeight = scale.read_average(5);
        Serial.print("crtWeight:");
        Serial.println(crtWeight);
        ave_value = (crtWeight - setup_offset) / LCELL_DIVIDER;

        Serial.print("setup_offset : \t");
        Serial.println(setup_offset);

        ave_value = moveAverage(fweight, ave_value, 10);
        //        Serial.print("이동 평균 경과 시간:\t");
        //        Serial.print((millis() - ckeck_weight));
        //        Serial.println("입니다");
        Serial.print("ave_value:");
        Serial.println(ave_value);
      }
      else {
        count = 0;
        for (int i = 0; i < 10; i++) {
          printf("10개 저장후 fweight[%d] : %8.2f\n", i, fweight[i]);
        }

      }
    }


    //    Serial.print("fWeighing Measure:\t");
    //    Serial.println(fave_value);

    print_data();

    static unsigned long send_prev_time  = millis();
    if (millis() - send_prev_time >= 15000) { // 15초
      send_data();
      Serial.print("전송주기 :\t");
      Serial.println(millis() - send_prev_time);
      send_prev_time = millis();
    }

    static unsigned long prev_btime = millis();
    if ((millis() - prev_btime) >= 60000) { // 60 초 마다 배터리 체크(시간지연으로 동작 오류제거)
      prev_btime = millis();
      bat_measure();
    }

    if (zero_key.pressed && (!zero) && (!cali_set)) {// zero는  cali adjust 일때까지 기다림
      zero_set();
      alarmsound();
      delay(10);
      zero = 1;
    }
    /*
        if ((digitalRead(zero_key.PIN) != 0)) {//zero sw 해제
          zero = 0;
          zero_key.pressed = 0;

        }
    */
    if (cali_set) {

      calibration_adjust();

    }
    Serial.print("Calibration set:\t");
    Serial.println(cali_set);
      Serial.print("Zero set exec:\t");
  Serial.println(zero_exe);
  }// mtag end
}
