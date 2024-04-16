
//https://github.com/espressif/arduino-esp32/releases/download/1.0.0/package_esp32_dev_index.json


#include <Ticker.h>
#include "HX711.h" //로드셀
//#include <EEPROM.h>

#define DOUT 4  // 로드셀 데이터핀
#define CLK 16   // 로드셀 클럭핀
#define ESP_OUT 15
Ticker timer;
HX711 scale;

float LCELL_INIT_VALUE = 223341.;  //기기별로 매우 다를 수 있으며, 공장값을 입력해 줘야 합니다.
float LCELL_DIVIDER = 596.;         //기기별로 약간의 차이가 있으며, 공장값을 입력해 줘야 합니다.

//////////////////////////////////////////////
void button_zero_click();
void timer_isr();
void gaugeMass();
int moveAverage();
//////////////////////////////////////////////

//////////////////////////////////////////////
float AUTO_OFFSET;
float avrGatt, Gatt, GRam, crtWeight, crtWeightDamp, preCrtWeight;
bool GRam250 = 0, injectEnd = 0;
unsigned long crtMilli, preMilli, DiffMilli;
bool DSPOn = 0, Transient = 0;
bool StateDOFSEI[6]; //fall Down, run Out, Fast; Slow, End, Inpect
//////////////////////////////////////////////

float ave_value = 0.0;
//////////////////////////////////////////////
int moveArr[10]; //충격에 의한 빠른 안정화를 위한 것으로 평균 투약 속도를 입력합니다.
void setup()
{
  Serial.begin(115200);
  Serial.println("Start.................................................\n");
  pinMode(25, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(25), button_zero_click, FALLING);

  pinMode(ESP_OUT, OUTPUT);
  digitalWrite(ESP_OUT, 1);//power on

  scale.begin(DOUT, CLK);

  delay(2000);
  //  for (int i=0; i < 10; i++) {
  //   moveArr[i] = 100;
  // }
  //  crtWeight = scale.read_average(100);
  //  ave_value = (crtWeight - 167207) / LCELL_DIVIDER;

  //  Serial.print("ave_value Gram : ");
  //  Serial.println(ave_value);
  //  Serial.print("crtWeight : ");
  //  Serial.println(crtWeight);
  scale.tare(10);//read_average(10);
  AUTO_OFFSET = scale.get_offset();
  Serial.print("AUTO_OFFSET : ");
  Serial.println(AUTO_OFFSET);

  GRam = 1.;
  Serial.println("Fast calibration...");
  AUTO_OFFSET = scale.read_average(50);

  while (GRam >= 0.15 ) { //빠른 0점을 잡습니다. 수액이 거치되어 있으면... //기울기 방향이 끝나는 점까지로 바꿀 것 ; 전값과 후 값의 곱이 -인 것으로 ...
    crtWeight = scale.read();
    Serial.print("1. crtWeight : ");
    Serial.println(crtWeight);

    GRam = (crtWeight - AUTO_OFFSET) / LCELL_DIVIDER;
    crtWeightDamp = crtWeight;

    crtWeight = scale.read();
    Serial.print("2. crtWeight : ");
    Serial.println(crtWeight);
    Serial.print("0.");
  
    if (abs(GRam) >= 5.0) {
      AUTO_OFFSET = scale.read_average(100);
      Serial.print("Retry AUTO OFFSET : ");
      Serial.println(AUTO_OFFSET);
    }
  }// while end
 

  Serial.print("\n");
  if ( AUTO_OFFSET > (LCELL_INIT_VALUE + LCELL_DIVIDER * 150.)) { //SAP 무게가 150g 이상이 걸리면 약물 바닥 알림을 준비합니다. // OFF_SET과는 무관합니다.
    GRam250 = 1;
    Serial.println("Set run out alarm.");
  }

  Serial.println("Reday...");

  preMilli = millis();
  preCrtWeight = crtWeight;

  timer.attach(0.2, timer_isr);
}

//////////////////////////////////////////////
void loop()
{

  // Alert, Alarm
  if ( crtWeight < (LCELL_INIT_VALUE - 14000.) )  StateDOFSEI[0] = 1; //I-Check has fallen.
  if ( ((LCELL_INIT_VALUE + (LCELL_DIVIDER * 100.)) > crtWeight) && (GRam250 == 1) ) StateDOFSEI[1] = 1; //IV fluid is running out.
  if ( (Gatt > 300) && (crtMilli > 60000 ) && (GRam250 == 1) )  StateDOFSEI[2] = 1; //Injection speed is too fast.
  if ((StateDOFSEI[2] == 1) && (Gatt < 300)) StateDOFSEI[2] = 0; //Injection speed is too fast.
  if ( (Gatt < 50) && (crtMilli > 60000 ) && (GRam250 == 1) && ((LCELL_INIT_VALUE + (LCELL_DIVIDER * 100.)) < crtWeight) ) StateDOFSEI[3] = 1; //Injection speed is too slow.
  if ((StateDOFSEI[3] == 1) &&  (Gatt > 50)) StateDOFSEI[3] = 0; //Injection speed is too slow.
  if (injectEnd) {
    StateDOFSEI[4] = 1;  //Injection has ended.
    Serial.println("투약이 종료되었습니다.");
  }
  if (Transient) StateDOFSEI[5] = 1; //It's a transient state.

  // Display
  if (DSPOn == 1) {
    avrGatt = (GRam / 0.05) / (crtMilli / (1000 * 60.)) * (-1);
    Serial.print("crtWeight ="); Serial.print(crtWeight); Serial.print("\t");
    Serial.print("Milli ="); Serial.print(crtMilli); Serial.print("\t");
    Serial.print("GRam ="); Serial.print(GRam * (-1), 2); Serial.print("\t");
    Serial.print("avrGatt ="); Serial.print( avrGatt ); Serial.print("\t");
    Serial.print("Gatt ="); Serial.print(Gatt); Serial.print("\t");
    Serial.print("GRam250 ="); Serial.print(GRam250);  Serial.print("\t");
    Serial.print("StateDOFSEI ="); Serial.print(StateDOFSEI[0]); Serial.print(StateDOFSEI[1]); Serial.print(StateDOFSEI[2]);
    Serial.print(StateDOFSEI[3]); Serial.print(StateDOFSEI[4]); Serial.print(StateDOFSEI[5]); Serial.print("\n");

    DSPOn = 0;
  }

}

//////////////////////////////////////////////
float dfWeight = 0., dfGRam = 0.;
int cntRead = 1, crtGatt, preGatt = 1;
//int moveArr[5] ={100,100,100,100,100}; //충격에 의한 빠른 안정화를 위한 것으로 평균 투약 속도를 입력합니다.
float preWeight_Transient = 0.;
float medi[5], mediCp[5];

//----------------------------------------------
void timer_isr() {
  gaugeMass();
}

//////////////////////////////////////////////
void gaugeMass() {
  // static unsigned long guage_time  = millis();
  // Serial.print("guage gap time :\t");
  // Serial.println(millis() - guage_time );
  // guage_time = millis();
  if (scale.is_ready() && injectEnd == 0) {
    crtWeight = scale.read();
    crtMilli = millis();

    //설정에서 기준값을 (damp)로 하고 이값과  현재 값을 차이를 배수(0.97) 하여  현재 값에 증감하여 사용
    //배수 가 적으면 현재값에 진동이크다
    crtWeightDamp = crtWeight + (0.97 * (crtWeightDamp - crtWeight));
    dfWeight = preCrtWeight - crtWeightDamp;
    // Serial.print("dfWeight="); Serial.print(dfWeight);Serial.print("\t");Serial.print("crtWeightDamp="); Serial.print(crtWeightDamp);Serial.print("\t");Serial.print("crtWeight="); Serial.print(crtWeight);  Serial.print("\t");  Serial.print("\n");

    // Serial.print("crtWeight ="); Serial.print(crtWeight);  Serial.print("\t");
    // Serial.print("crtWeightDamp0.97 ="); Serial.print(crtWeightDamp);  Serial.print("\t");
    // Serial.print("dfWeight ="); Serial.print(dfWeight);  Serial.print("\t");  Serial.print("\n");
    //0.1g 보다 크거나 5초 이상이면 cntRead=0
    if ( abs(dfWeight) >= 60. || cntRead > 25 ) { //abs(dfWeight)=0.1g   5초는 양방향으로 출렁이는 값을 고려한 것입니다.  5초
      //     Serial.print("Gatt :"); Serial.print(Gatt); Serial.print("\t");
      //     Serial.print("cntRead :"); Serial.println(cntRead); Serial.print("\n");
      DiffMilli = crtMilli - preMilli;
      GRam = (crtWeightDamp - AUTO_OFFSET) / LCELL_DIVIDER;

      crtGatt = int( ((dfWeight / LCELL_DIVIDER) / 0.05) / (DiffMilli / 60000.) ); //1분간의 무게변화
      Gatt = moveAverage(moveArr, crtGatt, 10);
      //      Serial.print("\n");
      //      Serial.print("Transient="); Serial.print(Transient);  Serial.print("\t");
      //      Serial.print("Gatt="); Serial.print(Gatt);  Serial.print("\t");
      //      Serial.print("DiffMilli/1000 ="); Serial.print(DiffMilli / 1000);  Serial.print("\t");  Serial.print("\n");

      // 정상 속도와 종료전 느린 속도를 구분하여 처리합니다.
      if (cntRead < 10) { //0.2 주기 기준으로 대기시간 2초(10) 이내에 무게 변화가 0.1g 이상 있으면 반영합니다.
        preCrtWeight = crtWeightDamp;
        preMilli = crtMilli;
        StateDOFSEI[5] = 0;
      }
      else if (DiffMilli / 1000. > (15 * 1 / 0.2) ) { //15초 동안 무게변화가 0.1 이상 변하지 않으면 crtGatt=0 으로 간주하고 측정을 종료합니다.
        Gatt = 0;
        injectEnd = 1;
      }
      //.........................................................................................

      if (Gatt < 350 && Gatt >= 0) DSPOn = 1;
      else {
        Transient = 1; Serial.print("s");  //안정화 과정 display를 제한합니다.

      }

      if (abs(preGatt - Gatt) > 10) Transient = 1;
      else Transient = 0; //Gatt 차가 10이상이면 불안정한 상태로 정보를 남깁니다.
      //.........................................................................................

      cntRead = 0;//dfWeight) >= 60. || cntRead > 25
      preGatt = Gatt;

    }

    cntRead++;
  }//measure

}//end

//////////////////////////////////////////////
void button_zero_click() {
  Serial.println(".................................................\n");
  delay(1000);
  ESP.restart();
}

////////////////////////////////////////////// 빠른 변화를 위해 Gatt 계산에서 median 사용 안함
int moveAverage(int *arr, int inData, int size) {
  int i, sum = 0;
  for (i = 0; i < (size - 1); i++) arr[i] = arr[i + 1];
  arr[size - 1] = inData;
  for (i = 0; i < size; i++) sum += arr[i];
  return int(sum / size);
}
