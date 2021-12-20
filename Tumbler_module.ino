#include <Arduino_LSM9DS1.h>              // IMU 센서
#include <ArduinoBLE.h>                   // BLE 모듈
#include <HX711.h>                        // 로드셀 센서

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;
//float calibration_factor = -4815;                                    // 캘리브레이션 인자 수정 필요함  // v2: -4627   v1 : -4815
float calibration_factor = -4630; 

#define WEIGHT_OF_EMPTY_MODULE 30                                    // 텀블러 부착 모듈에 텀블러가 부착되어 있지 않은 상태일 때 센서에서 읽게 되는 값 (텀블러 제작 후 수정)
#define MAXIMUM_WEIGHT 4000                                          // 센서에서 읽을 수 있는 가장 큰 값
#define ARRAY_LEN 1000                                                // 배열의 길이
#define DEBUG 0                                                      // 0 으로 설정할 경우 Release mode
#define LED_PWR 25


BLEService TumblerModuleService("1101");
BLEIntCharacteristic DataLengthChar("2101", BLERead);                 // 텀블러 모듈에 저장되어 있는 데이터 길이 Characteristic
BLEBoolCharacteristic NotifyReadyChar("2102",BLERead|BLEWrite);       // 어플리케이션에서 false가 아닌 값을 write한 이후 수분 섭취량과 시간 간격 데이터 전송 시작
BLEIntCharacteristic WaterIntakeDataChar("2104",BLERead|BLENotify);   // 수분 섭취량 Characteristic  (mL 단위의 값 전달)
BLEIntCharacteristic MinuteIntervalChar("2108",BLERead|BLENotify);    // 시간 간격 Characteristic (분 단위의 값 전달)
                                      // BLERead : 중앙장치(IOS)가 주변장치(아두이노)에서 데이터를 읽을 수 있는 권한
                                      // BLEWrite : 중앙 장치가 주변장치에 데이터를 쓸 수 있는 권한
                                      // BLENotify : 특성이 변경될 경우, 중앙장치는 알림을 받을 수 있음.

byte isNotifyReady = false; // IOS 단말의 데이터 수신 준비 여부를 나타내는 값
int loadcell_w = 0;         // 로드셀 센싱값 저장

int waterdata[ARRAY_LEN];   // 수분량을 저장하는 배열

int minutedata[ARRAY_LEN];  // 시간(분)을 저장하는 배열

int trans_waterdata[ARRAY_LEN];     // 앱으로 전송할 수분 섭취량 데이터를 저장한 배열

int trans_minutedata[ARRAY_LEN];    // 앱으로 전송할 시간 간격 데이터를 저장한 배열(0으로 초기화)

int tumblerstatus = 0;            // 0: 움직이고 있는 상태, 1: 바닥에 놓여있는 상태(이때 압력 센서 값을 읽어야함)
int datalength = 0;               // 시간, 섭취량 저장 데이터 길이
int transmission_datalength = 0;  // 전송할 데이터의 길이 (0일때에는 전송할 데이터 없음)
int delay_cnt = 5;                // 텀블러가 부착 되어있지 않은 경우 값이 10보다 큰값으로 변경됨 
                                  //          -> BLE 연결에서 더 오랜 시간 있게 됨



                                  
void setup() {                                                              /*         SETUP             */
  Serial.begin(9600); 
  // IMU 센서 실행
  startIMU();
  // BLE 모듈 초기화 및 실행 
  startBLE();
  // BLE 서비스 & 특성 advertise
  configureBLE();
  // 로드셀 센서 설정 및 실행 
  startLoadcell();
  // 배열 초기화
  reset_array();

  // POWER LED 끔
  digitalWrite(LED_PWR, LOW);

}
    
void loop() {                                                                /*             LOOP            */
  scale.set_scale(calibration_factor);            
    /* 가속도 센서 확인 -> 텀블러 움직임 여부 파악 */  
  tumblerstatus = gettumblerstatus();             // 함수 내부에서 로드셀 데이터 읽고 전역변수에 저장

  if(tumblerstatus==1){
    // 텀블러가 움직이지 않는 상태
    digitalWrite(LED_PWR, HIGH);
    delay(500);
    loadcell_w = scale.get_units(2)*10;            // 구간별 캘리브레이션 필요
    Serial.print("Loadcell: ");
    Serial.println(loadcell_w);
    digitalWrite(LED_PWR, LOW);

    
    /* 로드셀 센싱값으로 텀블러의 무게 추정*/
    if(loadcell_w > WEIGHT_OF_EMPTY_MODULE && loadcell_w < MAXIMUM_WEIGHT){          // 상한선 설정 필요
      // 텀블러가 부착되어 있는 경우
      delay_cnt = 5;
      if(datalength == 0){
        // 텀블러가 움직이지 않은 상태 중 가장 첫번째
//        minutedata[0] = minute(time(NULL));               // 오버플로우 문제 생기므로 사용 X
        minutedata[0] = millis()/(unsigned long)60000;      // Int= 4Byte =32bit                                                // unsigned long = 4Byte = 32bit
                                                            // minutedata[] 배열에 저장할 수 있는 최댓값 = 2^16 -1 = 65535          // unsigned long으로 표현가능한 최댓값 = 2^32 -1 = 4294967295
                                                            // 65535분 = 1092.25 시간 = 45.51일                                 // 4294967295ms = 4294967.295s = 71582.788m = 1193.046h = 49.71day     
        waterdata[0] = loadcell_w;
        datalength++;
      }
      
      else{
        // 기존에 들어있는 물의 양과 현재 측정항 물의 양의 무게차이 확인
        int diff_water =  waterdata[datalength-1] - loadcell_w;              // diff_Water > 0 : 물이 추가됨
                                                                         //            < 0 : 물을 마시거나 버림
        if(abs(diff_water)>=10){
          // 텀블러 내의 음료가 10ml이상 추가 되거나 10ml이상 없어진 경우
          minutedata[datalength] = millis()/(unsigned long)60000;
          waterdata[datalength] = loadcell_w;
          datalength++;
          if(diff_water >= 10){
            // 10ml 이상 줄어든 경우 
            transmission_datalength++;
          }
        }
        else{
          // 10ml미만은 센서의 측정 오차로 취급
          Serial.println("센서 측정 오차 (수분량 변화 10 미만)\n");
        }

      }
      
    }
    else if(loadcell_w >= MAXIMUM_WEIGHT){
      // 로드셀 센서가 읽을 수 있는 값보다 더 큰 값을 읽은 경우 에러로 취급
      Serial.println("센서 측정 오류 (읽을 수 있는 최댓값보다 더 높은 값이 읽힘)\n");
    }
    else{
      // 텀블러가 부착되어 있지 않은 경우
      delay_cnt = 20;
      Serial.println("텀블러가 부착되어 있지 않음.\n");
      delay(5000);                                             // 추후 sleep mode로 변경
    }

  }  
  else if(tumblerstatus==-1){
    Serial.println("\nIMU 센서 사용할 수 없음.\n");
  }
  else {
    // 텀블러가 움직이는 상태
    delay_cnt = 8;
    Serial.println("\n텀블러가 움직이므로 로드셀 센서 값 읽을 수 없음\n");
  }


  // 디바이스와 연결 시도                                                                                    
  for(int i=0; i<delay_cnt;i++){
    BLEDevice central = BLE.central();
    if(central){
      // 정상적으로 연결된 경우 
      Serial.print("Connected to central: ");
      Serial.println(central.address());
  
      // 전송할 데이터 생성
      create_data();
  
      // 데이터 전송 
      transmission_data(central);
      
//      // 변수 및 데이터 초기화
//      clear_all();
    }
    delay(1000);
  }
  delay_cnt = 5;
    
  if(DEBUG){
    print_debug();
  }
    
  delay(1000);
   


}



void create_data(){                                                                     /*  CREATE DATA Function   */
  int trans_idx = transmission_datalength -1;
  int data_idx = datalength -1;
  
  // 전송할 데이터 배열 생성
  while(trans_idx >= 0){

    if(waterdata[data_idx] < waterdata[data_idx - 1]){
      // 물이 줄어든 경우
      trans_waterdata[trans_idx] = waterdata[data_idx-1] - waterdata[data_idx] - 1;
      trans_idx--;

      if(trans_idx >=0){
        trans_minutedata[trans_idx] += minutedata[data_idx] - minutedata[data_idx-1];
      }
    }
    else{
      // 물이 늘어난 경우 전송할 시간데이터 값만 수정
      trans_minutedata[trans_idx] += minutedata[data_idx] - minutedata[data_idx-1];
    }

    data_idx--;
    
  }

}


void transmission_data(BLEDevice central){                                               /*  DATA TRANSFER Function   */
  int idx = 0;                                      
  while(central.connected()){
    /*디버깅을 위해 BLE 연결된 경우 LED 켬*/
    if(DEBUG){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    
    // 전송할 때의 시간 저장
    trans_minutedata[transmission_datalength-1] = millis()/(unsigned long)60000 - minutedata[datalength-1];
    
    DataLengthChar.writeValue(transmission_datalength);    // IOS 장치에서 읽어가야할 데이터 길이 확인

    if(NotifyReadyChar.written()){        
      // IOS 장치에서 해당 Characteristic에 false가 아닌 값을 쓴 경우 
      NotifyReadyChar.readValue(isNotifyReady);      // IOS 장치가 쓴 값을 isNotifyReady 변수에 저장
      Serial.print(" NotifyReadyChar: ");
      Serial.println(isNotifyReady);
      
    }  

    if(isNotifyReady){
      // IOS 에서 수신할 준비가 되었음
      if(idx == transmission_datalength){
        delay(2000);    // IOS 장치에서 모든 데이터를 읽을 때까지  잠시 대기
        isNotifyReady = false;
        // 변수 및 데이터 초기화
        clear_all();
        
        if(central.disconnect()){
          // central.disconnect() == true  : BLE 장치와 연결 끊어진 경우
          Serial.print("Disconnect Successed to ");
          Serial.println(central.address()); 
        }
        else{
          Serial.println("Disconnect Failed");
        }
        
        
      }
      
      if(trans_minutedata[idx]>=0 && trans_waterdata[idx]>0){
        // 데이터가 -1이 아닐 때까지 전송
        MinuteIntervalChar.writeValue(trans_minutedata[idx]);
        delay(30);
        WaterIntakeDataChar.writeValue(trans_waterdata[idx]);
        delay(30);

        
        // 전송 디버깅
        // 시리얼 모니터 확인용
        Serial.print(" BLE 전송: water-");
        Serial.print(trans_waterdata[idx]);
        Serial.print("\t minutes-");
        Serial.print(trans_minutedata[idx]);
        Serial.println('\t');
        idx++;

      }
      
    }
  }
  if(DEBUG){
    digitalWrite(LED_BUILTIN, LOW);
  }
  
}

void clear_all(){                                                           /*  CLEAR ALL Function   */
  
  for(int i = 0; i<datalength; i++){
    trans_waterdata[i] =-1;
    trans_minutedata[i] = 0;  
    waterdata[i]=-1;
    minutedata[i] =-1;
  }
  
  datalength = 0;
  tumblerstatus = 0;  
  transmission_datalength = 0;

}

int gettumblerstatus(){                                                     /*  GET TUMBLER STATUS Function   */
  // 텀블러의 상태 파악 함수 (IMU 센서 값 이용)
  // 텀블러가 바닥에 놓인 상태 = 1, 움직이는 상태 = 0 , 센서 에러 = -1
  float g_x=-10.0, g_y=10.0, g_z=10.0;            // 자이로 센서 값 저장
  float a_x=-10.0, a_y=10.0, a_z=10.0;            // 가속도 센서 값 저장
  if(IMU.gyroscopeAvailable()){
    IMU.readGyroscope(g_x,g_y,g_z);
    
    
    if(DEBUG){
    // 시리얼 모니터 확인용
      Serial.print(" G : ");
      Serial.print(g_x);
      Serial.print('\t');
      Serial.print(g_y);
      Serial.print('\t');
      Serial.println(g_z);  
    }
  
    
    if(fabs(g_x) < 5.0 && fabs(g_y) < 5.0){
      // 현재 텀블러에 움직임이 없는 상태
      // 텀블러가 정상적으로 놓여있는지 확인
      // 자이로 센서로 움직임을 파악
      if(IMU.accelerationAvailable()){
        IMU.readAcceleration(a_x, a_y, a_z);
        if(DEBUG){
          // 시리얼 모니터 확인용
          Serial.print(" A : ");
          Serial.print(a_x);
          Serial.print('\t');
          Serial.print(a_y);
          Serial.print('\t');
          Serial.println(a_z);
        }
        
        if(fabs(a_x)<0.1 && fabs(a_y)<0.1 && a_z>0.9){
          // 가속도 센서로 텀블러의 자세 파악
          // 텀블러가 반듯하게 놓여있는 상태
          // a_z ~= +0.98 일때 nano 보드가 위를 향함, a_z ~= -1.03일 때  nano 보드가 아래를 향함
          /* 보드를 넣는 뱡향에 따라 a_z 임계값 수정 필요*/
          return 1;           // 반듯하게 놓여있는 상태
        }
        else{
          // 텀블러가 누워있거나 기울어져있는 상태
          Serial.println("\n텀블러가 누워있거나 기울어져있음.\n");
          return 0;      
        }
      }
      else{
        // IMU 센서의 가속도계 값을 사용할 수 없는 상태
        Serial.println("\n가속도 센서 사용할 수 없음.\n");
        return -1;
      }
    }
    else{
      // 텀블러가 움직이고 있는 상태
      Serial.println("\n텀블러가 움직이고 있음.\n");
      return 0;
    }
  }
  else{
    // IMU 센서의 자이로 값을 사용할 수 없는 상태
    Serial.println("\nIMU 센서를 사용할 수 없음.\n");
    return -1;
  }
}

void print_debug(){                                                                 /*   PRINT DEBUG Function   */
    // 디버깅 출력
  Serial.print("Water data[] : \n");
  for(int i=0; i<datalength; i++){
    if(waterdata[i]>0){
      Serial.print(waterdata[i]);
      Serial.print(", ");
      if(i%20 == 19){
        Serial.print("\n");
      }
      
    }
      
  }
  Serial.print("\nminutedata[] : \n");
  for(int i=0; i<datalength; i++){
    if(minutedata[i]>=0){
      Serial.print(minutedata[i]);
      Serial.print(", ");
      if(i%20 == 19){
        Serial.print("\n");
      }
    }
 
  }
//  Serial.print("Transmission Water data[] : ");
//  for(int i=0; i<100; i++){
//    Serial.print(trans_waterdata[i]);
//    Serial.print(" ");
//    if(i%20 == 19){
//      Serial.print("\n");
//    }
//  }
//  Serial.print("\nTransmission minutedata[] : ");
//  for(int i=0; i<100; i++){
//    Serial.print(trans_minutedata[i]);
//    Serial.print(" ");
//    if(i%20 == 19){
//      Serial.print("\n");
//    }
//  }

  Serial.print("\nData Length: ");
  Serial.println(datalength);
  Serial.print("\nData Length to transmit: ");
  Serial.println(transmission_datalength);
  Serial.print("\n");
}





void startLoadcell(){                                                        /*  START LOAD CELL Function   */
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(5000);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  
}

void startBLE(){                                                             /*  START BLE Function   */
  if (!BLE.begin()) 
  // BLE 사용이 불가능한 경우
  {
    Serial.println("Failed to start BLE");
    while (1);
  }
}

void configureBLE(){                                                         /*  CONFIGUREBLE Function   */
  // 아두이노 BLE 모듈의 이름 설정
  BLE.setLocalName("[Dysson]Tumbler Module2");
  BLE.setDeviceName("[Dysson]Tumbler Module2");
  // 서비스 설정
  BLE.setAdvertisedService(TumblerModuleService);
  // 특성 추가
  TumblerModuleService.addCharacteristic(DataLengthChar);
  TumblerModuleService.addCharacteristic(WaterIntakeDataChar);
  TumblerModuleService.addCharacteristic(MinuteIntervalChar);
  TumblerModuleService.addCharacteristic(NotifyReadyChar);
  // 서비스 추가
  BLE.addService(TumblerModuleService);
//  BLE.setAdvertisingInterval(320) 320*0.625ms = 200ms 간격
  // BLE 장치 Advertise
  BLE.advertise();
}

void startIMU(){                                                             /*  START IMU Function   */
  if(!IMU.begin()){
  // IMU 센서 사용이 불가능한 경우    
    Serial.println("Failed to Initialize IMU");
    while(1);
  }
  
  // 가속도 센서
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  // 자이로 센서
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void reset_array(){                                                           /*  RESET ARRAY Function  */

  for(int j=0; j<ARRAY_LEN; j++){
    waterdata[j] = -1;   // 수분량을 저장하는 배열
    minutedata[j] = -1;  // 시간(분)을 저장하는 배열
    trans_waterdata[j] = -1;     // 앱으로 전송할 수분 섭취량 데이터를 저장한 배열     
  }

}