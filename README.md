<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# capstone_react_web
4학년 1학기 리액트 웹 페이지 구현부 로직 저장소입니다.
=======
# arduino
>>>>>>> 2bc636c (Initial commit)
=======
2025 졸작 아두이노 주요 기능 요약 
=======
<h2><span style="color: skyblue">2025 졸작 아두이노 주요 기능 요약</span></h2>
>>>>>>> 2a868ae (README.md)
=======
2025 졸작 아두이노 주요 기능 요약 
>>>>>>> 6b50b72 (Update README.md)

### setup()

시리얼 통신 ==(Serial, BT_SERIAL)== 및 
센서 초기화 ==(MPU6050, 초음파, 조도 센서 설정)==

---
### loop()

1. 초음파 센서를 통해 거리 측정 후 BLE & 시리얼 출력
2. `WARNING` 값 출력 ==(거리 50cm 이하 시 1, 초과 시 0)==
3. 조도 센서를 통해 주변 밝기 `(LIGHT)` 측정 후 출력
4. 주변 광량이 일정 수준 이하`(LIGHT_THRESHOLD)`일 경우 LED 점등
5. 2초 간 광량 변화 (1 또는 0) 없을 시 LED 점등 및 소등 변경
   (어두워진다고 바로 시도 때도 없이 켜지는 것을 방지)
6. 500ms 주기로 자이로 센서 데이터 `(TILT, IMPACT) 분석 & 출력`
---
### sendSensorData(String key, float value)

JSON 형식 `{ 키 : 값 }` 쌍으로 센서 데이터를 
블루투스 및 시리얼 모니터에 출력 (자바 제네릭과 유사)

---
### anlayzeMPU6050(unsigned long currentTime)

1. 가속도 센서 통해 충격`(IMPACT)` 감지 ==(기준 값 초과 시 1, 아님 0)==
2. 자이로 센서를 통해 기울기`(TILT)` 감지 후 출력
3. 기울기가 30도 이상일 경우 `넘어짐 감지`, 
   30도 이하로 복귀 시 `정상 주행 중` 메세지 출력
<<<<<<< HEAD
4. 사고 감지 기능 `(45도 이상 각도 변화 && 2.5G 이상 충격 발생 시)`   
=======
>>>>>>> 6b50b72 (Update README.md)
---
### measureDistance()

1. 초음파 센서 이용해 후방 접근 물체와의 거리 측정 (최대 4m)
2. 4m 거리 초과 시 마지막 유효한 거리 값을 반환 
   (-1 또는 0 값으로 설정 시 경고 범위 이하에 들어가는 오류 방지)
---
### getAverageDistance()

- 초음파 센서를 5회 측정하고 평균 값을 산출해 반환 (노이즈 제거 목적)
---
<<<<<<< HEAD
>>>>>>> e8289ab (README.md)
=======

>>>>>>> 6b50b72 (Update README.md)
