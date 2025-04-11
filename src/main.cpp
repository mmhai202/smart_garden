#include <Arduino.h>
#include <string.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <FirebaseESP32.h>

/* Khởi tạo ngoại vi */
#define DHTPIN  25 // Khởi tạo cảm biến DHT
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
uint8_t t = 0;
uint8_t h = 0;
const uint8_t asPin = 32; // Chân cảm biến ánh sáng
uint8_t as = 0;
const uint8_t adPin = 33; // Chân cảm biến ẩm đất
uint8_t ad = 0;

const uint8_t bomPin = 19;
const uint8_t fanPin = 18;
const uint8_t bombut = 27;
const uint8_t fanbut = 26;
uint8_t bomState = 0;
uint8_t fanState = 0;
uint8_t fanAuto = 0;
uint8_t curbomstate = 0;
uint8_t curfanstate = 0;
uint8_t setState = 0;
uint8_t setValue = 0;
bool timerState = 0;

/* set up LCD */
uint8_t lcdColumns = 16;
uint8_t lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
byte tempIcon[8] = {0b01110,0b01010,0b01010,0b01110,0b01110,0b01110,0b01110,0b01110};
byte humidityIcon[8] = {0b00100,0b00100,0b01010,0b01010,0b10001,0b10001,0b01110,0b00000};
byte lightIcon[8] = {0b00000,0b00100,0b10101,0b01110,0b10101,0b00100,0b00000,0b00000};
byte soilMoistureIcon[8] = {0b00100,0b00100,0b01110,0b01110,0b11111,0b11111,0b01110,0b00000};
byte degreeC[8] = {0b01110,0b01010,0b01110,0b00000,0b00000,0b00000,0b00000,0b00000};

void printLCD(int t, int h, int as, int ad) {
    lcd.setCursor(0, 0); 
    lcd.write(byte(0));
    lcd.print(" ");
    lcd.print(t);
    lcd.write(byte(4));
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.write(byte(1));
    lcd.print(" ");
    lcd.print(h);
    lcd.print(" %");

    lcd.setCursor(10, 0);
    lcd.write(byte(2));
    lcd.print("   ");
    lcd.setCursor(12, 0);
    lcd.print(as);
    lcd.setCursor(15, 0);
    lcd.print("%");

    lcd.setCursor(10, 1);
    lcd.write(byte(3));
    lcd.print("   ");
    lcd.setCursor(12, 1);
    lcd.print(ad);
    lcd.setCursor(15, 1);
    lcd.print("%");
}

void setup_LCD() {
    lcd.init(); // bắt đầu chạy lcd
    lcd.backlight();
    lcd.clear();
    lcd.createChar(0, tempIcon);
    lcd.createChar(1, humidityIcon);
    lcd.createChar(2, lightIcon);
    lcd.createChar(3, soilMoistureIcon);
    lcd.createChar(4, degreeC);
}

/* set up timer */
hw_timer_t* timer0 = NULL; // Khởi tạo timer
void IRAM_ATTR onTimer0() { // hàm ngắt timer
    curbomstate = HIGH;
}

/* set up wifi */
const char* ssid = "Hai";
const char* password = "11112222";
void setup_wifi() {
    Serial.printf("Connecting...");
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED) {
        Serial.printf(".");
        delay(500);
    };
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

/* Xử lý API thời gian */
String timeUrl = "http://worldtimeapi.org/api/timezone/Asia/Ho_Chi_Minh";
String timeJsonBuffer;
String httpGETRequest(String Url) { // Hàm GET request để lấy API
    HTTPClient http;
    http.begin(Url);
    int responseCode = http.GET();
    String responseBody = "{}";
    if(responseCode > 0) responseBody = http.getString();
    http.end();
    return responseBody;
}
String preAPI;
String readAPI() {
    int retryCount = 0;             // Số lần thử lại
    const int maxRetries = 10;     // Giới hạn số lần thử lại
    while (retryCount < maxRetries) {
        // Gửi yêu cầu GET đến API
        timeJsonBuffer = httpGETRequest(timeUrl);
        // Kiểm tra nếu phản hồi trống hoặc không phải JSON
        if (timeJsonBuffer == "{}" || timeJsonBuffer.length() == 0) {
            retryCount++;
            continue;
        }
        // Phân tích chuỗi JSON
        JSONVar timeJson = JSON.parse(timeJsonBuffer);
        // Kiểm tra xem phản hồi có chứa thuộc tính `datetime` không
        if (JSON.typeof(timeJson) == "undefined" || !timeJson.hasOwnProperty("datetime")) {
            retryCount++;
            continue;
        }
        // Lấy giá trị của `datetime` từ JSON
        String datetime = timeJson["datetime"];
        // Kiểm tra độ dài và định dạng ngày giờ
        if (datetime.length() >= 19) {
            datetime = datetime.substring(0, 19); // Lấy chuỗi theo định dạng "YYYY-MM-DDTHH:MM:SS"
            Serial.println(datetime);
            return datetime;
        }
        // Tăng bộ đếm và thử lại
        retryCount++;
    }
    // Nếu vượt quá số lần thử, trả về giá trị mặc định
    Serial.println("Failed to retrieve valid datetime after maximum retries.");
    return preAPI;
}

/* set up và xử lý Firebase */
#define FIREBASE_HOST "smart-garden-5caaa-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "50P4sp9NtgwwOUwJZYlKvxFgGPJHzS3ebKjP3J1B"

FirebaseData firebaseData; // Khởi tạo Firebase Data
FirebaseData firebaseStream; // Dữ liệu cho stream
FirebaseJson json;
FirebaseJsonArray sensor_data;
FirebaseJsonArray watering_history;
FirebaseJsonData result;

void streamCallback(StreamData data) { // Callback cho stream để lắng nghe thay đổi từ Firebase
    String path = data.dataPath();
    int value = data.intData();
    if (path == "/bomState") {
        curbomstate = value;
        Serial.printf("bomstate_firebase: ");
        Serial.println(curbomstate);
    } else if (path == "/fanState") {
        curfanstate = value;
        Serial.printf("fanstate_firebase: ");
        Serial.println(curfanstate);
    } else if (path == "/setState") {
        setState = value;
        Serial.printf("setstate_firebase: ");
        Serial.println(setState);
        if (Firebase.getInt(firebaseData, "/setValue")) {
            setValue = firebaseData.to<int>();
            Serial.printf("setvalue_firebase: ");
            Serial.println(setValue);
        }
    }
}

// Callback khi stream gặp sự cố hoặc hết thời gian chờ
void streamTimeoutCallback(bool timeout) {
    if (timeout) Serial.println("Stream timeout, trying to reconnect...");
}

void initFirebase() {
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);
    Firebase.RTDB.getArray(&firebaseData, "/sensor_data", &sensor_data);
    Firebase.RTDB.getArray(&firebaseData, "/watering_history", &watering_history);
    Serial.println("Firebase connected");

    // Bắt đầu stream từ đường dẫn Firebase
    if (Firebase.beginStream(firebaseStream, "/state")) {
        Firebase.setStreamCallback(firebaseStream, streamCallback, streamTimeoutCallback);
        Serial.println("Stream started successfully");
    } else Serial.println("Failed to start stream");
}

void saveSensorData(int t, int h, int as, int ad, String timestamp) {
    // Khởi tạo dữ liệu mới
    FirebaseJson newData;
    newData.set("t", t);
    newData.set("h", h);
    newData.set("as", as);
    newData.set("ad", ad);
    newData.set("time", timestamp);

    // Thêm dữ liệu vào mảng
    sensor_data.add(newData);
    if (sensor_data.size() > 20) sensor_data.remove(0);
 
    // Gửi dữ liệu lên Firebase
    Firebase.RTDB.setArray(&firebaseData, "/sensor_data", &sensor_data); 
}

void saveWateringHistory(String timestamp) {
    // Khởi tạo dữ liệu mới
    FirebaseJson newData;
    newData.set("time", timestamp);

    // Thêm dữ liệu vào mảng
    watering_history.add(newData);
    if (watering_history.size() > 20) watering_history.remove(0);
 
    // Gửi dữ liệu lên Firebase
    Firebase.RTDB.setArray(&firebaseData, "/watering_history", &watering_history); 
}

uint8_t fanbutPressed = 0;
/* Hàm xử lý ngắt */ 
void IRAM_ATTR fanbutPush() {
    fanbutPressed++;
}

void IRAM_ATTR bombutPush() {
    curbomstate = HIGH;
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    setup_LCD();
    pinMode(bomPin, OUTPUT);
    pinMode(fanPin, OUTPUT);
    pinMode(bombut, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(bombut), bombutPush, FALLING);
    pinMode(fanbut, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(fanbut), fanbutPush, FALLING);
    setup_wifi();
    initFirebase();
}

unsigned long pre1 = 0;
const long delay1 = 2000;

unsigned long pre2 = 0;
const long delay2 = 60000;

unsigned long pre3 = 0;
const long delay3 = 1000;

void loop() {
    // Kiểm tra bơm
    if (fanbutPressed > 0) {
        curfanstate = !curfanstate;
        fanbutPressed = 0;
    }

    if(bomState != curbomstate) {
        bomState = curbomstate;
        Serial.printf("bomstate_esp: ");
        Serial.println(bomState);
        pre3 = millis();
        Firebase.setInt(firebaseData, "/state/bomState", bomState);
    }

    // Kiểm tra quạt
    if(fanState != curfanstate) {
        fanState = curfanstate;
        Serial.printf("fanstate_esp: ");
        Serial.println(fanState);
        digitalWrite(fanPin, fanState);
        Firebase.setInt(firebaseData, "/state/fanState", fanState);
    }

    // Điều khiển bơm
    if(as > 0 && ad < 10 && curbomstate == LOW) {
        curbomstate = HIGH;
        pre3 = millis();
        Serial.println("do am thap: ");
    }

    if(curbomstate == HIGH) { // Bơm trong khoảng delay3
        unsigned long cur3 = millis();
        digitalWrite(bomPin, HIGH);
        if(cur3 - pre3 >= delay3) {
            digitalWrite(bomPin, LOW);
            curbomstate = LOW;
            lcd.clear();
            String time = readAPI();
            saveWateringHistory(time);
        }
    }

    // Điều khiển quạt tự động
    if(((t >= 30) || (h >= 75)) && fanAuto == LOW && curfanstate == LOW) {
        fanAuto = HIGH;
        digitalWrite(fanPin, HIGH);
        Firebase.setInt(firebaseData, "/state/fanAuto", fanAuto);
        Serial.println("bat quat");
    } else if((t < 30 && h < 75) && fanAuto == HIGH && curfanstate == LOW) {
        fanAuto = LOW;
        digitalWrite(fanPin, LOW);
        Firebase.setInt(firebaseData, "/state/fanAuto", fanAuto);
        Serial.println("tat quat");
    }

    // Mỗi delay1 giây đọc cảm biến 1 lần
    unsigned long cur1 = millis();
    if(cur1 - pre1 >= delay1) {
        int t1 = dht.readTemperature();
        int h1 = dht.readHumidity();
        if (t1 > 0 && t1 < 100) t = t1;
        if (h1 > 0 && h1 < 100) h = h1;
        as = 100 - (float)analogRead(asPin)/4095*100;
        ad = 100 - (float)analogRead(adPin)/4095*100;
        Firebase.setInt(firebaseData, "/sensor/t", t);
        Firebase.setInt(firebaseData, "/sensor/h", h);
        Firebase.setInt(firebaseData, "/sensor/as", as);
        Firebase.setInt(firebaseData, "/sensor/ad", ad);
        printLCD(t, h, as, ad);
        pre1 = cur1;
    }

    // Mỗi delay2 giây lưu dữ liệu 1 lần
    unsigned long cur2 = millis();
    if(cur2 - pre2 >= delay2) {
        Serial.printf("saved data: ");
        String time = readAPI();
        saveSensorData(t, h, as, ad, time);
        pre2 = cur2;
    }

    // Tưới tự động
    if(setState == HIGH) {
        // Nếu timer chưa được khởi tạo, thực hiện khởi tạo timer
        if(timerState == 0) {
            long value = setValue * 1000000; // Tính thời gian ngắt (microseconds)
            timer0 = timerBegin(0, 80, true); // Timer 0, chia tần số 80 (1 MHz), đếm tăng
            timerAttachInterrupt(timer0, &onTimer0, true); // Đăng ký hàm callback
            timerAlarmWrite(timer0, value, true); // Đặt thời gian ngắt, Auto-reload mode
            timerAlarmEnable(timer0); // Bật timer
            timerState = 1;
        }
    } else if(setState == LOW && timerState == 1) {
        timerAlarmDisable(timer0); // Tắt timer
        timerStop(timer0);         // Dừng bộ đếm timer
        timerWrite(timer0, 0);     // Reset bộ đếm về 0
        timerDetachInterrupt(timer0);
        timerEnd(timer0);
        timerState = 0;
    }
}
