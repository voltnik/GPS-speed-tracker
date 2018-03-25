/*********************************************************************
HELLO WORLD! ANYBODY HOME?
GPS спидометр

SITE: http://voltnik.ru/
YOUTUBE: https://www.youtube.com/channel/UC4s13gPVOMQVX3P1ZpdUwjA
Креатед бай voltNik 2018
*********************************************************************/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include "OneButton.h"
#include <EEPROMex.h>
//==============================================================
#define RXPin 4  // UART подключение GPS
#define TXPin 3  // UART подключение GPS
#define BUTN1 7  // пин кнопки1
#define OLED_RENEW 500  // как часто обновлять экран
#define SERIAL_RENEW 1000  // как часто обновлять данные на serial
#define GPSBaud 9600 // скорость обмена с GPS часто именно 9600, но встречается и 4800
#define OLED_I2C_ADDRESS 0x3C // I2C адрес OLED экрана
//==============================================================
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
SSD1306AsciiWire oled;
OneButton butn_1(BUTN1, true);
//==============================================================
long now_millis, lcd_millis, serial_millis; // миллисекунды для отсчета обновления
byte num_ekr = 0 ;                        // номер отображения экрана
double dist_LAT, dist_LNG, last_LAT, last_LNG, max_LAT, max_LNG;                // переменные расстояния в EEPROM
float distToPoint, max_spd, max_dist, distance;
boolean fix;
TinyGPSDate dd; 
TinyGPSTime tt;

//static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;  // нахер лондон
//==============================================================
void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
  //oled.setFont(Adafruit5x7);
  oled.setFont(font5x7);
  //oled.setFont(Arial_bold_14);
  oled.clear();
  oled.set2X();
  oled.println("SPEED   1.1");
  oled.println("TRACKER");
  delay(1000);  // сукадилей
  oled.clear();

  butn_1.attachClick(BTN1_click);                  // подключаем обработку клика кнопки
  butn_1.attachLongPressStart(BTN1_longPress);     // подключаем обработку удержания кнопки

  dist_LAT = EEPROM.readDouble(0); // читаем из памяти широту нулевой точки
  dist_LNG = EEPROM.readDouble(4); // читаем из памяти долготу нулевой точки
  Serial.print("EEPROM POINT, lat: "); Serial.print(dist_LAT,6); Serial.print(" lng: "); Serial.println(dist_LNG,6); 
  
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum Odometer"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to Point   ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}
//==============================================================
void loop()
{
  now_millis = millis();
  butn_1.tick(); // тик опроса кнопки 1
  
  if (now_millis - lcd_millis > OLED_RENEW) { // обновление экрана
    oled_print();
    lcd_millis = now_millis;
  }

  if (now_millis - serial_millis > SERIAL_RENEW) { // печать данных в serial
    dd = gps.date; 
    tt = gps.time;
    if (gps.location.isValid()) { // проверка на FIX и отсутствие нулей в координатах
      if ((gps.location.lat()!=1)or(gps.location.lng()!=1)) {
        distToPoint = (float)TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),dist_LAT,dist_LNG) / 1000;
        if ((distToPoint>max_dist)and(distToPoint-max_dist<100)) {max_dist=distToPoint; max_LAT = gps.location.lat(); max_LNG = gps.location.lng(); fix=1;}
        if (gps.speed.kmph()>max_spd) max_spd=gps.speed.kmph();  
        if ((last_LAT != 0)and(fix)and(gps.speed.kmph()>1)) {distance = distance + (float)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), last_LAT, last_LNG) / 1000; }
        last_LAT = gps.location.lat();
        last_LNG = gps.location.lng();    
        fix=0;
      }  
    } 
    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    printInt(gps.location.age(), gps.location.isValid(), 5);
    printDateTime(dd, tt);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
    printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
    printInt(distToPoint, gps.location.isValid(), 9);
    double courseToPoint = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),dist_LAT,dist_LNG);
    printFloat(courseToPoint, gps.location.isValid(), 7, 2);
    const char *cardinalToPoint = TinyGPSPlus::cardinal(courseToPoint);
    printStr(gps.location.isValid() ? cardinalToPoint : "*** ", 6);
    printInt(gps.charsProcessed(), true, 6);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 9);
    Serial.print("Odo: "); Serial.print(distance,3); Serial.print(" MaxD: "); Serial.print(max_dist,3); Serial.print(" MAX_LAT: "); Serial.print(max_LAT,6); Serial.print(" MAX_LNG: "); Serial.println(max_LNG,6); 
    serial_millis = now_millis;
  }  

  smartDelay(1000); // сукадилей для чтения данных c gps
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
//==============================================================
void BTN1_click() { // клик кнопки. функция смены отображения экрана
  Serial.println("click!");
  num_ekr = (num_ekr+1) % 5;
  oled.clear();
}
//==============================================================
void BTN1_longPress() { // удержание кнопки. функция записи текущего местоположения и сброса значений
  Serial.println("lognpress!");
  if (num_ekr == 1) { 
    max_spd = 0; // сброс максимальной скорости
  } else if (num_ekr == 2) {
    max_dist = 0; // сброс максимальной дистанции
  } else if (num_ekr == 3) {
    distance = 0; // сброс одометра
  } else { // записываем нулевую точку
    max_dist = 0;
    dist_LAT = gps.location.lat();
    dist_LNG = gps.location.lng();
    EEPROM.writeDouble(0,dist_LAT); // записываем в память широту текущей точки
    EEPROM.writeDouble(4,dist_LNG); // записываем в память долготу текущей точки
    oled.clear();
    oled.setCursor(0,0);
    oled.set1X();
    oled.print("Writing...");
    smartDelay(300);
  } 
}
//==============================================================
void oled_print() {  // смена отображения экранов
  oled.setCursor(0,0);
  switch (num_ekr) {
  case 0:
    oled.set1X();
    char st[12];
    sprintf(st, "%02d:%02d:%02d  ", tt.hour(), tt.minute(), tt.second());
    char sd[12];
    sprintf(sd, "%02d/%02d/%02d", dd.day(), dd.month(), dd.year());
    oled.print(st); oled.print(sd); oled.println("   ");
    oled.setCursor(0,1);
    oled.print("Spd: "); oled.print(gps.speed.kmph()); oled.print(" Max: "); oled.print(max_spd); oled.print("  ");
    oled.setCursor(0,2);
    oled.print("Dst: "); oled.print(distToPoint); oled.print(" Max: "); oled.print(max_dist); oled.print("  ");
    oled.setCursor(0,3);
    oled.print("Odo: "); oled.print(distance,3); oled.print(" Sat: "); oled.print((int)gps.satellites.value()); oled.print("  ");
    oled.setCursor(0,5);
    oled.print("Chars: "); oled.print(gps.charsProcessed());
    break;
  case 1:
    oled.set2X();
    oled.print("Spd:"); oled.print(gps.speed.kmph()); oled.println("  ");
    oled.print("Max:"); oled.print(max_spd); oled.print("  ");
    break;
  case 2:
    oled.set2X();
    oled.print("Pnt:"); oled.print(distToPoint,3); oled.println("  ");
    oled.print("Max:"); oled.print(max_dist,3); oled.println("  ");
    break;
  case 3:
    oled.set2X();
    oled.println("Odometer:");
    oled.print(distance,3); oled.println("  ");
    break;
  case 4:
    oled.set2X();
    oled.print(gps.location.lat(),6); oled.println("  ");
    oled.print(gps.location.lng(),6); oled.println("  ");
    break;
  }
}
//==============================================================
static void smartDelay(unsigned long ms) {  // сукадилей для чтения данных с gps
  unsigned long start = millis();
  do {
    while (ss.available()) {
      gps.encode(ss.read());
      //******
      now_millis = millis();    // пока читаем данные еще и кнопку опрашиваем чтобы не зависала и экран обновляем если надо
      butn_1.tick(); // опрос кнопки 1
      if (now_millis - lcd_millis > OLED_RENEW) {
       oled_print();
       lcd_millis = now_millis;
      }
      //******
    }  
  } while (millis() - start < ms);
}
//==============================================================
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid) {
    while (len-- > 1)
    Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}
//==============================================================
static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid) sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}
//==============================================================
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }
  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}
//==============================================================
static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
//==============================================================
