#define RXpin 4             //this is the pin that the Arduino will use to receive data from the GPS
#define TXpin 3             //this is the pin that the Arduino can use to send data (commands) to the GPS
#define GPS_BAUD 38400      //this is the baud rate for the gps (9600 by default)
#define BTN_PIN 2           //this is the pin for the button

#include <U8g2lib.h>  
#include <Arduino.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


#include <TinyGPSPlus.h>

// command to set up the gps
const PROGMEM  uint8_t ClearConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
const PROGMEM  uint8_t setUART115[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xB8,0x42};
const PROGMEM  uint8_t GPGLLOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
const PROGMEM  uint8_t GPGSVOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
const PROGMEM  uint8_t GPVTGOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
const PROGMEM  uint8_t GPGSAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
const PROGMEM  uint8_t GPGGAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
const PROGMEM  uint8_t GPRMCOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
const PROGMEM  uint8_t BAUDRATE38400[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF, 0x70};
const PROGMEM  uint8_t Navrate10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};


struct Coordinate{
  double lat;
  double lng;
};

struct Line{
  Coordinate point_1;   
  Coordinate point_2;   
};

struct LapTime{
  int centisec;
  int sec;
  int minute;
};

struct LineFurmulaConst{
  long double m;   
  long double b;   
};

Coordinate last_coor = {0,0};
Coordinate origin = {0,0};
Coordinate point_origin = {0,0};
// Just because origin is zero
Line start_line = {origin,origin};
TinyGPSPlus gps;

bool isGPSReady = false;
bool btn_click = false;
bool sessionActive = false;
bool start_line_set = false;
int nbLaps = 0;

LapTime last_time;
LapTime current_time;
LapTime sessionTime = {0,0,0};
LapTime bestTime = {0,0,0};


void setup()
{
  pinMode(BTN_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  u8g2.begin();
  u8g2.clearBuffer();          // clear the internal memory
  
  configurate_GPS();
  lcd_print();
}

void loop()
{
  // Check if we have a gps signal
  if(gps.satellites.value()<3 || !gps.satellites.isValid()){
    isGPSReady = false;
    lcd_print();
  }
  while(gps.satellites.value()<3 || !gps.satellites.isValid() || gps.location.lat() == 0.0 || gps.location.lng() == 0.0){
    smartDelay(100);
    if(!(gps.satellites.value()<3 || !gps.satellites.isValid() || gps.location.lat() == 0.0 || gps.location.lng() == 0.0)){
      isGPSReady = true;
      lcd_print();
    }
  }

  if(start_line_set){
    if(segmentsIntersect(gps.location.lat(), gps.location.lng(), last_coor.lat, last_coor.lng, start_line.point_1.lat, start_line.point_1.lng, start_line.point_2.lat, start_line.point_2.lng)){      
      Serial.println(F("LINE INTERSECT"));
      Coordinate new_coor;
      new_coor.lat = gps.location.lat();
      new_coor.lng = gps.location.lng();
      Coordinate intersect = getIntersectPoint(start_line.point_1,start_line.point_2,last_coor,new_coor);
      double d1 = gps.distanceBetween(gps.location.lat(), gps.location.lng(), intersect.lat, intersect.lng);
      double d2 = gps.distanceBetween(gps.location.lat(), gps.location.lng(), last_coor.lat, last_coor.lng);
      double d = d1*100.0/d2;
      int d_ = d/10;
      current_time = getTimeFromGPS(gps.time.centisecond(), gps.time.second(), gps.time.minute(), d_);
      sessionTime = getTime(last_time, current_time);
      if(isBestTime(sessionTime))
        bestTime = sessionTime;
      nbLaps++;
      last_time = current_time;
      lcd_print();
    }
  }
  
  if(btn_click){
     origin = {gps.location.lat(), gps.location.lng()};
     last_time = getTimeFromGPS(gps.time.centisecond(), gps.time.second(), gps.time.minute(), 0);
     //Serial.println("waiting for gps 5 meters");
     double distance = 0;
     while(distance < 10){
      smartDelay(100);
      distance = gps.distanceBetween(gps.location.lat(), gps.location.lng(), origin.lat, origin.lng);
      //Serial.println(distance);
     }
     btn_click = false;
     lcd_print();
     point_origin = {gps.location.lat(), gps.location.lng()};
     start_line = create_line(origin.lat, origin.lng, point_origin.lat, point_origin.lng);
     start_line_set = true; 
     Serial.print("Line :");
     Serial.print(start_line.point_1.lat,6);
     Serial.print(" ");
     Serial.print(start_line.point_1.lng,6);
     Serial.print(" && ");
     Serial.print(start_line.point_2.lat,6);
     Serial.print(" ");
     Serial.print(start_line.point_2.lng,6);
     Serial.println();
  } 
  last_coor = {gps.location.lat(), gps.location.lng()};
  smartDelay(100);
}

bool isBestTime(LapTime _time){
  if(bestTime.minute == 0 && bestTime.sec == 0 && bestTime.centisec == 0)
    return true;
  else if(_time.minute < bestTime.minute){
    return true;
  }
  else if(_time.minute == bestTime.minute){
    if(_time.sec < bestTime.sec){
      return true;
    }
    else if(_time.sec == bestTime.sec){
      if(_time.centisec < bestTime.centisec){
        return true;
      }
    }
  }
  return false;
}
void print_time(LapTime _time){
  if(_time.minute < 10)Serial.print("0");
  Serial.print(_time.minute);
  Serial.print(":");
  if(_time.sec < 10)Serial.print("0");
  Serial.print(_time.sec);
  Serial.print(".");
  if(_time.centisec < 10)Serial.print("0");
  Serial.println(_time.centisec);
}
Coordinate getIntersectPoint(Coordinate p1, Coordinate p2, Coordinate p3, Coordinate p4){
  LineFurmulaConst l1 =  getLineFormulaConst(p1.lat, p1.lng, p2.lat, p2.lng);
  LineFurmulaConst l2 =  getLineFormulaConst(p3.lat, p3.lng, p4.lat, p4.lng);
  Coordinate intersect_point;
  intersect_point.lat = (l2.b-l1.b)/(l1.m-l2.m);
  long double x = (l2.b-l1.b)/(l1.m-l2.m);
  intersect_point.lng = (l1.m*x) + l1.b;
  return intersect_point;
}
LineFurmulaConst getLineFormulaConst(double p1x, double p1y, double p2x, double p2y){  
  long double m = (p2y-p1y)/(p2x-p1x);
  long double b = p2y - (m*p2x);;
  LineFurmulaConst _lfc;
  _lfc.m = m;
  _lfc.b = b;
  return _lfc;
}
LapTime getTime(LapTime last_time, LapTime current_time){
  LapTime _time;
  _time.centisec = current_time.centisec - last_time.centisec; 
  _time.sec = current_time.sec - last_time.sec;
  _time.minute = current_time.minute - last_time.minute;
  if(_time.centisec < 0){
    _time.centisec += 100;
    _time.sec--;
  }
  if(_time.sec < 0){
    _time.sec += 60;
    _time.minute--;
  }
  return _time;
}
LapTime getTimeFromGPS(int centi, int sec, int minute, int millisec){
  LapTime laptime;
  if(centi < 10 && millisec > 0){
    centi += 100;
    sec -= 1;
  }
  laptime.centisec = centi - millisec;
  laptime.sec = sec;
  laptime.minute = minute;
  return laptime;
}
bool segmentsIntersect(double lat1, double lon1, double lat2, double lon2, double finishLineLat1, double finishLineLon1, double finishLineLat2, double finishLineLon2) {  // does line(p1, p2) intersect line(p3, p4)
  double dx = lat2 - lat1;
  double dy = lon2 - lon1;
  double da = finishLineLat2 - finishLineLat1;
  double db = finishLineLon2 - finishLineLon1;
  if ((da * dy - db * dx) == 0) {
    //The two line are parallele
    return false;
  }
  double s = (dx*(finishLineLon1-lon1) + dy*(lat1-finishLineLat1)) / (da*dy - db*dx);
  double t = (da*(lon1-finishLineLon1) + db*(finishLineLat1-lat1)) / (db*dx - da*dy);

  if ((s >= 0) && (s <= 1) && (t >= 0) && (t <= 1))
    return true;
  else
    return false;
}
Line create_line(double p1_lat, double p1_long,double p2_lat, double p2_long){
  double lat_ = p1_lat+((p2_long-p1_long)*0.5);
  double long_ = p1_long+(-(p2_lat-p1_lat));
  Coordinate c1 = {lat_, long_};
  lat_ = p1_lat+(-(p2_long-p1_long)*0.5);
  long_ = p1_long+(p2_lat-p1_lat);
  Coordinate c2 = {lat_, long_};
  Line sl = {c1,c2};
  return sl;
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
              
    check_btn_click();
  } while (millis() - start < ms);
}

void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize)
{
  uint8_t byteread, index;
  //Serial.print(F("GPSSend  "));
  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    /*if (byteread < 0x10)
    {
      Serial.print(F("0"));
    }
    Serial.print(byteread, HEX);
    Serial.print(F(" ")); */
  }
  //Serial.println();
  Progmem_ptr = Progmem_ptr - arraysize;                  //set Progmem_ptr back to start
  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    Serial1.write(byteread);
  }
  delay(100);
}
void configurate_GPS(){
  Serial1.begin(9600);
  GPS_SendConfig(ClearConfig, 21);
  delay(2000);
  GPS_SendConfig(GPGLLOff, 16);
  GPS_SendConfig(GPGSVOff, 16);
  GPS_SendConfig(GPVTGOff, 16);
  GPS_SendConfig(GPGSAOff, 16);
  GPS_SendConfig(Navrate10hz, 14);
  GPS_SendConfig(BAUDRATE38400, 28);
  delay(2000);
  Serial1.begin(38400);
}

void lcd_print(){
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_5x7_tf);//u8g2_font_6x10_tf // //u8g2_font_profont11_tf //u8g2_font_mozart_nbp_tf //u8g2_font_finderskeepers_tf //u8g2_font_8x13_tf 
    u8g2.drawButtonUTF8(37, 9, U8G2_BTN_INV|U8G2_BTN_HCENTER, 90, 0, 2, "BEST LAP" );
    u8g2.drawButtonUTF8(106, 9, U8G2_BTN_INV|U8G2_BTN_HCENTER, 44, 0, 2, "LAP" );
    
    u8g2.setFont(u8g2_font_profont11_tn);//u8g2_font_smart_patrol_nbp_tn
    u8g2.drawFrame(0,11,83,15);
    u8g2.setCursor(20,22);
    if(bestTime.minute != 0 || bestTime.sec != 0 || bestTime.centisec != 0){
      if(bestTime.minute < 10){u8g2.print("0");}
      u8g2.print(bestTime.minute);
      u8g2.print(":");
      if(bestTime.sec < 10){u8g2.print("0");}
      u8g2.print(bestTime.sec);
      u8g2.print(".");
      if(bestTime.centisec < 10){u8g2.print("0");}
      u8g2.print(bestTime.centisec);
    }
    else
       u8g2.print("--:--.--");
       
    u8g2.drawFrame(83,11,45,15);
    u8g2.setCursor(100,22);
    u8g2.print(nbLaps);

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setCursor(4,58);
    if(!isGPSReady){
      u8g2.print("Waiting for GPS");
    }
    else if(btn_click){
      u8g2.print("Setting the start line...");  
    }       
    else if(!start_line_set){
      u8g2.print("No start line");
    }
    
    
    u8g2.setFont(u8g2_font_VCR_OSD_tn);
    /*u8g2.drawButtonUTF8(116, 21, U8G2_BTN_INV|U8G2_BTN_HCENTER, 24, 0, 5, "5" );*/

    u8g2.drawFrame(0,25,128,24);
    u8g2.setCursor(16,45);
    if(sessionTime.minute != 0 || sessionTime.sec != 0 || sessionTime.centisec != 0){
      if(sessionTime.minute < 10){u8g2.print("0");}
      u8g2.print(sessionTime.minute);
      u8g2.print(":");
      if(sessionTime.sec < 10){u8g2.print("0");}
      u8g2.print(sessionTime.sec);
      u8g2.print(".");
      if(sessionTime.centisec < 10){u8g2.print("0");}
      u8g2.print(sessionTime.centisec);
    }
    else
       u8g2.print("--:--.--");
       
    u8g2.drawFrame(100,48,28,16);
    
    u8g2.setFont(u8g2_font_battery19_tn);
    u8g2.setFontDirection(1);
    u8g2.drawStr(104, 52, "5");
    u8g2.setFontDirection(0);

    u8g2.drawFrame(0,48,101,16);
  } while ( u8g2.nextPage() );
}
void check_btn_click(){
  if (digitalRead(BTN_PIN) == LOW) {
    btn_click = true;
    lcd_print();
  }
}
