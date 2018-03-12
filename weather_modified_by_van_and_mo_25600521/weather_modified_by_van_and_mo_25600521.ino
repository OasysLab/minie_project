//#include <Adafruit_BMP085.h>
//Adafruit_BMP085 bmp;

#include <TimerOne.h>
#include <TimerThree.h>

#include <DS3232RTC.h>
#include <Time.h>
#include <TimeLib.h>
#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#define BME280_ADDRESS 0x76
unsigned long int hum_raw, temp_raw, pres_raw;
signed long int t_fine;

#include<SHT1x.h>
#define dataPin 7
#define clockPin 6
SHT1x sht1x(dataPin, clockPin);

#include <SoftwareSerial.h>
#include <XBee.h> //plw

SoftwareSerial xbee(A8, A9); //RX, TX

//plw->
XBee xbeeObj = XBee();
XBeeResponse xbeeRes = XBeeResponse();
ZBRxResponse xbeeRx = ZBRxResponse();
//<-plw


#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);

#include <avr/wdt.h>

#include <Adafruit_VC0706.h>
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial2);

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

int analogPin_windvane = A1; // windvane
int analogPin_ultrasonic = A4; // windvane

int val_windVane = 0;
int val = 0;
int count = 0;

float velocity = 0;
int checkVelo = 0;
float direct = 0;

float temp_c = 0;
float temp_f = 0;
float humi = 0;

float rainfall = 0;
int countRain = 0;
int checkRain = 0;

float pressure = 0;

float dewpt = 0;

int val_button = 0;
int checkButton = 0;

float battery = 0;
float val_battery = 0;
float realBattery = 0;

int val_ultrasonic = 0;
int distance = 0;

int sendSelect = 0;

/*send to weatherunderground*/
String sendStid_value = String ();
String sendWinddir_value = String ();
String sendWindsp_value = String ();
String sendTempc_value = String();
String sendTempf_value = String ();
String sendRainfall_value = String ();
String sendPressure_value = String ();
String sendDewpt_value = String ();
String sendHumi_value = String ();
String sendSoiltemp1_value = String();
String sendSoilhumi1_value = String();
String sendSoiltemp2_value = String();
String sendSoilhumi2_value = String();
String sendImage_value = String();
String sendLux_value = String();
String sendBattery_value = String();


/*send to pixy*/
String sendTemp_pixy =  String();
String  sendHumi_pixy =  String() ;
String  sendHeight_pixy = String();
String  sendWidth_pixy = String();

/*send to sdcard*/
String sendWinddir_SD = String();
String sendWindsp_SD = String();
String sendTempf_SD = String();
String sendRainfall_SD =  String();
String sendPressure_SD =  String();
String sendDewpt_SD =  String();
String sendHumi_SD = String();

int BuffLength = 296;
int BuffLength_2 = 296;
int BuffLength_3 = 296;

int count_senddata = 0;

String str;


int countSD_1;
int countSD_5;
int countSD_15;
int countSD_30;
int countSD_60;

const int chipSelect = 53;

int checkSmsin = 0;

String phoneNumber;
String mixkey_ID;
String sti_ID = String();
String server_status = String();
String DTP = String();

const float alpha = 0.25;

int camera = 0;
int snap = 0;

String nodeData;
char nodeData_2;

String firstValue;
String secondValue;
String thirdValue;
String fourthValue;
String fifthValue;

String sixthValue;  //plw
String seventhValue;//plw
String eighthValue; //plw

#define NodesCount 99
int node[NodesCount];
float nodeTemp1[NodesCount];
int nodeHumi1[NodesCount];
int nodeHeight[NodesCount];
int nodeWidth[NodesCount];
int nodeBatt[NodesCount];       //plw
float nodeSTempC[NodesCount];   //plw
float nodeSHumidity[NodesCount];//plw

void setup() {

  Serial.begin(115200);
  Serial3.begin(115200);
  Serial.println("start");
  xbee.begin(9600);
  xbeeObj.setSerial(xbee);  //plw
  SD.begin(chipSelect);
  /*if(!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }*/
  setSyncProvider(RTC.get);


  //  setup serial
  pinMode(4, INPUT_PULLUP);

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(calculation);

  Timer3.initialize(1000000);
  Timer3.attachInterrupt(countDisplay);

  attachInterrupt(5, countVelo_, CHANGE); // anemometer pin 18
  attachInterrupt(4, calculationRain, CHANGE); // rain pin2

  attachInterrupt(0, checkSnap, FALLING); // buuton 2

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off
  uint8_t spi3w_en = 0;           //3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;



  //attachInterrupt(4,checkDisplay,LOW); // button pin 19
  wdt_enable(WDTO_8S);
  readTrim();
  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);

  for(int i = 0; i < NodesCount; i++)
  {
    node[i] = -99;
  }
}

void loop() {

  digitalWrite(12, HIGH);   // turn the LED on (HIGH is the volta
  // wait for a second
  delay(1000);
  digitalWrite(12, LOW);


  val_button = digitalRead(4);

  val_battery = analogRead(A0);


  Serial.println(count_senddata);



  val_windVane = analogRead(analogPin_windvane);

  getBME280();
  temp_f = ((temp_c * 1.80) + 32.00);
  dewPoint(temp_f, humi);

  getDirection();
  getBattery();
  xbeeRead();


  wdt_reset();

  delay(1000);

  wdt_reset();

  // printTempc(temp_c);
  // printHumidity(humi);
  // printPressure(pressure);
  // printVelocity(velocity);
  // printDirection(direct);
  // printRain(rainfall);
  // printUltrasonic(distance);
  //Serial.println(snap);



  if (count_senddata >= 20)
  {
    digitalWrite(11, HIGH);
    sendDatacheck(0);
    digitalWrite(11, LOW);
    count_senddata = 0;
    rainfall = 0.0;
  }

}

void getBME280() {

  double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
  signed long int temp_cal;
  unsigned long int press_cal, hum_cal;

  readData();

  temp_cal = calibration_T(temp_raw);
  press_cal = calibration_P(pres_raw);
  hum_cal = calibration_H(hum_raw);
  temp_c = (double)temp_cal / 100.0;
  pressure = (double)press_cal / 100.0;
  humi = (double)hum_cal / 1024.0;
}

void readTrim()
{
  uint8_t data[32], i = 0;
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 24);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xA1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  data[i] = Wire.read();
  i++;

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 7);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26] << 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}
void writeReg(uint8_t reg_address, uint8_t data)
{
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}
void readData()
{
  int i = 0;
  uint32_t data[8];
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 8);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw  = (data[6] << 8) | data[7];
}
signed long int calibration_T(signed long int adc_T)
{

  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0;
  }
  P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000)
  {
    P = (P << 1) / ((unsigned long int) var1);
  }
  else
  {
    P = (P / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
  P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
            ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
                ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int)(v_x1 >> 12);
}

void countRain_() {

  countRain++;

}


void calculationRain() {

  countRain++;
  rainfall = (0.2 * countRain) / 3;
  //countRain = 0;
  checkRain = 1;

}

void countVelo_() {

  count++;

  //Serial.println("ok");

}

void calculation() {

  velocity = 1.49 * count;
  count = 0;
  checkVelo = 1;


  countSD_1++;
  count_senddata++;




}

void dewPoint(float tempf, float humidity)
{
  float A0 = 373.15 / (273.15 + tempf);
  float SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1) ;
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP / 0.61078);
  dewpt = (241.88 * T) / (17.558 - T);

}

float EWMA(float value) {

  float ewma_value;
  float ewma_value_t1;


}


void getBattery() {
  battery = ((val_battery * 5) / 1023) * 6;
  if (battery > 14.45)
  {
    realBattery = 100;
  }
  else if (battery < 10.00)
  {
    realBattery = 0;
  }
  else
  {
    realBattery = ((battery - 10) / 4.45) * 100;
  }


}

void getDirection() {

  if (val_windVane >= 790 && val_windVane <= 780)
  {
    direct = 0;

  }
  else if (val_windVane >= 400 && val_windVane <= 410)
  {
    direct = 22.5;
    //Serial.println("22.5 deg");
  }
  else if (val_windVane >= 460 && val_windVane <= 470)
  {
    direct = 45;
    // Serial.println("45 deg");
  }
  else if (val_windVane >= 80 && val_windVane <= 90)
  {
    direct = 67.5;
    // Serial.println("67.5 deg");
  }
  else if (val_windVane >= 90 && val_windVane <= 100)
  {
    direct = 90;
    //Serial.println("90 deg");
  }
  else if (val_windVane >= 60 && val_windVane <= 70)
  {
    direct = 112.5;
    // Serial.println("112.5 deg");
  }

  else if (val_windVane >= 180 && val_windVane <= 190)
  {
    direct = 135;
    // Serial.println("135 deg");
  }
  else if (val_windVane >= 120 && val_windVane <= 130)
  {
    direct = 157.5;
    //Serial.println("157.5 deg");
  }
  else if (val_windVane >= 280 && val_windVane <= 290)
  {
    direct = 180;
    //Serial.println("180 deg");
  }
  else if (val_windVane >= 240 && val_windVane <= 250)
  {
    direct = 202.5;
    //Serial.println("202.5 deg");
  }
  else if (val_windVane >= 630 && val_windVane <= 640)
  {
    direct = 225;
    //Serial.println("225 deg");
  }
  else if (val_windVane >= 595 && val_windVane <= 605)
  {
    direct = 247.5;
    //Serial.println("247.5 deg");
  }
  else if (val_windVane >= 940 && val_windVane <= 950)
  {
    direct = 270;
    //Serial.println("270 deg");
  }

  else if (val_windVane >= 820 && val_windVane <= 830)
  {
    direct = 292.5;
    // Serial.println("292.5 deg");
  }
  else if (val_windVane >= 870 && val_windVane <= 880)
  {
    direct = 315;
    //Serial.println("315 deg");
  }
  else if (val_windVane >= 700 && val_windVane <= 710)
  {
    direct = 337.5;
    // Serial.println("337.5 deg");
  }
  else
  {
    direct = 0;
    // Serial.println("0 deg");
  }


}

void printVelocity(float velo) {

  //Serial.println(val);
  //Serial.println(count);
  Serial.print("Velocity : ");
  Serial.print(velo);
  Serial.println(" MPH");


}

void printRain(float rain) {

  //Serial.println(val);
  //Serial.println(count);
  Serial.print("rainfall : ");
  Serial.print(rain);
  Serial.println(" mm");


}


void printDirection(float direct) {
  Serial.print("direction : ");
  Serial.print(direct);
  Serial.println(" degree");

}


void printTempc(float temp) {
  Serial.print("Temperature: ");
  Serial.print(temp, DEC);
  Serial.print("C / ");

}

void printHumidity(float humi) {
  Serial.print("Humidity: ");
  Serial.print(humi);
  Serial.println("%");

}

void printPressure(float presure)
{
  Serial.print("pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");
}

void showDisplay_temp() {

  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 27);
  u8g.print("Temp : ");
  u8g.setPrintPos(60, 27);
  u8g.print(temp_c);
  u8g.setPrintPos(120, 27);
  u8g.print("C");

}

void showDisplay_humi() {

  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 42 );
  u8g.print("humi : ");
  u8g.setPrintPos(60, 42);
  u8g.print(humi);
  u8g.setPrintPos(120, 42);
  u8g.print("%");

}

void showDisplay_pressure() {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 57);
  u8g.print("Pres : ");
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(60, 57);
  u8g.print(pressure / 100);
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(110, 57);
  u8g.print("hPa");

}

void showDisplay_velocity(void) {

  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 27);
  u8g.print("Windsp");
  u8g.setPrintPos(50, 27);
  u8g.print(velocity);
  u8g.setPrintPos(90, 27);
  u8g.print("mph");

}


void showDisplay_direction(void) {

  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 42);
  u8g.print("Winddir");
  u8g.setPrintPos(57, 42);
  u8g.print(direct);
  u8g.setPrintPos(120, 42);
  u8g.print("Deg");


}

void showDisplay_rainfall(void) {

  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 27);
  u8g.print("Rainfall");
  u8g.setPrintPos(70, 27);
  u8g.print(rainfall);
  u8g.setPrintPos(110, 27);
  u8g.print("mm");


}



void showDisplay_battery(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 10);
  u8g.print("Battery");
  u8g.setPrintPos(60, 10);
  u8g.print((int)realBattery);
  u8g.setPrintPos(120, 10);
  u8g.print("%");

}

void countDisplay() {
  // Serial.println("ok");
  if (snap != 1) {
    checkButton++;
    if (checkButton > 3 ) {
      checkButton = 0;
    }
  }
  else
  {
    checkButton = 4;
  }

  checkDisplay();

}


void checkDisplay(void) {


  if (checkButton == 10)
  {
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_unifont);

      u8g.setPrintPos(0, 20);
      u8g.print("");

    } while ( u8g.nextPage() );
  }

  else if (checkButton == 1)
  {
    u8g.firstPage();
    do {
      showDisplay_temp();
      showDisplay_humi();
      showDisplay_pressure();
      showDisplay_battery();
      //delay(3000);
    } while ( u8g.nextPage());
  }

  else if (checkButton == 2 )
  {
    u8g.firstPage();
    do {
      showDisplay_velocity();
      showDisplay_direction();
      showDisplay_battery();
      //delay(3000);


    } while ( u8g.nextPage() );
  }

  else if (checkButton == 3)
  {
    u8g.firstPage();
    do {
      showDisplay_rainfall();

      showDisplay_battery();
      //delay(3000);

    } while ( u8g.nextPage() );
  }

  else if (checkButton == 4)
  {
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_unifont);

      u8g.setPrintPos(20, 30);
      u8g.print("Snap...");

    } while ( u8g.nextPage() );
  }
  //Serial.print("stage ");
  //Serial.println(checkButton);
}

void sendDatacheck(int sendSelect) {

  switch (sendSelect) {

    case 0:
      Serial.println('case0');
      // SendDataToWeatherunderground();
      SendDataToWt();
//      SendAllNodesDataToWt(); //plw
      // SendDataToPixy();
      break;

    case 1:
      Serial.println('case1');
      //SendDataToWeatherunderground();
      break;

    case 2:
      Serial.println('case2');
      //SendDataToPixy();

      break;

  }

}

/*void SendDataToWeatherunderground(void) {

  String SERVER  = "http://rtupdate.wunderground.com";
  String WEBPAGE = "/weatherstation/updateweatherstation.php?";
  String ID = "ID=ICHANGWA30&"; // ICHANGWA16,ICHIANGM12,I1275,ICHIANGM22
  String PASSWORD = "PASSWORD=718t3uky&";//0jalqq2z
  String timenow = "dateutc=now";


  String sendWinddir = "&winddir=";
  String sendWindsp = "&windspeedmph=";
  String sendWindgut = "&windgustmph=0";
  String sendTempf = "&tempf=";
  String sendRainfall = "&rainin=";
  String sendPressure = "&baromin=";
  String sendDewpt = "&dewptf=";
  String sendHumi = "&humidity=";
  String sendSoiltemp1 = "&soiltempf=";
  String sendSoilhumi1 = "&soilmoisture=";
  String sendSoiltemp2 = "&soiltemf2=";
  String sendSoilhumi2 = "&soilmoisture2=";
  String sendWeather = "&weather=&clouds=&softwaretype=vws%20versionxx&action=updateraw&realtime=1&rtfreq=2.5";
  String sendData = String();


  sendWinddir_value = sendWinddir + (int)direct;
  sendWindsp_value = sendWindsp + (int)velocity;
  sendTempf_value = sendTempf + (int)temp_f;
  sendRainfall_value =  sendRainfall + (int)rainfall;
  sendPressure_value =  sendPressure + (pressure/100.00);
  sendDewpt_value =  sendDewpt + dewpt;
  sendHumi_value = sendHumi + (int)humi;




  sendData =  SERVER + WEBPAGE + ID + PASSWORD + timenow + sendWinddir_value + sendWindsp_value + sendWindgut  + sendTempf_value + sendRainfall_value + sendPressure_value + sendDewpt_value + sendHumi_value +sendWeather;
  BuffLength = sendData.length();

  //Serial.println("senddata to weatherunderground");
  //delay(1000);

  Serial3.println("AT\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+CSQ\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QHTTPCFG=\"contextid\",1\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QHTTPCFG=\"responseheader\",1\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QICSGP=1,1,\"my\",\"\",\"\",0\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT=1\r");
  ReadSerial();
  delay(3500);

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.print("AT+QHTTPURL=");
  Serial3.print(BuffLength);
  Serial3.print(",80\r\n");
  ReadSerial();
  delay(1000);

  Serial.print(sendData);

  Serial3.print(sendData);

  Serial3.print("\r\n");
  ReadSerial();
  delay(3000);

  Serial3.println("AT+QHTTPGET=80\r");
  ReadSerial();


  delay(3000);


  Serial3.println("AT+QHTTPREAD=80\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIRD=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QICLOSE=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIDEACT=1");
  ReadSerial();
  delay(5000);
  // wdt_reset();
  //delay(5000);
  // wdt_reset();


  }

  void SendDataToPixy(void) {
  // http://202.28.24.69/~khunchangkhian/getdata.php?Temp_Air=23&Hum_Air=23&Image=asdf&Height=23&Width=23&Point=23&Processing=23
  String URL = "http://202.28.24.69/";
  String path = "~khunchangkhian/getdata.php?";

  String sendTemp_air = "Temp_Air=";
  String sendHumi_air = "&Hum_Air=";
  String sendHeight = "&Height=";
  String sendWidth = "&Width=";


  sendTemp_pixy =  sendTemp_air +  ("%.1f",nodeTemp1);
  sendHumi_pixy =  sendHumi_air + nodeHumi1 ;
  sendHeight_pixy = sendHeight + nodeHeight;
  sendWidth_pixy = sendWidth + nodeWidth;



  String sendData_Pixy = URL + path + sendTemp_pixy + sendHumi_pixy + sendHeight_pixy + sendWidth_pixy;
  BuffLength_3 = sendData_Pixy.length();



  Serial.println("Send Data To crflood\r\n");
  ReadSerial();
  delay(1000);

  Serial3.println("AT\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+CSQ\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QHTTPCFG=\"contextid\",1\r");
  ReadSerial();
  delay(50);

  Serial3.println("AT+QHTTPCFG=\"responseheader\",1\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QICSGP=1,1,\"my\",\"\",\"\",0\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT=1\r");
  ReadSerial();
  delay(5000);
  //wdt_reset();
  //delay(2500);
  //wdt_reset();

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.print("AT+QHTTPURL=");
  Serial3.print(BuffLength_3);
  Serial3.print(",80\r\n");
  ReadSerial();
  delay(1000);
  Serial.println(sendData_Pixy);

  Serial3.print(sendData_Pixy);


  Serial3.print("\r\n");
  ReadSerial();
  delay(3000);

  Serial3.println("AT+QHTTPGET=80\r");
  ReadSerial();
  delay(2500);

  Serial3.println("AT+QHTTPREAD=80\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIRD=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QICLOSE=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIDEACT=1");
  ReadSerial();
  delay(5000);

  }
*/
void SendDataToWt(void) {
  // http://202.28.24.69/~khunchangkhian/weather.php?Temperature=24&Humidity=23&Pressure=23&WindSpeed=23&WindDirection=23&Rain=23&Image=asdf&Lux=23&Battery=23

  //String URL = "http://202.28.24.69/";
  //String path = "~khunchangkhian/weather.php?";

  String URL = "http://nahmchan.oasys-lab.com/";
  String path = "getData_weather_station.php?";

  String sendStid_wt = "StationID=";
  String sendTempc_wt = "&Temperature=";
  String sendHumi_wt = "&Humidity=";
  String sendPressure_wt = "&Pressure=";
  String sendWindsp_wt = "&WindSpeed=";
  String sendWinddir_wt = "&WindDirection=";
  String sendRainfall_wt = "&Rain=";
  //String sendImage_wt = "&Image=";
  //String sendLux_wt = "&Lux";
  String sendBattery_wt = "&Battery=";


  sendStid_value =  sendStid_wt + "99"; /// change station ID
  sendTempc_value =  sendTempc_wt + (int)temp_c;
  sendHumi_value = sendHumi_wt + (int)humi;
  sendPressure_value =  sendPressure_wt + (pressure / 100.00);
  sendWindsp_value = sendWindsp_wt + (int)velocity;
  sendWinddir_value = sendWinddir_wt + (int)direct;
  sendRainfall_value =  sendRainfall_wt + (int)rainfall;
  //sendImage_value = sendImage_wt + "null";
  //sendLux_value = sendLux_wt + "0";
  sendBattery_value = sendBattery_wt + battery;

  String sendData_wt = URL + path + sendStid_value + sendTempc_value + sendHumi_value + sendPressure_value + sendWindsp_value + sendWinddir_value + sendRainfall_value + sendBattery_value;
  BuffLength_3 = sendData_wt.length();



  Serial.println("Send Data Station To wt\r\n");
  ReadSerial();
  delay(1000);

  Serial3.println("AT\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+CSQ\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QHTTPCFG=\"contextid\",1\r");
  ReadSerial();
  delay(50);

  Serial3.println("AT+QHTTPCFG=\"responseheader\",1\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QICSGP=1,1,\"my\",\"\",\"\",0\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT=1\r");
  ReadSerial();
  delay(5000);
  //wdt_reset();
  //delay(2500);
  //wdt_reset();

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.print("AT+QHTTPURL=");
  Serial3.print(BuffLength_3);
  Serial3.print(",80\r\n");
  ReadSerial();
  delay(1000);
  Serial.println(sendData_wt);

  Serial3.print(sendData_wt);


  Serial3.print("\r\n");
  ReadSerial();
  delay(3000);

  Serial3.println("AT+QHTTPGET=80\r");
  ReadSerial();
  delay(2500);

  Serial3.println("AT+QHTTPREAD=80\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIRD=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QICLOSE=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIDEACT=1");
  ReadSerial();
  delay(5000);

}

void SendAllNodesDataToWt()
{
  for(int i = 0; i < NodesCount; i++)
  {
    if(node[i] == -99)
    {
      continue;
    }
    SendNodeDataToWt(i);
  }
}

//plw->
void SendNodeDataToWt(int index) {
  // http://202.28.24.69/~khunchangkhian/weather.php?Temperature=24&Humidity=23&Pressure=23&WindSpeed=23&WindDirection=23&Rain=23&Image=asdf&Lux=23&Battery=23

  //String URL = "http://202.28.24.69/";
  //String path = "~khunchangkhian/weather.php?";

  String URL = "http://nahmchan.oasys-lab.com/";
  String path = "getData_weather_node.php?";

  String values = "NodeID=" + (String)node[index];
  values += "&StationID=02";
  values += "&s_Temperature=" + (String)("%.2f", nodeSTempC[index]);
  values += "&s_Humidity=" + (String)("%.2f", nodeSHumidity[index]);
  values += "&a_Temperature=" + (String)("%.2f", nodeTemp1[index]);
  values += "&a_Humidity=" + (String)nodeHumi1[index];
  values += "&Battery=" + (String)nodeBatt[index];

  node[index] = -99;

  String sendData_wt = URL + path + values;
  BuffLength_3 = sendData_wt.length();


  Serial.println("Send Data Node To wt\r\n");
  ReadSerial();
  delay(1000);

  Serial3.println("AT\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+CSQ\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QHTTPCFG=\"contextid\",1\r");
  ReadSerial();
  delay(50);

  Serial3.println("AT+QHTTPCFG=\"responseheader\",1\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QICSGP=1,1,\"my\",\"\",\"\",0\r");
  ReadSerial();
  delay(500);

  Serial3.println("AT+QIACT=1\r");
  ReadSerial();
  delay(5000);
  //wdt_reset();
  //delay(2500);
  //wdt_reset();

  Serial3.println("AT+QIACT?\r");
  ReadSerial();
  delay(500);

  Serial3.print("AT+QHTTPURL=");
  Serial3.print(BuffLength_3);
  Serial3.print(",80\r\n");
  ReadSerial();
  delay(1000);
  Serial.println(sendData_wt);

  Serial3.print(sendData_wt);


  Serial3.print("\r\n");
  ReadSerial();
  delay(3000);

  Serial3.println("AT+QHTTPGET=80\r");
  ReadSerial();
  delay(2500);

  Serial3.println("AT+QHTTPREAD=80\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIRD=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QICLOSE=0\r");
  ReadSerial();
  delay(1000);

  Serial3.println("AT+QIDEACT=1");
  ReadSerial();
  delay(5000);

}
//<-plw

void ReadSerial()
{
  str = Serial3.readString();
  Serial.println(str);
  wdt_reset();
}

void saveSD() {
  delay(1000);
  countSD_1 = 0;


  String sendWinddir = "winddir=";
  String sendWindsp = "&windspeedmph=";
  String sendWindgut = "&windgustmph=0";
  String sendTempc = "&tempc=";
  String sendRainfall = "&rainin=";
  String sendPressure = "&baromin=";
  String sendDewpt = "&dewptf=";
  String sendHumi = "&humidity=";

  sendWinddir_SD = sendWinddir + direct;
  sendWindsp_SD = sendWindsp + velocity;
  sendTempf_SD = sendTempc + temp_c;
  sendRainfall_SD = sendRainfall + rainfall;
  sendPressure_SD =  sendPressure + pressure;
  sendDewpt_SD =  sendDewpt + dewpt;

  sendHumi_SD =  sendHumi + humi;

  String sendData_SD = sendWinddir_SD + sendWindsp_SD + sendTempf_SD + sendRainfall_SD + sendPressure_SD + sendDewpt_SD + sendHumi_value;

  String dateTime = String();
  String dateTime_data = String();
  String dateTime_rain = String();
  dateTime += hour();
  dateTime += ":";
  dateTime += minute();
  dateTime += ":";
  dateTime += second();
  dateTime += " ";
  dateTime += day();
  dateTime += "-";
  dateTime += month();
  dateTime += "-";
  dateTime += year();
  dateTime += " ";

  dateTime_data += dateTime;
  dateTime_rain += dateTime;


  dateTime_data += sendData_SD;
  dateTime_rain += "rain=";
  dateTime_rain += rainfall;


  // File dataFile = SD.open("/data/sensor.txt", FILE_WRITE);

  File dataFile_1 = SD.open("/data/sensor1.txt", FILE_WRITE); // 1 min round


  // if the file is available, write to it:


  if (dataFile_1) {
    dataFile_1.println(dateTime_data);
    dataFile_1.close();
    // print to the serial port too:

    Serial.println(dateTime_data);
    Serial.println("1min save complete");


  }
  File dataRain = SD.open("/data/rain1.txt", FILE_WRITE);

  if (dataRain) {
    dataRain.println(dateTime_rain);
    dataRain.close();
    // print to the serial port too:

    Serial.println(dateTime_rain);
    Serial.println("rain save complete");
    rainfall = 0.00;


  }



}
void snapShot() {

  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_unifont);

    u8g.setPrintPos(20, 20);
    u8g.print("Snap...");

  } while ( u8g.nextPage() );



  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }

  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120
  // Remember that bigger pictures take longer to transmit!

  cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  //cam.setImageSize(VC0706_160x120);          // small

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");

  Serial.println("Snap in 3 secs...");
  delay(3000);

  if (! cam.takePicture())
    Serial.println("Failed to snap!");
  else
    Serial.println("Picture taken!");

  // Create an image with the name IMAGExx.JPG
  char filename[16];
  strcpy(filename, "/img/IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[10] = '0' + i / 10;
    filename[11] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // Open the file for writing
  File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if (++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
      wdt_reset();
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;

  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  //Serial.print(time); Serial.println(" ms elapsed");


}



void checkSnap() {
  snap = 1;
}

//plw->
void write_xbee(String data)
{
  uint8_t data_int[data.length()];
  for(int i = 0; i < data.length(); i++)
  {
    data_int[i] = (uint8_t)data[i];
  }
  XBeeAddress64 addr64 = XBeeAddress64();
  addr64.setMsb(0x00000000);
  addr64.setLsb(0x0000ffff);

  ZBTxRequest zbTx = ZBTxRequest(addr64, data_int, sizeof(data_int));
  xbeeObj.send(zbTx);
  Serial.println("Data sent: " + data);
}

String read_xbee()
{
  String sample;
  xbeeObj.readPacket();

  if (xbeeObj.getResponse().isAvailable())
  {
    if (xbeeObj.getResponse().getApiId() == ZB_RX_RESPONSE)
    {
      xbeeObj.getResponse().getZBRxResponse(xbeeRx);
      for (int i = 0; i < xbeeRx.getDataLength(); i++)
      {
        sample += (char)xbeeRx.getData(i);
      }
    }

    return sample;
  }
  else if (xbeeObj.getResponse().isError())
  {
    return (String)xbeeObj.getResponse().getErrorCode();
  }
  else
  {
    return "";
  }
}
//<-plw

void xbeeRead()
{
  String XBee_data = read_xbee();

  Serial.println(XBee_data);

  if (XBee_data != "" && XBee_data.endsWith("E"))
  {
    nodeData = XBee_data.substring(0, XBee_data.indexOf("E"));  //plw

    //nodeData = xbee.readStringUntil('E'); // tmpmesses2 = mySerial.readStringUntil('E');
    Serial.println(nodeData);

    //nodeData_2 = xbee.read();
    //Serial.print(nodeData_2);
    int commaIndex = nodeData.indexOf(',');
    int secondCommaIndex = nodeData.indexOf(',', commaIndex + 1);
    int thirdCommaIndex = nodeData.indexOf(',', secondCommaIndex + 1);
    int fourthCommaIndex = nodeData.indexOf(',', thirdCommaIndex + 1);

    //plw->
    int fifthCommaIndex = nodeData.indexOf(',', fourthCommaIndex + 1);
    int sixthCommaIndex = nodeData.indexOf(',', fifthCommaIndex + 1);
    int seventhCommaIndex = nodeData.indexOf(',', sixthCommaIndex + 1);
    //<-plw

    firstValue = nodeData.substring(0, commaIndex);
    secondValue = nodeData.substring(commaIndex + 1, secondCommaIndex);
    thirdValue = nodeData.substring(secondCommaIndex + 1, thirdCommaIndex); // To the end of the string
    fourthValue = nodeData.substring(thirdCommaIndex + 1, fourthCommaIndex);

    //plw->
    fifthValue = nodeData.substring(fourthCommaIndex + 1, fifthCommaIndex);
    sixthValue = nodeData.substring(fifthCommaIndex + 1, sixthCommaIndex);
    seventhValue = nodeData.substring(sixthCommaIndex + 1, seventhCommaIndex);
    eighthValue = nodeData.substring(seventhCommaIndex + 1);
    //<-plw

    String nodeValue = String();
    nodeValue = "N=" + firstValue + "&" + "T=" + secondValue + "&" + "H=" + thirdValue + "&" + "Hi=" + fourthValue + "&" + "Wd=" +  fifthValue + "&" + "Bt=" + sixthValue + "&" + "sTC=" + seventhValue + "&" + "sH=" + eighthValue;  //plw
    Serial.println(nodeValue);
    if (firstValue != "\r\n")
    {
      int index = firstValue.toInt() - 1;
      node[index] = firstValue.toInt();
      nodeTemp1[index] = secondValue.toFloat();
      nodeHumi1[index] = thirdValue.toInt();
      nodeHeight[index] = fourthValue.toInt();
      nodeWidth[index] = fifthValue.toInt();

      nodeBatt[index] = sixthValue.toInt();
      nodeSTempC[index] = seventhValue.toFloat();
      nodeSHumidity[index] = eighthValue.toFloat();

      write_xbee("ACK" + firstValue + "OK");
    }
//    Serial.println(node);
//    Serial.println(nodeTemp1);
//    Serial.println(nodeHumi1);
//    Serial.println(nodeHeight);
//    Serial.println(nodeWidth);
//    //plw->
//    Serial.println(nodeBatt);
//    Serial.println(nodeSTempC);
//    Serial.println(nodeSHumidity);
//    //<-plw
//    Serial.println("");
  }
}













