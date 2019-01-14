// Arduino APRS Tracker (aat) with Arduino Pro Mini 3.3V/8 MHz
// Based on https://github.com/sh123/aprs_tracker

//GPS 
#include <TinyGPS.h>
const int G_EN =  23;
TinyGPS gps;
char cmd[10];
int cmdIndex;
//long instead of float for latitude and longitude
long lat = 0;
long lon = 0;
int year = 0;
byte month = 0, day = 0, hour = 0, minute = 0, second = 0, hundredths = 0;
unsigned long age = 0;
// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

// Single shot button
#define BUTTON_PIN 26
#define BUTTON_PIN_SOS 25

const int BT_EN =  24;
const int BT_RST =  32;
const int PTT =  38;
// battery sence
int BATTERY_SENSE_PIN = A1;
int oldBatteryPcnt = 0;
float batteryV = 0;
int batteryPcnt = 0;
int sensorValue = 0;

// LibAPRS
#define OPEN_SQUELCH false
#define ADC_REFERENCE REF_5V

// APRS settings
#include <LibAPRS.h>
char APRS_CALLSIGN[] = "LZ1AAO";
const int APRS_SSID = 5;
char APRS_SYMBOL = '>';
char comment_flex[]= "APRS Tracker";

// Timer
#include <SimpleTimer.h>
#define TIMER_DISABLED -1
SimpleTimer timer;
char aprs_update_timer_id = TIMER_DISABLED;
bool send_aprs_update = false;

//Reading serial comands
bool BTnewData = false;
String S1inputString = "";
String S1inputStringtest = "";

//reading charger events
#define STATUS_STB 52
#define STATUS_CHRG 53
int status_stb_ch = 1 ;
int status_chrg_ch = 1 ;
int old_status_stb_ch = 0 ;
int old_status_chrg_ch = 0 ;

//one wire
#include <OneWire.h> 
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 33
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


void setup()
{
//Single shot button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN_SOS, INPUT_PULLUP);
//reading charger events
  pinMode(STATUS_STB, INPUT);
  pinMode(STATUS_CHRG, INPUT);
//GPS
  pinMode(G_EN, OUTPUT);
  
//Bluetooth
  pinMode(BT_EN, OUTPUT);
  pinMode(BT_RST, OUTPUT);
  
//Radio
  pinMode(PTT, OUTPUT);
  
//Battery
//  analogReference(INTERNAL1V1);
  
  Serial.begin(115200);
  digitalWrite(BT_EN, HIGH);
  delay(50); 
  Serial1.begin(9600);
  digitalWrite(G_EN, HIGH);
  Serial.println("GPS Power ON");
  Serial1.println("GPS Power ON");
  Serial.println("Waiting GPS init");
  Serial1.println("Waiting GPS init");
  delay(1000); 
  Serial.print(".");
  Serial1.print(".");
  delay(1000);
  Serial.print(".");
  Serial1.print(".");
  delay(1000);
  Serial.println(".");
  Serial1.println(".");
  delay(1000);
  Serial3.begin(9600);

  Serial.println("APRS Tracker");
  Serial1.println("APRS Tracker");

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(APRS_CALLSIGN, APRS_SSID);
  APRS_setSymbol(APRS_SYMBOL);

  aprs_update_timer_id = timer.setInterval(2L * 60L * 1000L, setAprsUpdateFlag);

  S1inputString.reserve(32);
  S1inputStringtest.reserve(32);

  cmdIndex = 0;

  //onewire
   sensors.begin(); 
  

}

void loop()
{
  bool newData = false;


  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial3.available())
    {
      char c = Serial3.read();
     // Serial1.write(c); // uncomment this line if you want to see the GPS data flowing
     //  BTSerial.println(F("GPS Signal Found"));
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &age);
    gps.get_position(&lat, &lon, &age);



    if (digitalRead(BUTTON_PIN) == 0)
    {
      while (digitalRead(BUTTON_PIN) == 0) {}; //debounce
      Serial.println("MANUAL UPDATE");
      Serial1.println("MANUAL UPDATE");
      locationUpdate();
    }

    if (digitalRead(BUTTON_PIN_SOS) == 0)
    {
      //Serial.println("Button Pressed");
      // BTSerial.println("Button Pressed");
      while (digitalRead(BUTTON_PIN) == 0) {}; //debounce
      Serial.println("MANUAL UPDATE SOS");
      Serial1.println("MANUAL UPDATE SOS");
//      comment_flex = "Test APRS Tracker";
      locationUpdateSOS();
    }

    if (send_aprs_update) {
      Serial.println("APRS UPDATE");
      Serial1.println("APRS UPDATE");
      locationUpdate();
      send_aprs_update = false;
    }

  }
  timer.run();

  status_stb_ch = digitalRead(STATUS_STB);
  if(status_stb_ch!=old_status_stb_ch) {
    if(status_stb_ch==0) {
      Serial.println("Charge Termination ON");
      Serial1.println("Charge Termination ON");
    }
    else {
      Serial.println("Charge Termination OFF");
      Serial1.println("Charge Termination OFF");
    }
    old_status_stb_ch = status_stb_ch;
  }

    status_chrg_ch = digitalRead(STATUS_CHRG);
  if(status_chrg_ch!=old_status_chrg_ch) {
    if(status_chrg_ch==0) {
      Serial.println("Charging");
      Serial1.println("Charging");
    }
    else {
      Serial.println("Charged");
      Serial1.println("Charged");
    }
    old_status_chrg_ch = status_chrg_ch;
  }

// get the battery Voltage
//    int sensorValue = analogRead(BATTERY_SENSE_PIN);
    sensorValue = analogRead(BATTERY_SENSE_PIN);
    batteryPcnt=map(sensorValue,600,850,0,100);
//For Volt Divider
//    batteryPcnt = sensorValue / 10;
//    if(batteryPcnt>100) {
//   batteryPcnt = 100;
// }
//  batteryV  = sensorValue * 0.003225806; //1V1 referenca 328p
    batteryV  = sensorValue * 0.004887585;

//onewire
    sensors.requestTemperatures(); // Send the command to get temperature readings 
//    Serial.print("Temperature is: "); 
//    Serial.print(sensors.getTempCByIndex(0));
}


void serialEvent1() {
  
  char readbt;

  while (Serial1.available())
  {
    readbt = Serial1.read();
    S1inputString += readbt;
    if (readbt  == '\r') {
    BTnewData = true;
    }
  }
  if (BTnewData) {
   // Serial.print("String= ");
    Serial1.println(S1inputString);
  //  Serial.print("char= ");
   // Serial.println(readbt);
    BTnewData = false;
   
   if(S1inputString == "battery\r" ) {
      Serial1.print("Battery Voltage= ");
      Serial1.print(batteryV);
      Serial1.println("V");
      Serial1.print("Battery Persentage= ");
      Serial1.print(batteryPcnt);
      Serial1.println("%");
//      Serial1.println(sensorValue);
    }


     
    else if(S1inputString == "state_battery\r" ) {
      if(status_stb_ch==0) {
      Serial1.println("Charge Termination ON");
    }
    else {
      Serial1.println("Charge Termination OFF");
    }
      if(status_chrg_ch==0) {
      Serial1.println("Charging");
    }
    else {
      Serial1.println("Charged");
    }
    }
// not work
//    else if(S1inputString == "bt_reser\r" ); {
//      digitalWrite(BT_RST, HIGH);
//      Serial1.println("EST");
//      delay(50);
//      digitalWrite(BT_RST, LOW);
//    }

    else if(S1inputString == "gpsoff\r" ) {
              digitalWrite(G_EN, LOW);
              Serial1.println("GPS is OFF");
    }

    else if(S1inputString == "gpson\r" ) {
              digitalWrite(G_EN, HIGH);
              Serial1.println("GPS is ON Wait 8sek. to init ");
    }
        else if(S1inputString == "btoff\r" ) {
              digitalWrite(BT_EN, LOW);
              Serial1.println("Bluetooth is OFF");
    }

    else if(S1inputString == "bton\r" ) {
              digitalWrite(BT_EN, HIGH);
              Serial1.println("Bluetooth is ON");
    }
    else if(S1inputString == "temp\r" ) {
              Serial1.print("Temperature=");
              Serial1.print(sensors.getTempCByIndex(0));
              Serial1.print(" C");
    }
      else if(S1inputString == "sendpos\r" ) {
              locationUpdate();
    }
S1inputString = "";
  }
}


void serialEvent() {
  
  char readbt;

  while (Serial.available())
  {
    readbt = Serial.read();
    S1inputString += readbt;
    if (readbt  == '\r') {
//      if (readbt  == '\9') {
    BTnewData = true;
    }
  }
  if (BTnewData) {
   // Serial.print("String= ");
   // Serial1.println(S1inputString);
  //  Serial.print("char= ");
   // Serial.println(readbt);
    BTnewData = false;
   
   if(S1inputString == "battery\r" ) {
      Serial.print("Battery Voltage= ");
      Serial.print(batteryV);
      Serial.println("V");
      Serial.print("Battery Persentage= ");
      Serial.print(batteryPcnt);
      Serial.println("%");
      Serial.println(sensorValue);
    }


     
    else if(S1inputString == "state_battery\r" ) {
      if(status_stb_ch==0) {
      Serial.println("Charge Termination ON");
    }
    else {
      Serial.println("Charge Termination OFF");
    }
      if(status_chrg_ch==0) {
      Serial.println("Charging");
    }
    else {
      Serial.println("Charged");
    }
    }
// not work
//    else if(S1inputString == "bt_reser\r" ); {
//      digitalWrite(BT_RST, HIGH);
//      Serial.println("TEST");
//      delay(50);
//      digitalWrite(BT_RST, LOW);
//    }

    else if(S1inputString == "gpsoff\r" ) {
              digitalWrite(G_EN, LOW);
              Serial.println("GPS is OFF");
    }

    else if(S1inputString == "gpson\r" ) {
              digitalWrite(G_EN, HIGH);
              Serial.println("GPS is ON Wait 8sek. to init");
    }
        else if(S1inputString == "btoff\r" ) {
              digitalWrite(BT_EN, LOW);
              Serial.println("Bluetooth is OFF");
    }

    else if(S1inputString == "bton\r" ) {
              digitalWrite(BT_EN, HIGH);
              Serial.println("Bluetooth is ON");
    }
    else if(S1inputString == "temp\r" ) {
              Serial.print("Temperature=");
              Serial.print(sensors.getTempCByIndex(0));
              Serial.print(" C");
    }
          else if(S1inputString == "sendpos\r" ) {
              locationUpdate();
    }
S1inputString = "";
  }
}





void aprs_msg_callback(struct AX25Msg *msg) {
}

void locationUpdate() {
  char comment [] = "APRS Tracker";
//   char comment = comment_flex;

  APRS_setLat((char*)deg_to_nmea(lat, true));
  APRS_setLon((char*)deg_to_nmea(lon, false));

  // turn off to stop interrupting tx
  Serial3.end();
  digitalWrite(G_EN, LOW);
  digitalWrite(PTT, HIGH);

  // TX
  APRS_sendLoc(comment, strlen(comment));

  // read TX LED pin and wait till TX has finished (PB5) digital write 13 LED_BUILTIN
  // while(bitRead(PORTB,5));

  // start Serial again
  digitalWrite(PTT, LOW);
  digitalWrite(G_EN, HIGH);
  delay(1000);
  Serial3.begin(9600);
//  comment_flex = "APRS Tracker";
//  Serial.println(comment);

    Serial.print(static_cast<int>(day));
    Serial.print("/");
    Serial.print(static_cast<int>(month));
    Serial.print("/");
    Serial.print(year);
    Serial.print(" ");
    Serial.print(static_cast<int>(hour));
    Serial.print(":");
    Serial.print(static_cast<int>(minute));
    Serial.print(":");
    Serial.print(static_cast<int>(second));
    Serial.print(" ");

    Serial1.print(static_cast<int>(day));
    Serial1.print("/");
    Serial1.print(static_cast<int>(month));
    Serial1.print("/");
    Serial1.print(year);
    Serial1.print(" ");
    Serial1.print(static_cast<int>(hour));
    Serial1.print(":");
    Serial1.print(static_cast<int>(minute));
    Serial1.print(":");
    Serial1.print(static_cast<int>(second));
    Serial1.print(" ");


    Serial.print("LAT=");
    Serial.print(lat);
    Serial.print(" LON=");
    Serial.print(lon);

    Serial1.print("LAT=");
    Serial1.print(lat);
    Serial1.print(" LON=");
    Serial1.print(lon);

    Serial.print(" ");
    Serial.print(deg_to_nmea(lat, true));
    Serial.print("/");

    Serial.println(deg_to_nmea(lon, false));

    Serial1.print(" ");
    Serial1.print(deg_to_nmea(lat, true));
    Serial1.print("/");

    Serial1.println(deg_to_nmea(lon, false));

}

void locationUpdateSOS() {
  char comment [] = "TEST Tracker";

  APRS_setLat((char*)deg_to_nmea(lat, true));
  APRS_setLon((char*)deg_to_nmea(lon, false));

  // turn off SoftSerial to stop interrupting tx
  Serial3.end();
  digitalWrite(G_EN, LOW);

  // TX
  APRS_sendLoc(comment, strlen(comment));

  // read TX LED pin and wait till TX has finished (PB5) digital write 13 LED_BUILTIN
  // while(bitRead(PORTB,5));

  // start SoftSerial again
  digitalWrite(G_EN, HIGH);
  delay(1000);
  Serial3.begin(9600);

  
  Serial.print(static_cast<int>(day));
    Serial.print("/");
    Serial.print(static_cast<int>(month));
    Serial.print("/");
    Serial.print(year);
    Serial.print(" ");
    Serial.print(static_cast<int>(hour));
    Serial.print(":");
    Serial.print(static_cast<int>(minute));
    Serial.print(":");
    Serial.print(static_cast<int>(second));
    Serial.print(" ");

    Serial1.print(static_cast<int>(day));
    Serial1.print("/");
    Serial1.print(static_cast<int>(month));
    Serial1.print("/");
    Serial1.print(year);
    Serial1.print(" ");
    Serial1.print(static_cast<int>(hour));
    Serial1.print(":");
    Serial1.print(static_cast<int>(minute));
    Serial1.print(":");
    Serial1.print(static_cast<int>(second));
    Serial1.print(" ");


    Serial.print("LAT=");
    Serial.print(lat);
    Serial.print(" LON=");
    Serial.print(lon);

    Serial1.print("LAT=");
    Serial1.print(lat);
    Serial1.print(" LON=");
    Serial1.print(lon);

    Serial.print(" ");
    Serial.print(deg_to_nmea(lat, true));
    Serial.print("/");

    Serial.println(deg_to_nmea(lon, false));

    Serial1.print(" ");
    Serial1.print(deg_to_nmea(lat, true));
    Serial1.print("/");

    Serial1.println(deg_to_nmea(lon, false));

  
}


/*
**  Convert degrees in long format to APRS string format
**  DDMM.hhN for latitude and DDDMM.hhW for longitude
**  D is degrees, M is minutes and h is hundredths of minutes.
**  http://www.aprs.net/vm/DOS/PROTOCOL.HTM
*/
char* deg_to_nmea(long deg, boolean is_lat) {
  bool is_negative = 0;
  if (deg < 0) is_negative = 1;

  // Use the absolute number for calculation and update the buffer at the end
  deg = labs(deg);

  unsigned long b = (deg % 1000000UL) * 60UL;
  unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
  b = (b % 1000000UL) / 10000UL;

  conv_buf[0] = '0';
  // in case latitude is a 3 digit number (degrees in long format)
  if ( a > 9999) {
    snprintf(conv_buf , 6, "%04u", a);
  } else snprintf(conv_buf + 1, 5, "%04u", a);

  conv_buf[5] = '.';
  snprintf(conv_buf + 6, 3, "%02u", b);
  conv_buf[9] = '\0';
  if (is_lat) {
    if (is_negative) {
      conv_buf[8] = 'S';
    }
    else conv_buf[8] = 'N';
    return conv_buf + 1;
    // conv_buf +1 because we want to omit the leading zero
  }
  else {
    if (is_negative) {
      conv_buf[8] = 'W';
    }
    else conv_buf[8] = 'E';
    return conv_buf;
  }
}

void setAprsUpdateFlag() {
  send_aprs_update = true;
}
