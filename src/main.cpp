/* -------------------------------------------------------------------------- */
/*                                  Libraries                                 */
/* -------------------------------------------------------------------------- */
#include <Arduino.h>
#include <Elegoo_GFX.h>
#include <Elegoo_TFTLCD.h>
#include <Wire.h>
#include <SPI.h>
#include "DHT.h"
#include <ds3231.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
/* -------------------------------------------------------------------------- */
/*                                    Debug                                   */
/* -------------------------------------------------------------------------- */
#define DEBUG_RTC 0
/* -------------------------------------------------------------------------- */
/*                                 Components                                 */
/* -------------------------------------------------------------------------- */
/* ---------------------------- TFT LCD (Screen) ---------------------------- */
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
/* --------------------------------- Version -------------------------------- */
String version = "2.0";
/* ------------------------ DS3231 (Real Time Clock) ------------------------ */
struct ts t;
/* ----------------------------- Set Actual Time ---------------------------- */
int setHour=12; 
int setMin=13;
int setSec=0;
int setDay=12;
int setMonth=10;
int setYear=2022;
int hour;
int minute;
int seconds;
int day;
int month;
int year;
/* ------------------------- RELAY (AC Power SOURCE) ------------------------ */
#define SOURCE 22
/* ------------------------------ RELAY  (Lamp) ----------------------------- */
#define LAMP 23
volatile byte statusLamp = LOW;
/* ----------------- DHT22 (Humidity and Temperature Sensor) ---------------- */
#define DHTPIN 24
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
/* ------------------------ PAM8406 (Audio Amplifier) ----------------------- */
#define PAM8406 25
/* ------------------------------- Push Button ------------------------------ */
#define pushButton1 26
#define pushButton2 27
volatile byte status1 = LOW;
volatile byte status2 = LOW;
/* --------------------- BARS GND (Floating Ground Bars) -------------------- */
#define firstBAR 28
#define lastBAR 45
/* --------------------- RESISTOR (Capacitor Discharge) --------------------- */
#define RESISTOR 46
unsigned int dischargeTime = 100;
/* --------------------------- Current Calibration -------------------------- */
// #define CALIBRATION 47
#define VOLTAGEREAD A15
volatile float currentValue = 0;
float calibrationResistor = 993 + 270;
/* ------------------------------- PIR Sensors ------------------------------ */
#define PIR1 A12
#define PIR2 A13
#define PIR3 A14
byte statusPIR1 = false;
byte statusPIR2 = false;
byte statusPIR3 = false;
String signalPIR1;
String signalPIR2;
String signalPIR3;
/* ----------------- microSD (Micro SD Card Breakout Board) ----------------- */
#define PIN_SD_CS 53
File fileParams;
File filePIR;
/* -------------------------------------------------------------------------- */
/*                           Expertiment Parameters                           */
/* -------------------------------------------------------------------------- */
/* -------------------------- HTML Form User Input -------------------------- */
String readString;
String experimentAnimalStr;
String dayOfExperimentStr;
String explorationTimeStr;
String toneFrequencyStr;
String toneTimeStr;
String stimulationTimeStr;
String movementAnalysisTimeStr;
String intervalTimeStr;
String numberOfEventsStr;
String experimentContext;
unsigned int explorationTime;
unsigned int toneFrequency;
unsigned int toneTime;
unsigned int stimulationTime;
unsigned int movementAnalysisTime;
unsigned int intervalTime;
int numberOfEvents;
volatile byte confirmed = false;
/* ------------------------ Temperature and Humidity ------------------------ */
float temperature;
float humidity;
/* -------------------------------- Time Data ------------------------------- */
unsigned long experimentStart;
unsigned long experimentEnd;
unsigned long experimentTotalTime;
int dayStart;
int monthStart;
int yearStart;
int hourStart;
int minuteStart;
int secondStart;
int dayEnd;
int monthEnd;
int yearEnd;
int hourEnd;
int minuteEnd;
int secondEnd;
String dateStart;
String timeStart;
String dateEnd;
String timeEnd;
String dataToSave;

/* -------------------------------------------------------------------------- */
/*                                Error Message                               */
/* -------------------------------------------------------------------------- */
void ErrorMessage(){
  tft.setTextColor(RED);
  tft.print("Error");
  tft.println("");
  delay(500);
}

/* -------------------------------------------------------------------------- */
/*                              Success Meessage                              */
/* -------------------------------------------------------------------------- */
void SuccessMessage(){
  tft.setTextColor(GREEN);
  tft.print("Exito");
  tft.println("");
  delay(500);
}
/* -------------------------------------------------------------------------- */
/*                              Validate Modules                              */
/* -------------------------------------------------------------------------- */
void ValidateModules(){
  tft.setTextColor(BLUE);
  tft.println("Validacion de modulos");
  tft.println("");
  tft.setTextColor(BLACK);
  tft.print("SD ..... ");
  /* -------------------------------- SD Module ------------------------------- */
  if (!SD.begin(PIN_SD_CS)){
    ErrorMessage();
    while (1); // If there's an error with the SD, the execution of the script stops.
  }
  else{
    SuccessMessage();
  }
  /* ------------------------------- RTC Module ------------------------------- */
  tft.setTextColor(BLACK);
  tft.print("RTC ..... ");
  DS3231_get(&t);
  hour = t.hour;
  if(hour == 0){
    ErrorMessage();
    while (1);
  }
  else{
    SuccessMessage();
  }
  
  /* ------------------------------- DHT Module ------------------------------- */
  tft.setTextColor(BLACK);
  tft.print("DHT ..... ");
  if(isnan(dht.readTemperature()) && isnan(dht.readHumidity())){
    ErrorMessage();
    //while (1);
  }
  else{
    SuccessMessage();
  }
  /* ------------------------------- PIR Sensors ------------------------------ */
  tft.setTextColor(BLACK);
  tft.print("PIR1 ..... ");

  for (int i = 0; i<20000; i++){
    if(analogRead(PIR1) * float(5) / (float(1023)) > float(4.1)){
      statusPIR1 = true;
    }
    if(analogRead(PIR2) * float(5) / (float(1023)) > float(4.1)){
      statusPIR2 = true;
    }
    if(analogRead(PIR3) * float(5) / (float(1023)) > float(4.1)){
      statusPIR3 = true;
    }
  }
  // PIR 1
  if(statusPIR1 == true){
    ErrorMessage();
    //while (1);
  }
  else{
    SuccessMessage();
  }
  // PIR 2
   tft.setTextColor(BLACK);
  tft.print("PIR2 ..... ");
  if(statusPIR2 == true){
    ErrorMessage();
    //while (1);
  }
  else{
    SuccessMessage();
  }
  // PIR 3
   tft.setTextColor(BLACK);
  tft.print("PIR3 ..... ");
  if(statusPIR3 == true){
    ErrorMessage();
    //while (1);
  }
  else{
    SuccessMessage();
  }
  tft.println("");
  tft.setTextColor(RED);
  tft.println("Presiona 2 para confirmar el correcto funcionamiento del modulo");
  tft.println("");
  /* ------------------------------- SPKR Module ------------------------------ */
  status2 = LOW;
  tft.setTextColor(BLACK);
  tft.print("SPKR ..... ");
  while(status2 == LOW){
    status2 = digitalRead(pushButton2);
    tone(PAM8406, 4000);
  }
  noTone(PAM8406);
  SuccessMessage();
  /* ------------------------------- Lamp Module ------------------------------ */
  status2 = LOW;
  tft.setTextColor(BLACK);
  tft.print("LAMP ..... ");
  while(status2 == LOW){
    status2 = digitalRead(pushButton2);
    digitalWrite(LAMP, HIGH);
  }
  digitalWrite(LAMP, LOW);
  SuccessMessage();
  status2 = LOW;
  delay(3000);
}
/* -------------------------------------------------------------------------- */
/*                                Screen Header                               */
/* -------------------------------------------------------------------------- */
void HeaderScreen(){
  tft.setRotation(3);
  tft.setTextSize(1);
  tft.setCursor(0,0);
  tft.fillScreen(WHITE);
  tft.setTextColor(BLUE);
  tft.println("");
  tft.println("Camara de Condicionamiento Operante v" + version);
  tft.setTextColor(BLACK);
  tft.println("");
  tft.setTextSize(1);

}
/* -------------------------------------------------------------------------- */
/*                                 Main Screen                                */
/* -------------------------------------------------------------------------- */
void MainScreen(){
  HeaderScreen();
  tft.println("Para configurar los parametros del experimento, conecta tu  dispositivo movil a la red inalambrica y accede en tu navegador a la direccion IP indicada.");
  tft.println("");
  tft.println("");
  tft.setTextColor(BLUE);
  tft.setTextSize(2);
  tft.println("SSID: Consola-CCO");
  tft.println("KEY: ciencias123");
  tft.println("IP: 192.168.4.1");
  tft.setTextSize(1);
  tft.setTextColor(BLACK);
  tft.println("");
  tft.println("");
  tft.println("Presiona 1 para calibrar la corriente de shock");
  tft.println("");
  tft.println("Presiona 2 para activar la lampara");
}
/* -------------------------------------------------------------------------- */
/*                                Start Screen                                */
/* -------------------------------------------------------------------------- */
void StartScreen(){
  HeaderScreen();
  tft.println("Laboratorio de Neurobiologia de la Adiccion y Plasticidad Cerebral");
  tft.println("");
  tft.println("Facultad de Ciecias, UAEMex");
  tft.println("");
  tft.println("");
  ValidateModules();
}
/* -------------------------------------------------------------------------- */
/*                          Temperature and Humidity                          */
/* -------------------------------------------------------------------------- */
void ReadDTH(){
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}
/* -------------------------------------------------------------------------- */
/*                                Date and Time                               */
/* -------------------------------------------------------------------------- */
void GetClock(){
  DS3231_get(&t);
  hour = t.hour;
  minute = t.min;
  seconds = t.sec;
  day = t.mday;
  month = t.mon;
  year = t.year;
}

/* -------------------------------------------------------------------------- */
/*                             Capacitor Discharge                            */
/* -------------------------------------------------------------------------- */
void Discharge(){
  digitalWrite(SOURCE, LOW);
  unsigned long timeLimit = dischargeTime * 1000;
  unsigned long startMillis = millis();
  tft.setTextColor(RED);
  tft.println("");
  tft.println("Por seguridad, espera en lo que se descarga el capacitor.");
  while(millis() - startMillis < (timeLimit)){
    digitalWrite(RESISTOR, HIGH);
  }
  digitalWrite(RESISTOR, LOW);
}

/* -------------------------------------------------------------------------- */
/*                          Save Data to microSD card                         */

/*void SaveData(){
  HeaderScreen();
  tft.setTextColor(BLUE);
  tft.println("Almacenando los datos en la tarjeta SD");
  tft.println("");

  fileParams = SD.open("params.txt", FILE_WRITE);
  delay(200);
  dateStart = String(dayStart) + "/" +  String(monthStart) + "/" + String(yearStart);
  timeStart = String(hourStart) + ":" + String(minuteStart) + ":" + String(secondStart);
  dateEnd = String(dayEnd) + "/" + String(monthEnd) + "/" + String(yearEnd);
  timeEnd = String(hourEnd) + ":" + String(minuteEnd) + ":" + String(secondEnd);
  dataToSave = dayOfExperimentStr + "," + experimentAnimalStr + "," + dateStart + "," + timeStart + "," + dateEnd + "," + timeEnd + "," + experimentTotalTime + "," + temperature + "," + humidity + "," + explorationTimeStr + "," + toneFrequencyStr + "," + toneTimeStr + "," + stimulationTimeStr + "," + movementAnalysisTimeStr + "," + intervalTimeStr + "," + numberOfEvents + "," + experimentContext;
  fileParams.println(dataToSave);
  fileParams.close();
  tft.setTextColor(BLACK);
  tft.print("Parametros del experimento ... ");
  SuccessMessage();

  if(statusPIR1 != true){
    filePIR1 = SD.open("pir1.txt", FILE_WRITE);
    delay(200);
    filePIR1.println(signalPIR1);
    filePIR1.close();
    delay(200);
    tft.setTextColor(BLACK);
    tft.print("PIR 1 ... ");
    SuccessMessage();
  }

  if(statusPIR2 != true){
    filePIR2 = SD.open("pir2.txt", FILE_WRITE);
    delay(200);
    filePIR2.println(signalPIR1);
    filePIR2.close();
    delay(200);
    tft.setTextColor(BLACK);
    tft.print("PIR 2 ... ");
    SuccessMessage();
  }

  if(statusPIR3 != true){
    filePIR3 = SD.open("pir3.txt", FILE_WRITE);
    delay(200);
    filePIR3.println(signalPIR1);
    filePIR3.close();
    delay(200);
    tft.setTextColor(BLACK);
    tft.print("PIR 3 ... ");
    SuccessMessage();
  }

  tft.println("");
  tft.setTextColor(BLACK);
  tft.println("Los datos se han almacenado correctamente");
  delay(3000);
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                              Exploration Time                              */
/* -------------------------------------------------------------------------- */
void Exploration(){
  unsigned long timeLimit = intervalTime;
  unsigned long startMillis = millis();
  tft.setTextColor(BLACK);
  tft.print("1. Exploracion del entorno ..... ");
  while(millis() - startMillis < (timeLimit)){
  
  }
  SuccessMessage();
}

/* -------------------------------------------------------------------------- */
/*                                Tone Stimulus                               */
/* -------------------------------------------------------------------------- */
void ToneActivation(){
  unsigned int toneFreq = toneFrequency;
  unsigned long timeLimit = toneTime;
  unsigned long startMillis = millis();
  tft.setTextColor(BLACK);
  tft.print("2. Estimulacion auditiva ..... ");
  while(millis() - startMillis < (timeLimit)){
    tone(PAM8406, toneFreq);
  }
  SuccessMessage();
  noTone(PAM8406);
}

/* -------------------------------------------------------------------------- */
/*                       Floating Ground Bars Activation                      */
/* -------------------------------------------------------------------------- */
void Stimulus(){
  unsigned long timeLimit = stimulationTime;
  unsigned long startMillis = millis();
  tft.setTextColor(BLACK);
  tft.print("3. Estimulacion electrica ..... ");
  digitalWrite(RESISTOR, LOW);
  digitalWrite(SOURCE, HIGH);
  while(millis() - startMillis < (timeLimit)){
      for(int i=firstBAR; i<=lastBAR; i++)
        {
          digitalWrite(i, HIGH);
          delay(5);
          digitalWrite(i, LOW);
        }
  }
  for(int i=firstBAR; i<=lastBAR; i++)
  {
    digitalWrite(i, LOW);
  }
  SuccessMessage();
  digitalWrite(SOURCE, LOW);
  digitalWrite(RESISTOR, HIGH);
}

/* -------------------------------------------------------------------------- */
/*                              Motion Detection                              */
/* -------------------------------------------------------------------------- */
void MotionDetection(){
  unsigned long timeLimit = movementAnalysisTime;
  unsigned long startMillis = millis();
  float voltagePIR1; 
  float voltagePIR2; 
  float voltagePIR3;
  tft.setTextColor(BLACK);
  tft.print("4. Deteccion de movimiento ..... ");
  while(millis() - startMillis < (timeLimit)){
    if(statusPIR1 != true){
      voltagePIR1 = analogRead(PIR1) * float(5) / (float(1023));
      filePIR.print(String(voltagePIR1) + ",");
    }
    if(statusPIR2 != true){
      voltagePIR2 = analogRead(PIR2) * float(5) / (float(1023));
      filePIR.print(String(voltagePIR2) + ",");
    }
    if(statusPIR3 != true){
      voltagePIR3 = analogRead(PIR3) * float(5) / (float(1023));
      filePIR.print(String(voltagePIR3) + ";");
    }
  }
  SuccessMessage();
}

/* -------------------------------------------------------------------------- */
/*                         Waiting Time Between Events                        */
/* -------------------------------------------------------------------------- */
void Wait(){
  unsigned long timeLimit = intervalTime;
  unsigned long startMillis = millis();
  tft.setTextColor(BLACK);
  tft.print("5. Intervalo de espera ..... ");
  while(millis() - startMillis < (timeLimit)){
    
  }
  SuccessMessage();
}

/* -------------------------------------------------------------------------- */
/*                    String to Integer variable conversion                   */
/* -------------------------------------------------------------------------- */
void VariableConversion(){
  toneFrequency = toneFrequencyStr.toInt();
  toneTime = toneTimeStr.toInt() * 1000;
  stimulationTime = stimulationTimeStr.toInt() * 1000;
  movementAnalysisTime = movementAnalysisTimeStr.toInt() * 1000;
  intervalTime = intervalTimeStr.toInt() * 1000;
  numberOfEvents = numberOfEventsStr.toInt();
}

/* -------------------------------------------------------------------------- */
/*                            Full Experiment Event                           */
/* -------------------------------------------------------------------------- */
void Experiment(){
  ReadDTH();
  /* -------------------------- Data Type Conversion -------------------------- */
  VariableConversion();
  /* ----------------------- Experiment Start Clock Data ---------------------- */
  GetClock();
  dayStart = day;
  monthStart = month;
  yearStart = year;
  hourStart = hour;
  minuteStart = minute;
  secondStart = seconds;
  /* --------------- Repetition of events during the experiment --------------- */
  filePIR = SD.open("pir.txt", FILE_WRITE);
  filePIR.print("|");
  if (filePIR){
    experimentStart = millis();
    for(int i= 1; i<=numberOfEvents; i++){
      HeaderScreen();
      filePIR.print("e" + String(i) + ",");
      tft.setTextColor(BLACK);
      tft.println("Iniciando el evento " + String(i));
      tft.println("");
      tft.println("");
      tft.setTextColor(BLACK);
      Exploration();
      ToneActivation();
      Stimulus();
      MotionDetection();
      Wait();
      tft.println("");
      tft.println("");
      tft.setTextColor(RED);
      tft.println("Finalizo el evento " + String(i));
      delay(3000);
    }
    filePIR.close();
  }
  else{
    tft.setTextColor(RED);
    tft.println("Error al amacenar parametros en SD");
  }
  experimentEnd = millis();
  experimentTotalTime = (experimentEnd - experimentStart) / 1000;
  /* ------------------------ Experiment End Clock Data ----------------------- */
  GetClock();
  dayEnd = day;
  monthEnd = month;
  yearEnd = year;
  hourEnd = hour;
  minuteEnd = minute;
  secondEnd = seconds;
  /* --------------------- Save data into the microSD card -------------------- */
  //SaveData(); // Call to function is avoided due to errors in writing data to SD
  dateStart = String(dayStart) + "/" +  String(monthStart) + "/" + String(yearStart);
  timeStart = String(hourStart) + ":" + String(minuteStart) + ":" + String(secondStart);
  dateEnd = String(dayEnd) + "/" + String(monthEnd) + "/" + String(yearEnd);
  timeEnd = String(hourEnd) + ":" + String(minuteEnd) + ":" + String(secondEnd);
  fileParams = SD.open("params.txt", FILE_WRITE);
  if(fileParams){
    fileParams.print(String(temperature) + ",");
    fileParams.print(String(humidity) + ",");
    fileParams.print(dateStart + ",");
    fileParams.print(timeStart + ",");
    fileParams.print(dateEnd + ",");
    fileParams.print(timeEnd);
    fileParams.close();
  }
  else{
    tft.setTextColor(RED);
    tft.println("Error al amacenar parametros en SD");
  }
  /* ----------------------------- Reset variables ---------------------------- */
  status1 = LOW;
  status2 = LOW;
  signalPIR1 = "";
  signalPIR2 = "";
  signalPIR3 = "";
  /* ------------------------------- Main Screen ------------------------------ */
  MainScreen();
}

/* -------------------------------------------------------------------------- */
/*                              Current Reading                               */
/* -------------------------------------------------------------------------- */
void Calibration(){
  status2 = LOW;
  digitalWrite(SOURCE, HIGH);
  digitalWrite(RESISTOR, LOW);
  int voltageReading;
  HeaderScreen();
  tft.println("Estimacion de la corriente de estimulacion");
  tft.println("");
  tft.setTextColor(BLACK);
  tft.println("Asegurate de conectar correctamente los cables banana-caiman al animal antes de iniciar la calibracion.");
  tft.setTextColor(RED);
  tft.println("");
  tft.println("Presiona 2 para establecer el valor de la corriente");
  tft.println("");
  tft.println("");
  tft.setTextSize(2);
  while(status2 == LOW){
    status2 = digitalRead(pushButton2);
    voltageReading = analogRead(VOLTAGEREAD);
    currentValue = ((voltageReading * float(5) / (float(1023) * calibrationResistor)));
    tft.setTextColor(BLUE, WHITE);
    tft.setCursor(0,80);
    tft.println("Corriente: " + String(currentValue*1000)+ " mA");
  }
  tft.setTextSize(1);
  tft.println("");
  Discharge();
  status1 = LOW;
  status2 = LOW;
}

/* -------------------------------------------------------------------------- */
/*                                    UART                                    */
/* -------------------------------------------------------------------------- */
void UART(){
  
  if(Serial1.available()){

    int ind1;
    int ind2;
    int ind3;
    int ind4;
    int ind5;
    int ind6;
    int ind7;
    int ind8;
    int ind9;

    readString = Serial1.readString();
    Serial.println(readString);
    if(readString){

      confirmed = true;

      HeaderScreen();
      tft.setTextColor(BLUE);
      tft.println("Parametros del protocolo de condicionamiento");
      tft.println("");
      /* -------------------------- Split received string ------------------------- */
      ind1 = readString.indexOf(','); 
      dayOfExperimentStr = readString.substring(0, ind1); 
      ind2 = readString.indexOf(',', ind1+1 );
      experimentAnimalStr = readString.substring(ind1+1, ind2);
      ind3 = readString.indexOf(',', ind2+1 );
      explorationTimeStr = readString.substring(ind2+1, ind3);
      ind4 = readString.indexOf(',', ind3+1 );
      toneFrequencyStr = readString.substring(ind3+1, ind4);
      ind5 = readString.indexOf(',', ind4+1 );
      toneTimeStr = readString.substring(ind4+1, ind5);
      ind6 = readString.indexOf(',', ind5+1 );
      stimulationTimeStr = readString.substring(ind5+1, ind6);
      ind7 = readString.indexOf(',', ind6+1 );
      movementAnalysisTimeStr = readString.substring(ind6+1, ind7);
      ind8 = readString.indexOf(',', ind7+1 );
      intervalTimeStr = readString.substring(ind7+1, ind8);
      ind9 = readString.indexOf(',', ind8+1 );
      numberOfEventsStr = readString.substring(ind8+1, ind9);
      experimentContext = readString.substring(ind9+1);

        if(dayOfExperimentStr == "0" || experimentAnimalStr == "0" || explorationTimeStr == "0" || toneFrequencyStr == "0" || toneTimeStr == "0" || stimulationTimeStr == "0" || movementAnalysisTimeStr == "0" || intervalTimeStr == "0" || numberOfEventsStr == "0" || experimentContext == "0"){
          tft.setTextColor(RED);
          tft.println("Se ingreso algun parametro de forma incorrecta, verifica tus datos y repite la operacion");
          confirmed = false;
        }
        else{
          /* ----------------------- Display received parameters ---------------------- */
          tft.setTextColor(BLACK);
          tft.println("Dia: " + dayOfExperimentStr);
          tft.println("ID del animal: " + experimentAnimalStr);
          tft.println("Tiempo de exploracion: " + explorationTimeStr + " s");
          tft.println("Frecuencia de tono: " + toneFrequencyStr + " Hz");
          tft.println("Tiempo de tono: " + toneTimeStr + " s");
          tft.println("Tiempo de shock: " + stimulationTimeStr + " s");
          tft.println("Tiempo de analisis: " + movementAnalysisTimeStr + " s");
          tft.println("Tiempo de intervalo: " + intervalTimeStr + " s");
          tft.println("Repeticiones del evento: " + numberOfEventsStr);
          tft.println("Contexto del experimento: " + experimentContext);
          confirmed = true;
          fileParams = SD.open("params.txt", FILE_WRITE);
          if(fileParams){
            fileParams.print("|");
            fileParams.print(dayOfExperimentStr + ",");
            fileParams.print(experimentAnimalStr + ",");
            fileParams.print(explorationTimeStr + ",");
            fileParams.print(toneFrequencyStr + ",");
            fileParams.print(toneTimeStr + ",");
            fileParams.print(stimulationTimeStr + ",");
            fileParams.print(movementAnalysisTimeStr + ",");
            fileParams.print(intervalTimeStr + ",");
            fileParams.print(numberOfEventsStr + ",");
            fileParams.print(experimentContext+",");
            fileParams.close();
          }
          else{
            tft.setTextColor(RED);
            tft.println("Error al amacenar parametros en SD");
          }
        }
    }

    tft.println("");
    tft.println("");
    tft.setTextColor(RED);
    tft.println("Presiona 2 para continuar");
    while (status2 == LOW) {
      status2 = digitalRead(pushButton2);
    }
    status2 = LOW;
  }
}

/* -------------------------------------------------------------------------- */
/*                                Arduino Setup                               */
/* -------------------------------------------------------------------------- */
void setup() {

  /* -------------------------- Serial Communication -------------------------- */
  Serial.begin(115200); // Arduino Mega2560
  Serial1.begin(115200); // ESP8266
  pinMode(PIN_SD_CS, OUTPUT);
  /* ---------------------------- IC2 Communication --------------------------- */
  Wire.begin(); 
  /* -------------------------------- LCD Setup ------------------------------- */
  tft.reset();
  tft.begin(0x9341);
  /* -------------------------------- Pin Setup ------------------------------- */
  pinMode(SOURCE, OUTPUT); // Power Source Relay
  digitalWrite(SOURCE, LOW); // Relay is set to OFF
  pinMode(LAMP, OUTPUT); // Lamp SOURCE Relay
  digitalWrite(LAMP, statusLamp); // Relay is set to OFF
  pinMode(PAM8406, OUTPUT); // Audio Amplifier 
  pinMode(DHTPIN, INPUT); // DHT-22 Sensor
  pinMode(PIR1, INPUT); // PIR 1
  pinMode(PIR2, INPUT); // PIR 2
  pinMode(PIR3, INPUT); // PIR 3
  pinMode(RESISTOR, OUTPUT); // Cement Resistor Optocoupler
  digitalWrite(RESISTOR, HIGH);  // 19th optocoupler is set to ON
  // There is an error regarding the 20th optocoupler configuration
  // using  the banana jack connectors, so it's discarded.
  // pinMode(CALIBRATION, OUTPUT); 
  // digitalWrite(CALIBRATION, LOW);
  pinMode(VOLTAGEREAD, INPUT); // A15 is used to read calibration voltage and estimate the current 
  pinMode(pushButton1, INPUT); // Button 1
  pinMode(pushButton2, INPUT); // Button 2
  // 1st - 18th optocouplers pins are configured
  for(int i=firstBAR; i<=lastBAR; i++)
  {
    pinMode(i, OUTPUT);
  }
  /* ------------------------------ DS3231 Setup ------------------------------ */
  DS3231_init(DS3231_CONTROL_INTCN);
  if(DEBUG_RTC == 1){
    t.hour=setHour; 
    t.min=setMin;
    t.sec=setSec;
    t.mday=setDay;
    t.mon=setMonth;
    t.year=setYear;
    DS3231_set(t);
  }
  /* ------------------------------- DTH22 Setup ------------------------------ */
  dht.begin();
  /* --------------------------------- Screens -------------------------------- */
  StartScreen();
  delay(50);
  MainScreen();
}

/* -------------------------------------------------------------------------- */
/*                                    Main                                    */
/* -------------------------------------------------------------------------- */
void loop() {

    UART();
    if(confirmed == true){
      Experiment();
      confirmed = false;
    }
    digitalWrite(RESISTOR, HIGH);
    status1 = digitalRead(pushButton1);
    if(status1 == HIGH)
    {
      Calibration();
      digitalWrite(SOURCE, LOW);
      MainScreen();
      digitalWrite(RESISTOR, HIGH);
    }
    status2 = digitalRead(pushButton2);
    if(status2 == HIGH){
      statusLamp = !statusLamp;
      digitalWrite(LAMP, statusLamp);
      delay(100);
      status2 = LOW;
      
    }
}