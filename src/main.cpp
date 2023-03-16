#include <Arduino.h>
#include <Elegoo_GFX.h>
#include <Elegoo_TFTLCD.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <DS3231.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <TimerOne.h>
#include <PriUint64.h>
/* -------------------------------------------------------------------------- */
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
String firmware_version = "2.0";
#define DC_170VDC_SOURCE 22
#define LAMP_120VAC_SOURCE 23
#define AUDIO_AMPLIFIER_PIN 25
#define PUSH_BUTTON_1_PIN 26
#define PUSH_BUTTON_2_PIN 27
volatile byte push_button_1_status = LOW;
volatile byte push_button_2_status = LOW;
#define FIRST_BAR_PIN 28
#define LAST_BAR_PIN 45
#define UNLOAD_RESISTOR_PIN 46
uint8_t capacitor_discharge_time_in_seconds = 60;
#define SHOCK_CURRENT_ESTIMATION_ADC_PIN A15
volatile uint32_t shock_current_estimation = 0;
float calibration_voltage_divider_resistor = 993 + 270;
#define PIR_1_ADC_PIN A12
#define PIR_2_ADC_PIN A13
#define PIR_3_ADC_PIN A14
#define SD_CS_PIN 53

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

File file_experiment_configuration;
File file_pir_samples;

String received_str;
String experiment_day_number_str;
String animal_id_srt;
String exploration_time_str;
String tone_frequency_str;
String tone_time_srt;
String shock_time_str;
String motion_recording_time_str;
String between_events_time_str;
String number_of_total_events_str;
String experiment_context;

volatile bool esp8266_sent_configuration_data = false;

uint64_t experiment_total_millis;
/* -------------------------------------------------------------------------- */
#define DHT_PIN 24
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
float temperature, humidity;
void retrieve_dht_data()
{
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}
/* -------------------------------------------------------------------------- */
DS3231 rtc;
// #define DEBUG_RTC
#ifdef DEBUG_RTC
uint8_t set_hour = 0;
uint8_t set_minute = 55;
uint8_t set_second = 0;
uint8_t set_day = 16;
uint8_t set_month = 3;
uint16_t set_year = 23;
void set_rtc_date_and_time()
{
  rtc.setClockMode(false);
  rtc.setHour(set_hour);
  rtc.setMinute(set_minute);
  rtc.setSecond(set_second);
  rtc.setDate(set_day);
  rtc.setMonth(set_month);
  rtc.setYear(set_year);
}
#endif
/* -------------------------------------------------------------------------- */
byte hour, minute, seconds, day, month, year;
bool century_bit, h12, hPM;
void retrieve_rtc_date_and_time()
{
  hour = rtc.getHour(h12, hPM);
  minute = rtc.getMinute();
  seconds = rtc.getSecond();
  day = rtc.getDate();
  month = rtc.getMonth(century_bit);
  year = rtc.getYear();
}
/* -------------------------------------------------------------------------- */
String current_path;
void new_directory_path()
{
  String parent_path = String(day) + String(month) + String(year) + "/";
  String child_path = String(hour) + String(minute) + String(seconds) + "/";
  current_path = parent_path + child_path;
  SD.mkdir((char *)current_path.c_str());
}
/* -------------------------------------------------------------------------- */
void tft_error_flag()
{
  tft.setTextColor(RED);
  tft.println("ERROR");
  tft.setTextColor(BLACK);
}
/* -------------------------------------------------------------------------- */
void tft_success_flag()
{
  tft.setTextColor(GREEN);
  tft.println("EXITO");
  tft.setTextColor(BLACK);
}
/* -------------------------------------------------------------------------- */
void initialize_external_modules()
{
  tft.setTextColor(BLUE);
  tft.println("Inicializando modulos externos");
  tft.setTextColor(BLACK);
  tft.print("SD ..... ");
  if (!SD.begin(SD_CS_PIN))
  {
    tft_error_flag();
    while (1)
      ;
  }
  else
  {
    tft_success_flag();
  }
  tft.print("DHT ..... ");
  if (isnan(dht.readTemperature()) && isnan(dht.readHumidity()))
  {
    tft_error_flag();
    while (1)
      ;
  }
  else
  {
    tft_success_flag();
  }
  tft.setTextColor(BLUE);
  tft.println("");
  tft.println("Presiona 2 para confirmar el funcionamiento correcto");
  tft.println("");
  tft.setTextColor(BLACK);
  push_button_2_status = LOW;
  tft.print("SPKR ..... ");
  while (push_button_2_status == LOW)
  {
    push_button_2_status = digitalRead(PUSH_BUTTON_2_PIN);
    tone(AUDIO_AMPLIFIER_PIN, 4000);
  }
  noTone(AUDIO_AMPLIFIER_PIN);
  tft_success_flag();

  push_button_2_status = LOW;
  delay(1000);
  tft.print("LAMP ..... ");
  while (push_button_2_status == LOW)
  {
    push_button_2_status = digitalRead(PUSH_BUTTON_2_PIN);
    digitalWrite(LAMP_120VAC_SOURCE, HIGH);
  }
  digitalWrite(LAMP_120VAC_SOURCE, LOW);
  tft_success_flag();
  push_button_2_status = LOW;
  delay(1000);
}
/* -------------------------------------------------------------------------- */
void tft_header()
{
  tft.setRotation(3);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.fillScreen(WHITE);
  tft.setTextColor(BLUE);
  tft.println("");
  tft.println("Camara de Condicionamiento Operante v" + firmware_version);
  tft.setTextColor(BLACK);
  tft.println("");
  tft.setTextSize(1);
}
/* -------------------------------------------------------------------------- */
void tft_main()
{
  tft_header();
  tft.println("Usa tu dispositivo para conectarte al dispositivo");
  tft.println("y configurar los parametros del experimento usando");
  tft.println("el formulario");
  tft.println("");
  tft.println("");
  tft.setTextColor(BLUE);
  tft.setTextSize(2);
  tft.println("SSID: Consola-CCO");
  tft.println("PWD: ciencias123");
  tft.println("IP: 192.168.4.1");
  tft.setTextSize(1);
  tft.setTextColor(BLACK);
  tft.println("");
  tft.println("Presiona 1 para comenzar el proceso de calibracion");
  tft.println("de la corriente de shock");
  tft.println("");
  tft.println("Presiona 2 para encender la lampara");
}
/* -------------------------------------------------------------------------- */
void tft_power_on()
{
  tft_header();
  tft.println("Universidad Autonoma del Estado de Mexico");
  tft.println("");
  initialize_external_modules();
}
/* -------------------------------------------------------------------------- */
uint64_t to_mills(uint8_t value)
{
  return (uint64_t)value * (uint64_t)1000;
}
/* -------------------------------------------------------------------------- */
void capacitor_discharge()
{
  digitalWrite(DC_170VDC_SOURCE, LOW);
  digitalWrite(UNLOAD_RESISTOR_PIN, HIGH);
  tft.setTextColor(RED);
  tft.println("");
  tft.println("Por seguridad, el capacitor se esta descargando.");
  tft.println("Espera 60 segundos.");
  delay(to_mills(capacitor_discharge_time_in_seconds));
}
/* -------------------------------------------------------------------------- */
void experiment_exploration_event()
{
  uint64_t limit_time = to_mills(exploration_time_str.toInt());
  uint64_t start_millis = millis();
  tft.setTextColor(BLACK);
  tft.print("1. Exploracion del ambiente ..... ");
  while ((millis() - start_millis) <= limit_time)
  {
    Serial.println(".");
  }
  tft_success_flag();
}
/* -------------------------------------------------------------------------- */
void experiment_tone_event()
{
  uint64_t tone_frequency = tone_frequency_str.toInt();
  uint64_t limit_time = to_mills(tone_time_srt.toInt());
  uint64_t start_millis = millis();
  tft.setTextColor(BLACK);
  tft.print("2. Estimulacion auditiva ..... ");
  while ((millis() - start_millis) <= limit_time)
  {
    tone(AUDIO_AMPLIFIER_PIN, tone_frequency);
  }
  tft_success_flag();
  noTone(AUDIO_AMPLIFIER_PIN);
}
/* -------------------------------------------------------------------------- */
void experiment_shock_event()
{
  uint64_t limit_time = to_mills(shock_time_str.toInt());
  uint64_t start_millis = millis();
  tft.setTextColor(BLACK);
  tft.print("3. Estimulacion electrica ..... ");
  digitalWrite(UNLOAD_RESISTOR_PIN, LOW);
  digitalWrite(DC_170VDC_SOURCE, HIGH);
  while ((millis() - start_millis) <= limit_time)
  {
    for (int bar = FIRST_BAR_PIN; bar <= LAST_BAR_PIN; bar++)
    {
      digitalWrite(bar, HIGH);
      delay(5);
      digitalWrite(bar, LOW);
    }
  }
  for (int bar = FIRST_BAR_PIN; bar <= LAST_BAR_PIN; bar++)
  {
    digitalWrite(bar, LOW);
  }
  digitalWrite(DC_170VDC_SOURCE, LOW);
  digitalWrite(UNLOAD_RESISTOR_PIN, HIGH);
  tft_success_flag();
}
/* -------------------------------------------------------------------------- */
uint64_t current_sample, max_samples;
float interrupt_timer_in_microseconds;
uint64_t data_logger_frequency_in_hz = 100;
unsigned long startMillis, stopMillis, totalMillis;
void read_analog_pir_sample_and_save_to_sd()
{
  int sensor_reading_pir_1 = analogRead(PIR_1_ADC_PIN);
  int sensor_reading_pir_2 = analogRead(PIR_2_ADC_PIN);
  int sensor_reading_pir_3 = analogRead(PIR_3_ADC_PIN);
  file_pir_samples.print(sensor_reading_pir_1);
  file_pir_samples.print(",");
  file_pir_samples.print(sensor_reading_pir_2);
  file_pir_samples.print(",");
  file_pir_samples.println(sensor_reading_pir_3);
  current_sample++;
}
/* -------------------------------------------------------------------------- */
void experiment_motion_recording_event(int current_event)
{
  uint64_t motion_recording_time = motion_recording_time_str.toInt();

  tft.setTextColor(BLACK);
  tft.print("4. Registro de moviento ..... ");

  current_sample = 0;
  max_samples = (motion_recording_time)*data_logger_frequency_in_hz;
  interrupt_timer_in_microseconds = float(1) / float(data_logger_frequency_in_hz);
  interrupt_timer_in_microseconds = interrupt_timer_in_microseconds * float(1000000);
  String new_recording_file = current_path + "pir" + String(current_event) + ".txt";
  file_pir_samples = SD.open(new_recording_file.c_str(), FILE_WRITE);

  if (file_pir_samples)
  {
    Timer1.initialize((uint64_t)interrupt_timer_in_microseconds);
    Timer1.attachInterrupt(read_analog_pir_sample_and_save_to_sd);
    while (current_sample <= max_samples - 1)
    {
      Serial.println(PriUint64<DEC>(current_sample));
    }
    Timer1.detachInterrupt();
    file_pir_samples.close();
    tft_success_flag();
  }
  else
  {
    tft_error_flag();
  }
}
/* -------------------------------------------------------------------------- */
void experiment_between_event()
{
  uint64_t limit_time = to_mills(between_events_time_str.toInt());
  uint64_t start_millis = millis();
  tft.setTextColor(BLACK);
  tft.print("5. Espera entre eventos ..... ");
  while ((millis() - start_millis) <= limit_time)
  {
    Serial.println(".");
  }
  tft_success_flag();
}
/* -------------------------------------------------------------------------- */
void save_experiment_configuration_data_to_sd()
{
  String new_experiment = current_path + "config.txt";
  file_experiment_configuration = SD.open(new_experiment.c_str(), FILE_WRITE);
  if (file_experiment_configuration)
  {
    file_experiment_configuration.print(hour);
    file_experiment_configuration.print(",");
    file_experiment_configuration.print(minute);
    file_experiment_configuration.print(",");
    file_experiment_configuration.print(seconds);
    file_experiment_configuration.print(",");
    file_experiment_configuration.print(day);
    file_experiment_configuration.print(",");
    file_experiment_configuration.print(month);
    file_experiment_configuration.print(",");
    file_experiment_configuration.print(year);
    file_experiment_configuration.print(",");
    file_experiment_configuration.print(PriUint64<DEC>(experiment_total_millis));
    file_experiment_configuration.print(",");
    file_experiment_configuration.print(received_str);
    file_experiment_configuration.close();
  }
}
/* -------------------------------------------------------------------------- */
void full_experiment()
{
  retrieve_rtc_date_and_time();
  retrieve_dht_data();
  new_directory_path();
  uint64_t experiment_start_millis = millis();
  uint8_t number_of_total_events = number_of_total_events_str.toInt();
  for (int current_event = 1; current_event <= number_of_total_events; current_event++)
  {
    tft_header();
    tft.println("Iniciando el evento " + String(current_event));
    tft.println("");
    experiment_exploration_event();
    experiment_tone_event();
    experiment_shock_event();
    experiment_motion_recording_event(current_event);
    experiment_between_event();
    tft.println("");
    tft.setTextColor(BLUE);
    tft.println("Finalizo el evento " + String(current_event));
  }
  digitalWrite(LAMP_120VAC_SOURCE, LOW);
  uint64_t experiment_end_millis = millis();
  experiment_total_millis = (experiment_end_millis - experiment_start_millis);
  save_experiment_configuration_data_to_sd();
  tft_main();
}
/* -------------------------------------------------------------------------- */
void shock_current_calibration()
{
  push_button_1_status = LOW;
  digitalWrite(UNLOAD_RESISTOR_PIN, LOW);
  digitalWrite(DC_170VDC_SOURCE, HIGH);
  tft_header();
  tft.println("Estimacion de la corriente de estimulacion");
  tft.println("");
  tft.println("Asegurate de conectar correctamente los cables");
  tft.println("banana-caiman al animal antes de iniciar.");
  tft.setTextColor(BLUE);
  tft.println("");
  tft.println("Presiona 2 para establecer el valor de la corriente");
  tft.println("");
  tft.setTextSize(2);
  while (push_button_2_status == LOW)
  {
    push_button_2_status = digitalRead(PUSH_BUTTON_2_PIN);
    uint8_t adc_voltage_reading_current_estimation = analogRead(SHOCK_CURRENT_ESTIMATION_ADC_PIN);
    shock_current_estimation = ((adc_voltage_reading_current_estimation * float(5) / (float(1023) * calibration_voltage_divider_resistor)));
    tft.setTextColor(BLUE, WHITE);
    tft.setCursor(0, 80);
    tft.println("Corriente: " + String(shock_current_estimation * (uint32_t)1000) + " mA");
  }
  tft.setTextSize(1);
  capacitor_discharge();
  tft.println("");
  push_button_1_status = LOW;
  push_button_2_status = LOW;
}
/* -------------------------------------------------------------------------- */
void esp8266_serial_event()
{

  if (Serial1.available())
  {

    int ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, ind9;

    received_str = Serial1.readString();
    Serial.println(received_str);
    if (received_str)
    {
      esp8266_sent_configuration_data = true;
      tft_header();
      tft.setTextColor(BLUE);
      tft.println("Parametros del protocolo de condicionamiento");
      tft.println("");

      ind1 = received_str.indexOf(',');
      experiment_day_number_str = received_str.substring(0, ind1);

      ind2 = received_str.indexOf(',', ind1 + 1);
      animal_id_srt = received_str.substring(ind1 + 1, ind2);

      ind3 = received_str.indexOf(',', ind2 + 1);
      exploration_time_str = received_str.substring(ind2 + 1, ind3);

      ind4 = received_str.indexOf(',', ind3 + 1);
      tone_frequency_str = received_str.substring(ind3 + 1, ind4);

      ind5 = received_str.indexOf(',', ind4 + 1);
      tone_time_srt = received_str.substring(ind4 + 1, ind5);

      ind6 = received_str.indexOf(',', ind5 + 1);
      shock_time_str = received_str.substring(ind5 + 1, ind6);

      ind7 = received_str.indexOf(',', ind6 + 1);
      motion_recording_time_str = received_str.substring(ind6 + 1, ind7);

      ind8 = received_str.indexOf(',', ind7 + 1);
      between_events_time_str = received_str.substring(ind7 + 1, ind8);

      ind9 = received_str.indexOf(',', ind8 + 1);
      number_of_total_events_str = received_str.substring(ind8 + 1, ind9);

      experiment_context = received_str.substring(ind9 + 1);

      if (experiment_day_number_str == "0" || animal_id_srt == "0" || exploration_time_str == "0" || tone_frequency_str == "0" || tone_time_srt == "0" || shock_time_str == "0" || motion_recording_time_str == "0" || between_events_time_str == "0" || number_of_total_events_str == "0" || experiment_context == "0")
      {
        tft.setTextColor(RED);
        tft.println("Se ingreso algun parametro de forma incorrecta, verifica los datos.");
        esp8266_sent_configuration_data = false;
        delay(3000);
      }
      else
      {
        tft.setTextColor(BLACK);
        tft.println("Dia: " + experiment_day_number_str);
        tft.println("ID del animal: " + animal_id_srt);
        tft.println("Tiempo de exploracion: " + exploration_time_str + " s");
        tft.println("Frecuencia de tono: " + tone_frequency_str + " Hz");
        tft.println("Tiempo de tono: " + tone_time_srt + " s");
        tft.println("Tiempo de shock: " + shock_time_str + " s");
        tft.println("Tiempo de analisis: " + motion_recording_time_str + " s");
        tft.println("Tiempo de intervalo: " + between_events_time_str + " s");
        tft.println("Repeticiones del evento: " + number_of_total_events_str);
        tft.println("Contexto del experimento: " + experiment_context);
      }
    }
    if (esp8266_sent_configuration_data == true)
    {
      tft.println("");
      tft.setTextColor(RED);
      tft.println("Presiona 2 para comenzar el experimento");
      while (push_button_2_status == LOW)
      {
        push_button_2_status = digitalRead(PUSH_BUTTON_2_PIN);
      }
      push_button_2_status = LOW;
      tft.setTextColor(BLUE);
      tft.println("");
      tft.println("El experimento comenzara en 10 segundos");
      delay(10000);
    }
  }
}
/* -------------------------------------------------------------------------- */
volatile byte status_lamp = LOW;
void turn_on_off_lamp()
{
  status_lamp = !status_lamp;
  digitalWrite(LAMP_120VAC_SOURCE, status_lamp);
  delay(1000);
}
/* -------------------------------------------------------------------------- */
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200); // ESP8266
  pinMode(SD_CS_PIN, OUTPUT);
  Wire.begin();
  tft.reset();
  tft.begin(0x9341);
  pinMode(DC_170VDC_SOURCE, OUTPUT);
  digitalWrite(DC_170VDC_SOURCE, LOW);
  pinMode(LAMP_120VAC_SOURCE, OUTPUT);
  digitalWrite(LAMP_120VAC_SOURCE, LOW);
  pinMode(AUDIO_AMPLIFIER_PIN, OUTPUT);
  pinMode(DHT_PIN, INPUT);
  pinMode(PIR_1_ADC_PIN, INPUT);
  pinMode(PIR_2_ADC_PIN, INPUT);
  pinMode(PIR_3_ADC_PIN, INPUT);
  pinMode(UNLOAD_RESISTOR_PIN, OUTPUT);
  digitalWrite(UNLOAD_RESISTOR_PIN, HIGH);
  pinMode(SHOCK_CURRENT_ESTIMATION_ADC_PIN, INPUT);
  pinMode(PUSH_BUTTON_1_PIN, INPUT);
  pinMode(PUSH_BUTTON_2_PIN, INPUT);
  for (int bar = FIRST_BAR_PIN; bar <= LAST_BAR_PIN; bar++)
  {
    pinMode(bar, OUTPUT);
  }
#ifdef DEBUG_RTC
  set_rtc_date_and_time();
#endif
  dht.begin();
  tft_power_on();
  tft_main();
}
/* -------------------------------------------------------------------------- */
void loop()
{
  esp8266_serial_event();
  if (esp8266_sent_configuration_data == true)
  {
    full_experiment();
    esp8266_sent_configuration_data = false;
  }
  push_button_1_status = digitalRead(PUSH_BUTTON_1_PIN);
  if (push_button_1_status == HIGH)
  {
    shock_current_calibration();
    tft_main();
  }
  push_button_2_status = digitalRead(PUSH_BUTTON_2_PIN);
  if (push_button_2_status == HIGH)
  {
    turn_on_off_lamp();
    push_button_2_status = LOW;
  }
}