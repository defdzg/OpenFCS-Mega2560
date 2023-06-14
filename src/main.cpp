/**
 * @file main.cpp
 * @author Daniel Fernández (defdzg@gmail.com)
 * @version 2.0
 * @date 2023-03-21
 *
 */
/* ------------------------------ Dependencies ------------------------------ */
#include <Arduino.h>         // Arduino framework library
#include <Elegoo_GFX.h>      // Elegoo graphics library
#include <Elegoo_TFTLCD.h>   // Elegoo TFT display library
#include <Wire.h>            // I2C library for Arduino
#include <SPI.h>             // Serial Peripheral Interface (SPI) library
#include <DHT.h>             // DHT sensors library
#include <DS3231.h>          // Arduino library for the DS3231 Real-Time Clock chip
#include <Adafruit_Sensor.h> // Adafruit Sensor libray
#include <SD.h>              //  SD library
#include <TimerOne.h>        // Interrupt and PWM utilities for Timer1
#include <PriUint64.h>       // Print uint64_t using Arduino
/* ------------------------------ Digital pins ------------------------------ */
#define DC_170VDC_SOURCE 22    // Power supply 170VDC relay pin
#define LAMP_120VAC_SOURCE 23  // Lamp 110VAC relay pin
#define DHT_PIN 24             // DHT-22 temperature and humidity sensor pin
#define AUDIO_AMPLIFIER_PIN 25 // Audio amplifier tone output pin
#define PUSH_BUTTON_1_PIN 26   // Push button 1 pin
#define PUSH_BUTTON_2_PIN 27   // Push button 2 pin
#define FIRST_BAR_PIN 28       // Arduino pin connected to the first bar optocoupler
#define LAST_BAR_PIN 45        // Arduino pin connected to the last bar optocoupler
#define UNLOAD_RESISTOR_PIN 46 // Arduino pin connected to the unload resistor optocoupler
#define SD_CS_PIN 53           // SD chip-select pin
/* ------------------------------- Analog pins ------------------------------ */
#define LCD_RD A0                            // TFT pin
#define LCD_WR A1                            // TFT pin
#define LCD_CD A2                            // TFT pin
#define LCD_CS A3                            // TFT pin
#define LCD_RESET A4                         // TFT pin
#define PIR_1_ADC_PIN A12                    // PIR 1 analog signal input pin
#define PIR_2_ADC_PIN A13                    // PIR 2 analog signal input pin
#define PIR_3_ADC_PIN A14                    // PIR 3 analog signal input pin
#define SHOCK_CURRENT_ESTIMATION_ADC_PIN A15 // Shock current estimation input pin
/* --------------------------- Application macros --------------------------- */
#define BLACK 0x0000                                    // TFT color
#define BLUE 0x001F                                     // TFT color
#define RED 0xF800                                      // TFT color
#define GREEN 0x07E0                                    // TFT color
#define CYAN 0x07FF                                     // TFT color
#define MAGENTA 0xF81F                                  // TFT color
#define YELLOW 0xFFE0                                   // TFT color
#define WHITE 0xFFFF                                    // TFT color
volatile byte push_button_1_status = LOW;               // Push button 1 status
volatile byte push_button_2_status = LOW;               // Push button 2 status
volatile uint32_t shock_current_estimation = 0;         // Shock current estimation value
float calibration_voltage_divider_resistor = 993 + 270; // Voltage divider resistor value for current estimation
uint8_t capacitor_discharge_time_in_seconds = 60;       // Minimum safe capacitor discharge time in seconds
String firmware_version = "2.0";                        // Current firmware version
String received_str;                                    // UART received experiment parameters
String experiment_day_number_str;                       // Experiment day (experiment parameter)
String animal_id_srt;                                   // Animal ID (experiment parameter)
String exploration_time_str;                            // Exploration time (experiment parameter)
String tone_frequency_str;                              // Tone frequency (experiment parameter)
String tone_time_srt;                                   // Tone duration time (experiment parameter)
String shock_time_str;                                  // Shock duration time (experiment parameter)
String motion_recording_time_str;                       // Motion recording time (experiment parameter)
String between_events_time_str;                         // Delay between events time (experiment parameter)
String number_of_total_events_str;                      // Number of total experiments to repeat (experiment parameter)
String experiment_context;                              // Experiment context 'A or B' (experiment parameter)
/* -------------------------------------------------------------------------- */
/**
 * It reads the temperature and humidity from the DHT22 sensor and stores the values in the global
 * variables temperature and humidity
 */
DHT dht(DHT_PIN, DHT22);
float temperature, humidity;
void retrieve_dht_data()
{
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}
/* -------------------------------------------------------------------------- */
/* Setting the time and date of the RTC */
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
/**
 * This function retrieves the current date and time from the RTC and stores it in the variables hour,
 * minute, seconds, day, month, and year
 */
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
/**
 * It creates a new directory path based on the current date and time
 */
void new_directory_path()
{
  String parent_path = String(day) + String(month) + String(year) + "/";
  String child_path = String(hour) + String(minute) + String(seconds) + "/";
  current_path = parent_path + child_path;
  SD.mkdir((char *)current_path.c_str());
}
/* -------------------------------------------------------------------------- */
Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
/**
 * If the error flag is set, print the word "ERROR" in red
 */
void tft_error_flag()
{
  tft.setTextColor(RED);
  tft.println("ERROR");
  tft.setTextColor(BLACK);
}
/* -------------------------------------------------------------------------- */
/**
 * This function prints the word "SUCCESS" in green on the TFT screen
 */
void tft_success_flag()
{
  tft.setTextColor(GREEN);
  tft.println("EXITO");
  tft.setTextColor(BLACK);
}
/* -------------------------------------------------------------------------- */
/**
 * It checks if the SD card, the DHT sensor, the speaker and the lamp are working
 */
void initialize_external_modules()
{
  tft.setTextColor(BLUE);
  tft.println("Inicializando modulos externos");

  /* Checking if the SD card is working. */
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

  /* Checking if the DHT sensor is working. */
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

  /* Playing a tone on the speaker until the button confirmation pressed. */
  push_button_2_status = LOW;
  tft.print("SPKR ..... ");
  while (push_button_2_status == LOW)
  {
    push_button_2_status = digitalRead(PUSH_BUTTON_2_PIN);
    tone(AUDIO_AMPLIFIER_PIN, 4000);
  }
  noTone(AUDIO_AMPLIFIER_PIN);
  tft_success_flag();

  /* Turning on the lamp until the button confirmation pressed. */
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
/**
 * This function sets the rotation of the screen, the text size, the cursor position, the background
 * color, the text color, and prints the firmware version
 */
void tft_header()
{
  tft.setRotation(3);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.fillScreen(WHITE);
  tft.setTextColor(BLUE);
  tft.println("");
  tft.println("OpenOCC v" + firmware_version);
  tft.setTextColor(BLACK);
  tft.println("");
  tft.setTextSize(1);
}
/* -------------------------------------------------------------------------- */
/**
 * It prints the main menu on the TFT screen
 */
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
/**
 * The function tft_power_on() is called when the Arduino is powered on. It displays the name of the
 * university on the screen
 */
void tft_power_on()
{
  tft_header();
  tft.println("Universidad Autonoma del Estado de Mexico");
  tft.println("");
  initialize_external_modules();
}
/* -------------------------------------------------------------------------- */
/**
 * Convert a value in seconds to milliseconds.
 *
 * @param value The value to convert to milliseconds.
 *
 * @return The return value is a uint64_t.
 */
uint64_t to_mills(uint8_t value)
{
  return (uint64_t)value * (uint64_t)1000;
}
/* -------------------------------------------------------------------------- */
/**
 * It discharges the capacitor by turning on the unload resistor and turning off the DC source
 */
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
/**
 * The function is called experiment_exploration_event() and it's purpose is to print a message to the
 * screen and wait for a certain amount of time
 */
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
/**
 * The function generates a tone of a given frequency for a given time
 */
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
/**
 * The function is called experiment_shock_event() and it's purpose is to turn on the DC power supply
 * and turn off the unload resistor, then turn on the bars one by one for a period of time
 */
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
File file_pir_samples;
uint64_t current_sample, max_samples;
float interrupt_timer_in_microseconds;
uint64_t data_logger_frequency_in_hz = 100;
unsigned long startMillis, stopMillis, totalMillis;
/**
 * Read the analog values from the PIR sensors and save them to the SD card
 */
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
/**
 * It enables ISR that calls read_analog_pir_sample_and_save_to_sd() to start datalogging.
 *
 * @param current_event This is the current event number.
 */
void experiment_motion_recording_event(int current_event)
{
  uint64_t motion_recording_time = motion_recording_time_str.toInt();

  tft.setTextColor(BLACK);
  tft.print("4. Registro de moviento ..... ");

  current_sample = 0;
  max_samples = (motion_recording_time)*data_logger_frequency_in_hz;
  interrupt_timer_in_microseconds = float(1) / float(data_logger_frequency_in_hz);
  interrupt_timer_in_microseconds = interrupt_timer_in_microseconds * float(1000000);
  String new_recording_file = current_path + "evento" + String(current_event) + ".txt";
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
/**
 * The function waits for a time period specified by the user
 */
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
File file_experiment_configuration;
uint64_t experiment_total_millis;
/**
 * It writes the current time, date, and experiment configuration parameters to a file on the SD card
 */
void save_experiment_configuration_data_to_sd()
{
  String new_experiment = current_path + "configuracion_experimento.txt";
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
/**
 * It's a function that runs the full experiment
 */
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
/**
 * This function is used to calibrate the shock current
 */
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
volatile bool esp8266_sent_configuration_data = false;
/**
 * It reads the data sent by the ESP8266 and stores it in a string. Then, it parses the string and
 * stores the data in different variables. Finally, it prints the data on the screen and waits for the
 * user to press a button to start the experiment
 */
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
/**
 * The function `turn_on_off_lamp()` turns on and off the lamp.
 */
void turn_on_off_lamp()
{
  status_lamp = !status_lamp;
  digitalWrite(LAMP_120VAC_SOURCE, status_lamp);
  delay(1000);
}
/* -------------------------------------------------------------------------- */
/**
 * The setup() function initializes the serial ports, the SD card, the I2C bus, the TFT display, the DC
 * power source, the lamp power source, the audio amplifier, the DHT sensor, the PIR sensors, the
 * unload resistor, the shock current estimation ADC, the push buttons, and the bars
 */
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
/**
 * The function is called every time the Arduino is looping through its code. It checks to see if the
 * ESP8266 has sent any data to the Arduino. If it has, it calls the function that runs the experiment.
 * It also checks to see if the push buttons have been pressed. If they have, it calls the appropriate
 * function
 */
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