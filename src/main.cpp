/**
 * @author Daniel Enrique Fernández García (dfernandezg250@alumno.uaemex.mx)
 * @brief
 * @version 2.1
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023 Universidad Autónoma del Estado de México
 */

/* -------------------------------- Libraries ------------------------------- */
#include <Arduino.h> // Arduino Library
#include <Elegoo_GFX.h>
#include <Elegoo_TFTLCD.h>
#include <Wire.h>
#include <SPI.h>
#include "DHT.h"
#include <ds3231.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
/* --------------------------------- VERSION -------------------------------- */
String version = "2.1";
/* --------------------------------- MACROS --------------------------------- */
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
#ifdef SET_RTC
int setHour = 12;
int setMin = 13;
int setSec = 0;
int setDay = 12;
int setMonth = 10;
int setYear = 2022;
#endif
#define POWER_SOURCE_RELAY_PIN 22           // +170V Rectified Voltage Source
#define LAMP_RELAY_PIN 23                   // Incandescent lamp relay pin
#define DHT_TYPE DHT22                      // DHT sensor type
#define DHT_PIN 24                          // DHT22 sensor pin
#define AUDIO_AMPLIFIER_PIN 25              // Audio amplifier tone output
#define BUTTON_1_PIN 26                     // Push Button 1 pin
#define BUTTON_2_PIN 27                     // Push Button 2 pin
#define FIRST_SHOCK_BAR 28                  // First shock bar pin wired to the board teminal blocks
#define LAST_SHOCK_BAR 45                   // Last shock bar pin wired to the board teminal blocks
#define CAPACITOR_DISCHARGE_RESISTOR_PIN 46 // Capacitor discharge resistor pin
uint8_t capacitor_discharge_time_in_sec = 100;
// Due to PCB design errors, the calibration optocoupler is discarded
// #define CALIBRATION 47
#define CS_SD_PIN 53                             // SD chip-select pin
#define PIR_1_ANALOG_PIN A12                     // PIR 1 analog input pin
#define PIR_2_ANALOG_PIN A13                     // PIR 2 analog input pin
#define PIR_3_ANALOG_PIN A14                     // PIR 3 analog input pin
#define SHOCK_VOLTAGE_ESTIMATION_ANALOG_PIN A15  // Current estimation analog input pin
volatile byte lamp_status = LOW;                 // Lamp status ON/OFF
float temperature = 0;                           // DHT22 temperature reading
float humidity = 0;                              // DHT22 relative humidity reading
volatile byte button_1_status = LOW;             // Push button 1 pressed status
volatile byte button_2_status = LOW;             // Push button 2 pressed status
volatile float current_estimation_value = 0;     // Current estimation voltage ADC voltage reading value
uint16_t calibration_resistor_value = 993 + 270; // Current calibration voltage divider resistor
const int shock_bars_activation_delay_in_ms = 10;
/* -------------------------- HTML Form User Input -------------------------- */
String received_str;
String experiment_day_str;
String animal_strain_str;
String exploration_duration_str;
String tone_frequency_str;
String tone_duration_str;
String shock_duration_str;
String movement_register_duration_str;
String waiting_interval_duration_str;
String number_of_experiments_to_do_str;
String experiment_context_str;
// Variables to convert strings to integers
uint16_t exploration_duration;
uint16_t tone_frequency;
uint16_t tone_duration;
uint16_t shock_duration;
uint16_t movement_register_duration;
uint16_t waiting_interval_duration;
uint8_t number_of_experiments_to_do;
uint16_t experiment_context;
volatile byte received_parameters_confirmation = false;
String start_experiment_date;
String start_experiment_time;
String end_experiment_date;
String end_experiment_time;
unsigned long experiment_duration_in_ms;

/* -------------------------------- INSTANCES ------------------------------- */
DHT dht(DHT_PIN, DHT_TYPE);                                   // DHT22 sensor instance
Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET); // TFT Instance
struct ts t;                                                  // Time struct for RTC data
/* -------------------------------------------------------------------------- */
void Validate_Device_Startup()
{
  Serial.println("[INFO] Validating device");
  if (!SD.begin(CS_SD_PIN))
  {
    tft.setTextColor(RED);
    tft.println("[FAILED] SD Module");
    Serial.println("[FAILED] SD Module");
    while (1)
      ; // If there's an error with the SD, the execution of the script stops.
  }
  else
  {
    tft.setTextColor(GREEN);
    tft.println("[SUCCESS] SD Module");
    Serial.println("[SUCCESS] SD Module");
  }

  DS3231_get(&t);
  if (t.hour == 0)
  {
    tft.setTextColor(RED);
    tft.println("[FAILED] DS3231 Module");
    Serial.println("[FAILED] DS3231 Module");
    while (1);
  }
  else
  {
    tft.setTextColor(GREEN);
    tft.println("[SUCCESS] DS3231 Module");
    Serial.println("[SUCCESS] DS3231 Module");
  }

  if (isnan(dht.readTemperature()) && isnan(dht.readHumidity()))
  {
    tft.setTextColor(RED);
    tft.println("[FAILED] DHT22 Module");
    Serial.println("[FAILED] DHT22 Module");
    while (1);
  }
  else
  {
    tft.setTextColor(GREEN);
    tft.println("[SUCCESS] DHT22 Module");
    Serial.println("[SUCCESS] DHT22 Module");
  }
  tft.setTextColor(BLACK);
  tft.println("[TEST] Tone Output");
  Serial.println("[TEST] Tone Output");
  tone(AUDIO_AMPLIFIER_PIN, 4000);
  delay(1000);
  noTone(AUDIO_AMPLIFIER_PIN);
  tft.println("[TEST] Lamp Output");
  Serial.println("[TEST] Lamp Output");
  digitalWrite(LAMP_RELAY_PIN, HIGH);
  delay(1000);
  digitalWrite(LAMP_RELAY_PIN, LOW);
  delay(1000);
}
/* -------------------------------------------------------------------------- */
/*                                Screen Header                               */
/* -------------------------------------------------------------------------- */
void TFT_Header()
{
  tft.setRotation(3);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.fillScreen(WHITE);
  tft.setTextColor(BLUE);
  tft.println("");
  tft.println("Operant Conditioning Chamber v" + version);
  tft.setTextColor(BLACK);
  tft.println("");
  tft.setTextSize(1);
}
/* -------------------------------------------------------------------------- */
/*                                 Main Screen                                */
/* -------------------------------------------------------------------------- */
void TFT_Main_Screen()
{
  TFT_Header();
  tft.println("To set the experiment parameters, complete the forms using your web browser.");
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
  tft.println("Press 2 to enter Calibration Mode");
}
/* -------------------------------------------------------------------------- */
/*                                Start Screen                                */
/* -------------------------------------------------------------------------- */
void TFT_Startup_Screen()
{
  Serial.println("[INFO] Startup screen");
  TFT_Header();
  tft.println("Laboratorio de Neurobiologia de la Adiccion y Plasticidad Cerebral");
  tft.println("");
  tft.println("Facultad de Ciecias, UAEMex");
  tft.println("");
  tft.println("");
  Validate_Device_Startup();
}
/* -------------------------------------------------------------------------- */
/*                          Temperature and Humidity                          */
/* -------------------------------------------------------------------------- */
void ReadDTH()
{
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}
/* -------------------------------------------------------------------------- */
/**
 * The function turns on the capacitor discharge resistor for a specified duration of time
 *
 * @param duration The time in milliseconds for which the capacitor should be discharged.
 */
void Capacitor_Discharge(unsigned long duration)
{
  digitalWrite(POWER_SOURCE_RELAY_PIN, LOW);
  unsigned long start_time = millis();
  tft.fillScreen(WHITE);
  Serial.println("[STARTED] Discharging the capacitor");
  while ((millis() - start_time) < duration)
  {
    digitalWrite(CAPACITOR_DISCHARGE_RESISTOR_PIN, HIGH);
  }
  Serial.println("[ENDED] Discharging the capacitor");
} // End Capacitor_Discharge()
/* -------------------------------------------------------------------------- */
/**
 * This function will display a cyan screen for the duration of the event
 *
 * @param duration The duration of the event in milliseconds
 */
void Experiment_Exploration_Event(unsigned long duration)
{
  unsigned long start_time = millis();
  tft.fillScreen(CYAN);
  Serial.println("[STARTED] Environment event");
  while ((millis() - start_time) < duration)
  {
  }
  Serial.println("[ENDED] Exploration event");
} // End Experiment_Exploration_Event()
/* -------------------------------------------------------------------------- */
/**
 * This function plays a tone for a specified duration
 *
 * @param duration The duration of the tone in milliseconds
 */
void Experiment_Tone_Event(unsigned long duration)
{
  unsigned long start_time = millis();
  tft.fillScreen(GREEN);
  Serial.println("[STARTED] Tone stimulus event");
  while ((millis() - start_time) < duration)
  {
    tone(AUDIO_AMPLIFIER_PIN, tone_frequency);
  }
  Serial.println("[ENDED] Tone stimulus event");
  noTone(AUDIO_AMPLIFIER_PIN);
} // End Experiment_Tone_Event()
/* -------------------------------------------------------------------------- */
/**
 * It turns on the shock bars for a specified duration
 *
 * @param duration The duration of the shock event in milliseconds
 */
void Experiment_Shock_Event(unsigned long duration)
{
  unsigned long start_time = millis();
  tft.fillScreen(RED);
  Serial.println("[STARTED] Shock stimulus event");
  digitalWrite(CAPACITOR_DISCHARGE_RESISTOR_PIN, LOW);
  digitalWrite(POWER_SOURCE_RELAY_PIN, HIGH);
  while ((millis() - start_time) < duration)
  {
    for (int i = FIRST_SHOCK_BAR; i <= LAST_SHOCK_BAR; i++)
    {
      digitalWrite(i, HIGH);
      delay(shock_bars_activation_delay_in_ms);
      digitalWrite(i, LOW);
    }
  }
  for (int i = FIRST_SHOCK_BAR; i <= LAST_SHOCK_BAR; i++)
  {
    digitalWrite(i, LOW);
  }
  Serial.println("[ENDED] Shock stimulus event");
  digitalWrite(POWER_SOURCE_RELAY_PIN, LOW);
  digitalWrite(CAPACITOR_DISCHARGE_RESISTOR_PIN, HIGH);
} // End Experiment_Shock_Event()
/* -------------------------------------------------------------------------- */
volatile uint64_t current_sample = 0; // Counter for the current sample in the IST
const uint16_t datalogger_frequency = 100;  // This frequency is set in Timer_1_Interrupt_Setup()
const uint32_t max_samples = datalogger_frequency * 60;
volatile uint16_t PIR_1_Data[max_samples];       // PIR 1 Sensor data vector
volatile uint16_t PIR_2_Data[max_samples];       // PIR 2 Sensor data vector
volatile uint16_t PIR_3_Data[max_samples];       // PIR 3 Sensor data vector
/* -------------------------------------------------------------------------- */
/**
 * It enables the time compare interrupt on Timer 1, which will trigger the interrupt service routine
 * (ISR) every time the timer reaches the value of OCR1A. The ISR will then call the function that
 * reads the sensors data and stores it in the sensors data vector. The function will run until the
 * current_sample variable reaches the maximum number of samples, which is calculated by multiplying
 * the duration of the experiment by the datalogger frequency
 *
 * @param duration The duration of the experiment in seconds
 */
void Experiment_Motion_Detection_Event()
{
  tft.fillScreen(YELLOW);
  // uint64_t Max_Samples = duration * datalogger_frequency; // Maximum number of samples for the sensors data vector
  Serial.println("[STARTED] Motion detection event");
  while (current_sample <= max_samples)
  {
    // Enable time compare interrupt on Timer 1
    TIMSK1 = 1 << OCIE1A;
  }
  TIMSK1 = 0 << OCIE1A; // Disable time compare interrupt on Timer 1
  current_sample = 0;   // Resets current_sample variable to 0
  Serial.println("[ENDED] Motion detection event");
} // End Experiment_Motion_Detection_Event()
/* -------------------------------------------------------------------------- */
/**
 * Set the timer 1 control registers to use a 64 prescaler and CTC mode, and set the compare match
 * register to 2499
 */
void Timer_1_Interrupt_Setup()
{
  // Using Timer 1 for 100 Hz events
  TCCR1A = 0; // Set entire TCCR1A register to 0
  TCCR1B = 0; // Set entire TCCR1B register to 0
  TCNT1 = 0;  // Initialize the counter to 0
  // Set bits for 64 prescaler and CTC mode
  TCCR1B = 1 << WGM12 | 0 << CS12 | 1 << CS11 | 1 << CS10;
  OCR1A = 2499; // Set comparte match register to 100 Hz increments
} // End Timer_1_Interrupt_Setup()
/* -------------------------------------------------------------------------- */
/**
 * The function is called when the timer reaches the value specified in the OCR1A register
 */
ISR(TIMER1_COMPA_vect)
{
  // Timer 1 Interruption Service Routine
  PIR_1_Data[current_sample] = analogRead(PIR_1_ANALOG_PIN);
  PIR_2_Data[current_sample] = analogRead(PIR_2_ANALOG_PIN);
  PIR_3_Data[current_sample] = analogRead(PIR_3_ANALOG_PIN);
  current_sample++;
} // End ISR(TIMER1_COMPA_vect)
/* -------------------------------------------------------------------------- */
/**
 * This function will wait for a specified amount of time and then return
 *
 * @param duration The duration of the waiting event in milliseconds.
 */
void Experiment_Waiting_Event(unsigned long duration)
{
  unsigned long start_time = millis();
  tft.fillScreen(MAGENTA);
  Serial.println("[STARTED] Waiting event");
  while ((millis() - start_time) < duration)
  {
  }
  Serial.println("[ENDED] Waiting event");
} // End Experiment_Waiting_Event()
/* -------------------------------------------------------------------------- */
/**
 * It converts the strings that were received from the serial port into integers
 */
void Received_UART_Data_Conversion()
{
  exploration_duration = exploration_duration_str.toInt();
  tone_frequency = tone_frequency_str.toInt();
  tone_duration = tone_duration_str.toInt() * 1000;
  shock_duration = shock_duration_str.toInt() * 1000;
  movement_register_duration = movement_register_duration_str.toInt();
  waiting_interval_duration = waiting_interval_duration_str.toInt() * 1000;
  number_of_experiments_to_do = number_of_experiments_to_do_str.toInt();
} // End Received_UART_Data_Conversion()
/* -------------------------------------------------------------------------- */
void Full_Experiment()
{
  unsigned long experiment_start_in_ms;
  unsigned long experiment_stop_in_ms;
  DS3231_get(&t);
  start_experiment_date = String(t.mday) + "," + String(t.mon) + "," + String(t.year);
  start_experiment_time = String(t.hour) + "," + String(t.min) + "," + String(t.sec);
  experiment_start_in_ms = millis();
  for (int i = 1; i <= number_of_experiments_to_do; i++)
  {
    TFT_Header();
    Serial.println("[STARTED] Event " + String(i));
    Experiment_Exploration_Event(exploration_duration);
    Experiment_Tone_Event(tone_duration);
    Experiment_Shock_Event(shock_duration);
    Experiment_Motion_Detection_Event();
    Experiment_Waiting_Event(waiting_interval_duration);
    Serial.println("[STARTED] Event " + String(i));
    delay(2000);
  }
  experiment_stop_in_ms = millis();
  experiment_duration_in_ms = experiment_start_in_ms - experiment_stop_in_ms;
  DS3231_get(&t);
  end_experiment_date = String(t.mday) + "," + String(t.mon) + "," + String(t.year);
  end_experiment_time = String(t.hour) + "," + String(t.min) + "," + String(t.sec);
  button_1_status = LOW;

  TFT_Main_Screen();
}

/* -------------------------------------------------------------------------- */
/*                              Current Reading                               */
/* -------------------------------------------------------------------------- */
void Shock_Current_Calibration()
{
  int voltage_reading_for_current_estimation;

  button_2_status = LOW;
  digitalWrite(CAPACITOR_DISCHARGE_RESISTOR_PIN, LOW);
  digitalWrite(POWER_SOURCE_RELAY_PIN, HIGH);

  TFT_Header();
  tft.println("Shock current estimation");
  tft.println("");
  tft.println("Press 2 to exit Calibration Mode");
  tft.println("");
  tft.setTextSize(2);

  while (button_2_status == LOW)
  {
    button_2_status = digitalRead(BUTTON_2_PIN);
    voltage_reading_for_current_estimation = analogRead(SHOCK_VOLTAGE_ESTIMATION_ANALOG_PIN);
    current_estimation_value = ((voltage_reading_for_current_estimation * float(5) / (float(1023) * calibration_resistor_value)));
    tft.setTextColor(RED, WHITE);
    tft.setCursor(0, 80);
    tft.println("Current: " + String(current_estimation_value * 1000) + " mA");
  }

  Capacitor_Discharge(capacitor_discharge_time_in_sec * 1000);

  button_2_status = LOW;
}
/* -------------------------------------------------------------------------- */
void Waiting_for_Web_Server_Parameters()
{
  if (Serial1.available())
  {
    int ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, ind9;

    received_str = Serial1.readString();
    Serial.println("[SUCCESS] Parameters were received");
    Serial.println(received_str);
    if (received_str)
    {
      received_parameters_confirmation = true;
      TFT_Header();
      tft.setTextColor(BLUE);
      tft.println("Experiment parameters");
      tft.println("");

      ind1 = received_str.indexOf(',');
      experiment_day_str = received_str.substring(0, ind1);
      ind2 = received_str.indexOf(',', ind1 + 1);
      animal_strain_str = received_str.substring(ind1 + 1, ind2);
      ind3 = received_str.indexOf(',', ind2 + 1);
      exploration_duration_str = received_str.substring(ind2 + 1, ind3);
      ind4 = received_str.indexOf(',', ind3 + 1);
      tone_frequency_str = received_str.substring(ind3 + 1, ind4);
      ind5 = received_str.indexOf(',', ind4 + 1);
      tone_duration_str = received_str.substring(ind4 + 1, ind5);
      ind6 = received_str.indexOf(',', ind5 + 1);
      shock_duration_str = received_str.substring(ind5 + 1, ind6);
      ind7 = received_str.indexOf(',', ind6 + 1);
      movement_register_duration_str = received_str.substring(ind6 + 1, ind7);
      ind8 = received_str.indexOf(',', ind7 + 1);
      waiting_interval_duration_str = received_str.substring(ind7 + 1, ind8);
      ind9 = received_str.indexOf(',', ind8 + 1);
      number_of_experiments_to_do_str = received_str.substring(ind8 + 1, ind9);
      experiment_context_str = received_str.substring(ind9 + 1);

      if (experiment_day_str == "0" || animal_strain_str == "0" || exploration_duration_str == "0" || tone_frequency_str == "0" || tone_duration_str == "0" || shock_duration_str == "0" || movement_register_duration_str == "0" || waiting_interval_duration_str == "0" || number_of_experiments_to_do_str == "0" || experiment_context_str == "0")
      {
        tft.setTextColor(RED);
        tft.println("Se ingreso algun parametro de forma incorrecta, verifica tus datos y repite la operacion");
        received_parameters_confirmation = false;
      }
      else
      {
        tft.setTextColor(BLACK);
        tft.println("Day: " + experiment_day_str);
        tft.println("Animal ID: " + animal_strain_str);
        tft.println("Exploration duration: " + exploration_duration_str + " s");
        tft.println("Tone frequency: " + tone_frequency_str + " Hz");
        tft.println("Tone duration: " + tone_duration_str + " s");
        tft.println("Shock duration: " + shock_duration_str + " s");
        tft.println("Movement detection duration: " + movement_register_duration_str + " s");
        tft.println("Waiting interval duration: " + waiting_interval_duration_str + " s");
        tft.println("Number of events: " + number_of_experiments_to_do_str);
        tft.println("Experiment context: " + experiment_context_str);
        received_parameters_confirmation = true;
      }
    }

    tft.println("");
    tft.println("");
    tft.setTextColor(RED);
    tft.println("Press 1 to start the experiment");

    while (button_1_status == LOW)
    {
      button_1_status = digitalRead(BUTTON_1_PIN);
    }
    button_1_status = LOW;
  }
}

void setup()
{
  Serial.begin(115200);  // Serial Communication with PC
  Serial1.begin(115200); // Serial Communication with ESP8266
  Wire.begin();

  tft.reset();
  tft.begin(0x9341);

  pinMode(CS_SD_PIN, OUTPUT);
  pinMode(POWER_SOURCE_RELAY_PIN, OUTPUT);              // Power Source Relay
  digitalWrite(POWER_SOURCE_RELAY_PIN, LOW);            // Relay is set to OFF
  pinMode(LAMP_RELAY_PIN, OUTPUT);                      // Lamp SOURCE Relay
  digitalWrite(LAMP_RELAY_PIN, lamp_status);            // Relay is set to OFF
  pinMode(AUDIO_AMPLIFIER_PIN, OUTPUT);                 // Audio Amplifier
  pinMode(DHT_PIN, INPUT);                              // DHT-22 Sensor
  pinMode(PIR_1_ANALOG_PIN, INPUT);                     // PIR 1
  pinMode(PIR_2_ANALOG_PIN, INPUT);                     // PIR 2
  pinMode(PIR_3_ANALOG_PIN, INPUT);                     // PIR 3
  pinMode(CAPACITOR_DISCHARGE_RESISTOR_PIN, OUTPUT);    // Cement Resistor Optocoupler
  digitalWrite(CAPACITOR_DISCHARGE_RESISTOR_PIN, HIGH); // 19th optocoupler is set to ON
  // Due to PCB design errors, the calibration optocoupler is discarded
  // pinMode(CALIBRATION, OUTPUT);
  // digitalWrite(CALIBRATION, LOW);
  pinMode(SHOCK_VOLTAGE_ESTIMATION_ANALOG_PIN, INPUT); // A15 is used to read calibration voltage and estimate the current
  pinMode(BUTTON_1_PIN, INPUT);                        // Button 1
  pinMode(BUTTON_2_PIN, INPUT);                        // Button 2
  // 1st - 18th optocouplers pins are configured
  for (int i = FIRST_SHOCK_BAR; i <= LAST_SHOCK_BAR; i++)
  {
    pinMode(i, OUTPUT);
  }
  DS3231_init(DS3231_CONTROL_INTCN);
#ifdef SET_RTC
  t.hour = setHour;
  t.min = setMin;
  t.sec = setSec;
  t.mday = setDay;
  t.mon = setMonth;
  t.year = setYear;
  DS3231_set(t);
#endif
  dht.begin();
  TFT_Startup_Screen();
  TFT_Main_Screen();
}

void loop()
{

  Waiting_for_Web_Server_Parameters();
  if (received_parameters_confirmation == true)
  {
    Received_UART_Data_Conversion();
    Full_Experiment();
    received_parameters_confirmation = false;
  }
  // button_1_status = digitalRead(BUTTON_1_PIN);
  // if (button_1_status == HIGH)
  // {
  //   button_1_status = LOW;
  // }
  button_2_status = digitalRead(BUTTON_2_PIN);
  if (button_2_status == HIGH)
  {
    Shock_Current_Calibration();
    TFT_Main_Screen();
  }
}