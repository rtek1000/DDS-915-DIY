// DDS-915-DIY - Digital desoldering station (ZD-915 based)
//
// by RTEK1000 07/2023

#include <U8g2lib.h>    // https://github.com/olikraus/u8g2
#include <TimerOne.h>   // https://github.com/PaulStoffregen/TimerOne
#include <PID_v1.h>     // https://github.com/br3ttb/Arduino-PID-Library
#include <EEPROM.h>
#include <math.h>       /* log */
#include <avr/wdt.h>

#include <avr/pgmspace.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>       // to OLED display 
#endif

// OLED 0.96 inch LCD I2C addr: 0x3C (Check using I2C address scanning sketch):
// https://playground.arduino.cc/Main/I2cScanner/

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
// All Boards without Reset of the Display

#define pump_input    2     // Pistol trigger
#define heater_out    3     // PWM
#define pump_out      5     // PWM
#define btn_up        6     // Panel Button
#define btn_dn        7     // Panel Button
#define psu_enable    8     // Pump and heater power control
#define current_input A0    // Heater current sensor
#define tc_input      A1    // Heater temperature sensor
#define lm35_input    A2    // Cold temperature sensor
#define ref2495_input A3    // TL431 (Shunt)
#define LED           13    // Activity indicator

#define ALARM_TEMP 800   // General blocking temperature
#define MAX_TEMP 750     // Maximum setpoint temperature
#define MIN_TEMP 150     // Minimum setpoint temperature

#define ERROR_FREE 0     // No errors
#define ERROR_CURRENT 3
#define ERROR_TEMP 4

int error_number = ERROR_FREE;
float AdcRef = 0.0048828125; // calc by ref2495_input
int for_current;
int for_pump;
int calc_ambient_temp;
int temper_buff;
int disp_temper;
unsigned long millis_old;
unsigned long millis_old2;
unsigned long millis_old3;
int minutes_recall = 5;
int seconds_recall = 0;
int minutes = minutes_recall;
int seconds = seconds_recall;
int timeout_step = 0;
const int address0 = 0;
const int address2 = 2;
//int degree = 0; // Switch *C <--> *F
boolean lock_pump = LOW;
float ColdTemp_val;
int cycle_dsp_setup = 5;
int button_repeat = 100;
int setpoint_temp = 160;
float heater_temper = 0;
float HotTemp_val;
int current_adc = 0;
float ambient_temp = 0;
int blink_lcd = 0;
int heater_pwm = 10;
int pump_pwm = 0;
boolean pump_input_old = LOW;
int count_delay = 40;
int i = 0;
char str1[5] = {" "};
char str2[5] = {" "};
char str3[6] = {" "};
int power_heater = 0;
float current_heater = 0;
double current_old = 0;
double current_setpoint_limit = 4; // 4A
double current_setpoint = 0; // Auto increment
boolean pid_enable = LOW;
int WindowSize = 255; // current_setpoint_limit;
double Setpoint, Input, Output;

// PID parameters obtained by experiments:
double Kp = 48.89384052204361, Ki = 0.6984834360291945, Kd = 8.556422091357632;

// double Kp = 255, Ki = 0.0, Kd = 0.0; // Use this line to calculate Ku:
//
// wait for stabilization, note the time between the cycles
// (when the temperature starts to drop)
// and also note the temperature range of the oscillation
// Then calculate the terms of the PID with these formulas:
//
// A: Maximum and minimum temperature difference
// Pu: Time between the cycles
// D: Output current range for resistance (256 ==> PWM 8 bits)
// pi: Numerical constant PI
//
// A:4°C; Pu:14s; D:256; pi: 3.1415
//
// Ku = 4 * D / (A * pi)
// Kp = 0.6 * Ku
// Ki = 1.2 * Ku / Pu
// Kd = 0.075 * Ku * Pu

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Convert type E thermocouple voltages to temperatures efficiently using calibration equations based on rational polynomial function.
// Ref.: http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/thermocouple/type-e-calibration-table

const PROGMEM float E_t0[6] = { -117.21668, -50.0, 250.146, 601.3989, 804.359114, 25.0};
const PROGMEM float E_v0[6] = { -5.99016982152282, -2.78717773194122, 17.1917125502476, 45.2061669803173, 61.3591777093507, 1.49505819808044};
const PROGMEM float E_p1[6] = {23.6472750978691, 19.0227362111351, 13.1155219835456, 12.3993567843162, 12.7595082837539, 0.0609584430323512};
const PROGMEM float E_p2[6] = {12.807376698882, -1.70427248072634, 1.17803644700794, 0.433999633709735, -1.11160722790487, -0.000273517892774302};
const PROGMEM float E_p3[6] = {2.06650690516682, -0.35195188535448, 0.0364224332065292, 0.00919670854311092, 0.0353325355734949, -0.0000191301455052136};
const PROGMEM float E_p4[6] = {0.0865134722209353, 0.00477661021730048, 0.000395842612268852, 0.000169015849229869, 0.0000330803799209422, -0.0000000139488399503399};
const PROGMEM float E_q1[6] = {0.589958597259284, -0.0653797603792467, 0.0931127555672345, 0.0344246799080835, -0.0881968893185364, -0.00523823782546346};
const PROGMEM float E_q2[6] = {0.109607127643588, -0.0217328331374547, 0.00298042317717519, 0.000697412150284603, 0.00284974146705302, -0.000309701677615794};
const PROGMEM float E_q3[5] = {0.0061769587805607, 0.0, 0.0000332630318232913, 0.0000129469920158824, 0.0};

int for_start;

int temp_table[10];
byte temp_index = 0;

float get_Evcj(float tcj);
float get_Et(float v);
void setPwmFrequency(int pin, int divisor);
int get_f_degree(int c_degree);
void draw_init(void);
void draw_start(void);
void draw_starting(void);
void draw(int error);

void setup() {
  wdt_reset();

  // See bootloader problem: https://github.com/arduino/ArduinoCore-avr/issues/150
  // Only enable WDT if bootloader is already rewritten via ISP programmer
  // wdt_enable(WDTO_2S);

  u8g2.begin();

  Wire.setClock(400000); // 400kHz

  pinMode(btn_up, INPUT_PULLUP);
  pinMode(btn_dn, INPUT_PULLUP);
  pinMode(pump_input, INPUT_PULLUP);
  pinMode(current_input, INPUT);
  pinMode(tc_input, INPUT);
  pinMode(ref2495_input, INPUT);
  pinMode(heater_out, OUTPUT);
  pinMode(pump_out, OUTPUT);
  pinMode(psu_enable, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);
  digitalWrite(heater_out, LOW);
  digitalWrite(pump_out, LOW);
  digitalWrite(psu_enable, LOW);

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  myPID.SetMode(AUTOMATIC);

  // heater_out: 3
  //  o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
  //  pins 3 and 11 are:       1, 8, 32, 64, 128, 256, and 1024.
  setPwmFrequency(heater_out, 256);

  // pump_out: 5
  //  o The base frequency for pins 5 and 6 is 62500 Hz.
  //  pins 5, 6, 9 and 10 are: 1, 8,     64,      256, and 1024.
  //setPwmFrequency(pump_out, 256); //

  setpoint_temp = EEPROM.read(address0) * 256;
  setpoint_temp += EEPROM.read(address0 + 1);

  if ((setpoint_temp > MAX_TEMP) || (setpoint_temp < MIN_TEMP))
  {
    setpoint_temp = MIN_TEMP;
  }

  //  degree = EEPROM.read(address2);
  //
  //  if (degree > 1)
  //  {
  //    degree = 0;
  //  }

  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  u8g2.firstPage();
  do {
    draw_init();
  } while ( u8g2.nextPage() );

  delay(3000);

  u8g2.firstPage();
  do {
    draw_start();
  } while ( u8g2.nextPage() );

  beep();

  digitalWrite(LED, HIGH);

  while ((digitalRead(pump_input) == HIGH) && (digitalRead(btn_up) == HIGH) && (digitalRead(btn_dn) == HIGH)) {
    wdt_reset();
  }

  while ((digitalRead(pump_input) == LOW) || (digitalRead(btn_up) == LOW) || (digitalRead(btn_dn) == LOW)) {
    wdt_reset();
  }

  if (error_number == ERROR_FREE) {
    digitalWrite(psu_enable, HIGH);
  }

  if (current_adc > 10)
  {
    error_number = ERROR_CURRENT; // 3;
  }

  millis_old = millis();

  /*
    Ideal:
    5.0V / 1024 = 0.0048828125V
    2.495V / 0.0048828125V = 510

    Real:
    5.2V / 1024 = 0.005078125V
    2.495V / 0.005078125V = 491 (Read by ADC)

    491 = 2.495V
    1023 = xV

    x = (1023 * 2.495) / 491
    x = 2552.385 / 491
    x = 5.198340122199593V (Real)

    5.198340122199593V / 1024 = 0.0050765040255855V
    Ref = 0.0050765040255855V

    Ref = 2.495 / ADC
  */

  AdcRef = 0.0;

  for (int i = 0; i < 10; i++) {
    AdcRef += analogRead(ref2495_input);

    delay(10);
  }

  AdcRef /= 10.0;

  AdcRef = 2.495 / AdcRef;
}

void beep(void) {
  if (error_number == ERROR_FREE) {
    digitalWrite(pump_out, HIGH);

    delay(25);

    digitalWrite(pump_out, LOW);
  }

  //tone(pump_out, 1500, 250);
}

void loop() {
  if (error_number == ERROR_FREE)
  {
    if (disp_temper >= 0)
    {
      //      if (degree == 0)
      //      {
      temper_buff = disp_temper;
      //      }
      //      else
      //      {
      //      temper_buff = get_f_degree(disp_temper);
      //      }

      str1[0] = (temper_buff / 100);
      str1[1] = ((temper_buff - (str1[0] * 100)) / 10);
      str1[2] = (temper_buff - (str1[0] * 100) - (str1[1] * 10));

      str1[0] += 48;
      str1[1] += 48;
      str1[2] += 48;
      str1[3] = 0;
    }
    else if (disp_temper < -99)
    {
      str1[0] = 45;
      str1[1] = 45;
      str1[2] = 45;
      str1[3] = 0;
    }
    else
    {
      str1[0] = ((disp_temper * -1) / 100);
      str1[1] = (((disp_temper * -1) - (str1[0] * 100)) / 10);
      str1[2] = ((disp_temper * -1) - (str1[0] * 100) - (str1[1] * 10));

      str1[0] = 45;
      str1[1] += 48;
      str1[2] += 48;
      str1[3] = 0;
    }

    //    if (degree == 0)
    //    {
    temper_buff = setpoint_temp;
    //    }
    //    else
    //    {
    //      temper_buff = get_f_degree(setpoint_temp);
    //    }

    str2[0] = (temper_buff / 100);
    str2[1] = ((temper_buff - (str2[0] * 100)) / 10);
    str2[2] = (temper_buff - (str2[0] * 100) - (str2[1] * 10));

    str2[0] += 48;
    str2[1] += 48;
    str2[2] += 48;
    str2[3] = 0;

    str3[0] = minutes / 10;
    str3[1] = minutes - (str3[0] * 10) ;
    str3[2] = 58;
    str3[3] = seconds / 10;
    str3[4] = seconds - (str3[3] * 10) ;
    str3[5] = 0;

    str3[0] += 48;
    str3[1] += 48;
    str3[3] += 48;
    str3[4] += 48;

    if (blink_lcd < 8)
    {
      blink_lcd++;

    }
    else
    {
      blink_lcd = 0;
    }
  }

  u8g2.firstPage();
  do {
    draw(error_number);
  } while ( u8g2.nextPage() );

  if ((millis() - millis_old2) > 1000)
  {
    disp_temper = heater_temper;

    millis_old2 = millis();
  }

  if (digitalRead(pump_input) == LOW)
  {
    if (blink_lcd < 4)
    {
      if (timeout_step == 1)
      {
        analogWrite(heater_out, 0);

        while (digitalRead(pump_input) == LOW);

        timeout_step = 0;

        millis_old = millis();
      }
    }
  }
  else
  {
    if ((millis() - millis_old) > 1000)
    {
      millis_old = millis();

      if (seconds > 0)
      {
        seconds--;
      }
      else
      {
        seconds = 59;

        if (minutes > 0)
        {
          minutes--;
        }
        else
        {
          if (timeout_step == 0)
          {
            timeout_step++;

            minutes = minutes_recall;
            seconds = seconds_recall;

            if (error_number == ERROR_FREE) {
              digitalWrite(psu_enable, HIGH);
            }
          }
          else
          {
            analogWrite(heater_out, 0);

            u8g2.firstPage();
            do {
              draw_start();
            } while ( u8g2.nextPage() );

            digitalWrite(psu_enable, LOW);

            while ((digitalRead(pump_input) == HIGH) && (digitalRead(btn_up) == HIGH) && (digitalRead(btn_dn) == HIGH));

            while ((digitalRead(pump_input) == LOW) || (digitalRead(btn_up) == LOW) || (digitalRead(btn_dn) == LOW));

            minutes = minutes_recall;
            seconds = seconds_recall;

            if (error_number == ERROR_FREE) {
              digitalWrite(psu_enable, HIGH);
            }
          }
        }
      }
    }
  }

  if (error_number == ERROR_TEMP) {
    digitalWrite(pump_out, HIGH);

    digitalWrite(psu_enable, LOW);
  } else {
    if (((digitalRead(pump_input) == LOW) && (pump_input_old == false) && (timeout_step == 0)) &&
        ((lock_pump == LOW) || ((setpoint_temp - heater_temper) < 15)))
    {
      pump_input_old = true;

      minutes = minutes_recall;
      seconds = seconds_recall;

      delay(100);

      if (digitalRead(pump_input) == LOW)
      {
        for (for_pump = 0; for_pump <= 25; for_pump++)
        {
          if (pump_pwm < 240)
          {
            pump_pwm += 10;

            analogWrite(pump_out, pump_pwm);
          }
          else
          {
            pump_pwm = 255;

            digitalWrite(pump_out, HIGH);
          }

          delay(1);
        }
      }
    }
    else if ((heater_temper - setpoint_temp) > 25)
    {
      digitalWrite(pump_out, HIGH);

      pump_input_old = true;
    }
    else if ((digitalRead(pump_input) == HIGH) && (pump_input_old == true))
    {
      pump_input_old = false;

      for (for_pump = 25; for_pump >= 0; for_pump--)
      {
        if (pump_pwm > 10)
        {
          pump_pwm -= 10;
        }
        else
        {
          pump_pwm = 0;
        }

        analogWrite(pump_out, pump_pwm);

        delay(1);
      }
    }
  }

  if (digitalRead(btn_up) == LOW)
  {
    minutes = minutes_recall;
    seconds = seconds_recall;

    if ((setpoint_temp + button_repeat) < MAX_TEMP)
    {
      setpoint_temp += button_repeat;

      if (button_repeat == 5) {
        beep();
      }

    }
    else
    {
      if (setpoint_temp < MAX_TEMP) {
        beep();
      }

      setpoint_temp = MAX_TEMP;
    }

    cycle_dsp_setup = 3;
  }
  else if (digitalRead(btn_dn) == LOW)
  {
    minutes = minutes_recall;
    seconds = seconds_recall;

    if ((setpoint_temp - button_repeat) > MIN_TEMP)
    {
      setpoint_temp -= button_repeat;

      if (button_repeat == 5) {
        beep();
      }

    }
    else
    {
      if (setpoint_temp > MIN_TEMP) {
        beep();
      }

      setpoint_temp = MIN_TEMP;
    }

    cycle_dsp_setup = 3;
  }

  if ((digitalRead(btn_up) == HIGH) && (digitalRead(btn_dn) == HIGH))
  {
    button_repeat = 5;
  }
  else if ((digitalRead(btn_up) == LOW) || (digitalRead(btn_dn) == LOW))
  {
    if (button_repeat < 100)
    {
      button_repeat += 5;
    }
  }

  if (cycle_dsp_setup > 0)
  {
    cycle_dsp_setup--;

    if (cycle_dsp_setup == 1)
    {
      EEPROM.update(address0, setpoint_temp / 256);
      EEPROM.update(address0 + 1, setpoint_temp - ((setpoint_temp / 256) * 256));
    }
  }

  analogWrite(heater_out, 0);

  delay(2);

  if (error_number == ERROR_FREE) {

    int ADC_Val = 0; //LM35_temper.cel();

    for (uint16_t i = 0; i < 25; i++) {
      ADC_Val += analogRead(lm35_input);
    }

    ADC_Val /= 25;

    ColdTemp_val = (float(ADC_Val) * AdcRef) / 0.01;

    HotTemp_val = 0;

    for (uint16_t i = 0; i < 25; i++) {
      HotTemp_val += analogRead(tc_input);
    }

    HotTemp_val /= 25;

    HotTemp_val *= AdcRef; // ADC to V

    HotTemp_val *= 1000; // V to mV

    /*
       160ºC: TC=13.79mV; ADC=1053mV; Amp Op. Gain=76.3
       1645 - 1053 = 592; 592 / 2 + 1053 = 1349
       240ºC: TC=19.11mV; ADC=1645mV; Amp Op. Gain=86.1
       2268 - 1645 = 623; 623 / 2 + 1053 = 1956.5
       320ºC: TC=24.70mV; ADC=2268mV; Amp Op. Gain=91.8
       2955 - 2268 = 687; 687 / 2 + 2268 = 2611.5
       400ºC: TC=30.70mV; ADC=2955mV; Amp Op. Gain=96.3
       3829 - 2955 = 874; 874 / 2 + 2955 = 3392
       480ºC: TC=38.65mV; ADC=3829mV; Amp Op. Gain=99.0

    */

    HotTemp_val /= 50.0; // Amp. Op. gain: 100x

    HotTemp_val = HotTemp_val + get_Evcj(ColdTemp_val); // mV

    heater_temper = get_Et(HotTemp_val);

    if (heater_temper < 50) {
      current_setpoint = 42; // 6V 1.615R
    } else if (heater_temper < 75) {
      current_setpoint = 96; // 9V 1.72R
    } else if (heater_temper < 100) {
      current_setpoint = 128; // 12V 1.815
    } else if (heater_temper < 125) {
      current_setpoint = 160; // 15V 1.915R
    } else {

      if (heater_temper > ALARM_TEMP)
      {
        error_number = ERROR_TEMP; // ERROR_TEMP
      }

      Input = heater_temper;

      if (timeout_step == 1) // stand by after 15 min
      {
        Setpoint = 160 + 0.5;

        myPID.Compute();

        current_setpoint = Output;
      }
      else
      {
        Setpoint = setpoint_temp + 0.5;

        if ((setpoint_temp - heater_temper) > 100)
        {
          current_setpoint = 191; // 18V // current_setpoint_limit;
        }
        else if ((setpoint_temp - heater_temper) > 30)
        {
          current_setpoint = 160; // 15V // current_setpoint_limit;
        }
        else if ((setpoint_temp - heater_temper) > 5)
        {
          current_setpoint = 128; // 12V // current_setpoint_limit / 3;
        }
        else if ((setpoint_temp - heater_temper) <= 5)
        {
          myPID.Compute();

          current_setpoint = Output;
        }
      }
    }

    analogWrite(heater_out, current_setpoint);

    current_adc = analogRead(current_input);

    // Shunt sensor: 0R10
    // 1A = 0.1V
    // Amp Op. gain: 10
    // 1A = 1V
    // 5V = 5A

    current_heater = current_adc * AdcRef;

    if (current_heater > 4.8)
    {
      error_number = ERROR_CURRENT; // 3; // sensor inverted or MOSFET short circuit

      digitalWrite(psu_enable, LOW);
    }

    power_heater = (current_setpoint * 100) / 255;

  }

  if (error_number == ERROR_FREE) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }

  while ((millis() - millis_old3) < 200) { // To complete the cycle time of 200ms
    delayMicroseconds(1);
  }

  if (error_number == ERROR_FREE) {
    digitalWrite(LED, LOW);
  } else {
    digitalWrite(LED, HIGH);
  }

  wdt_reset();

  millis_old3 = millis();
}

float get_Evcj(float tcj)
{
  int i = 5;
  float vcj;
  float t0;
  float v0;
  float p1;
  float p2;
  float p3;
  float p4;
  float q1;
  float q2;
  float q3;

  t0 = pgm_read_float_near(E_t0 + i);
  v0 = pgm_read_float_near(E_v0 + i);
  p1 = pgm_read_float_near(E_p1 + i);
  p2 = pgm_read_float_near(E_p2 + i);
  p3 = pgm_read_float_near(E_p3 + i);
  p4 = pgm_read_float_near(E_p4 + i);
  q1 = pgm_read_float_near(E_q1 + i);
  q2 = pgm_read_float_near(E_q2 + i);

  vcj = v0 + (((tcj - t0) * (p1 + (tcj - t0) * (p2 + (tcj - t0) * (p3 + p4 * (tcj - t0))))) / (1 + ((tcj - t0) * (q1 + q2 * (tcj - t0)))));

  return vcj;
}

float get_Et(float v)
{
  int i;
  float t;
  float t0;
  float v0;
  float p1;
  float p2;
  float p3;
  float p4;
  float q1;
  float q2;
  float q3;

  if ((v >= -9.835) & (v < -5.237))
  {
    i = 0;
  }
  else if ((v >= -5.237) & (v < 0.591))
  {
    i = 1;
  }
  else if ((v >= 0.591) & (v < 24.964))
  {
    i = 2;
  }
  else if ((v >= 24.964) & (v < 53.112))
  {
    i = 3;
  }
  else if ((v >= 53.112) & (v < 76.373))
  {
    i = 4;
  }
  else
  {
    return -999;
  }

  t0 = pgm_read_float_near(E_t0 + i);
  v0 = pgm_read_float_near(E_v0 + i);
  p1 = pgm_read_float_near(E_p1 + i);
  p2 = pgm_read_float_near(E_p2 + i);
  p3 = pgm_read_float_near(E_p3 + i);
  p4 = pgm_read_float_near(E_p4 + i);
  q1 = pgm_read_float_near(E_q1 + i);
  q2 = pgm_read_float_near(E_q2 + i);
  q3 = pgm_read_float_near(E_q3 + i);

  t = t0 + (((v - v0) * (p1 + (v - v0) * (p2 + (v - v0) * (p3 + p4 * (v - v0))))) / (1 + ((v - v0) * (q1 + (v - v0) * (q2 + q3 * (v - v0))))));

  return t;
}

/**
   Divides a given PWM pin frequency by a divisor.

   The resulting frequency is equal to the base frequency divided by
   the given divisor:
     - Base frequencies:
        o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
        o The base frequency for pins 5 and 6 is 62500 Hz.
     - Divisors:
        o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
          256, and 1024.
        o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
          128, 256, and 1024.

   PWM frequencies are tied together in pairs of pins. If one in a
   pair is changed, the other is also changed to match:
     - Pins 5 and 6 are paired on timer0
     - Pins 9 and 10 are paired on timer1
     - Pins 3 and 11 are paired on timer2

   Note that this function will have side effects on anything else
   that uses timers:
     - Changes on pins 3, 5, 6, or 11 may cause the delay() and
       millis() functions to stop working. Other timing-related
       functions may also be affected.
     - Changes on pins 9 or 10 will cause the Servo library to function
       incorrectly.

   Thanks to macegr of the Arduino forums for his documentation of the
   PWM frequency divisors. His post can be viewed at:
     http://forum.arduino.cc/index.php?topic=16612#msg121031
*/
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

//int get_f_degree(int c_degree)
//{
//  return ((c_degree * 9) / 5) + 32;
//}

void draw_init(void) {
  u8g2.setFont(u8g2_font_fur14_tf); // 14 pixels

  u8g2.drawStr( 2, 16, "DDS-915 DIY");

  u8g2.setFont(u8g2_font_fur11_tr); // 11 pixels

  u8g2.drawStr( 7, 40, "(DZ-915 based)");
}

void draw_start(void) {
  u8g2.setFont(u8g2_font_fur14_tr); // 14 pixels

  u8g2.drawStr( 5, 16, "Press Switch");

  if (timeout_step == 0)
  {
    u8g2.drawStr( 25, 40, "To start");
  }
  else
  {
    u8g2.drawStr( 20, 40, "To restart");
  }
}

void draw_starting(void)
{
  u8g2.drawStr( 15, 25, "Starting...");
}

void draw(int _error) {
  char char1[2];

  if (_error != 0)
  {
    u8g2.setFont(u8g2_font_fur11_tr); // 14 pixels

    u8g2.drawStr( 0, 0, "ERROR:"); // 15

    //    if (_error == 1)
    //    {
    //      u8g2.drawStr( 0, 20, "Sensor Fail"); // 15
    //      u8g2.drawStr( 0, 40, "(Thermocouple)"); // 15
    //    }
    //    else if (_error == 2)
    //    {
    //      u8g2.drawStr( 0, 20, "Heater Fail"); // 15
    //    }
    //    else

    if (_error == ERROR_CURRENT)
    {
      u8g2.drawStr( 0, 20, "Heater Fail"); // 15
      u8g2.drawStr( 0, 40, "HIGH Current"); // 15
      //u8g2.drawStr( 0, 20, "Sensor Fail"); // 15
      //u8g2.drawStr( 0, 40, "(Current)"); // 15
    }
    else if (_error == ERROR_TEMP)
    {
      u8g2.drawStr( 0, 20, "HIGH Temp"); // 15
      //u8g2.drawStr( 0, 20, "Temperature"); // 15
      //u8g2.drawStr( 0, 40, "Out of range"); // 15
    }
  }
  else
  {
    u8g2.setFont(u8g2_font_7Segments_26x42_mn); // 35 u8g2_font_fur42_tr

    if (timeout_step == 0)
    {
      u8g2.drawStr( 0, 0, str1); // 15
    }
    else
    {
      if (blink_lcd < 4)
      {
        u8g2.drawStr( 0, 0, str1); // 15
      }
      else
      {
        u8g2.setFont(u8g2_font_fur14_tr); // 14 pixels
        u8g2.drawStr( 0, 0, "Press");
        u8g2.drawStr( 0, 20, "Switch"); // Trigger
      }
    }

    u8g2.setFont(u8g2_font_fur11_tr); // 11 pixels

    char1[0] = 111;
    char1[1] = 0;

    // u8g2.drawStr( 107, -3, char1);

    u8g2.drawStr( 107, -3, char1);

    u8g2.setFont(u8g2_font_fur14_tr); // 14 pixels

    //    if (degree == 0)
    //    {
    u8g2.drawStr( 116, 0, "C");
    //    }
    //    else
    //    {
    //      u8g2.drawStr( 116, 0, "F");
    //    }

    u8g2.drawStr( 75, 48, str3);

    u8g2.drawStr( 0, 48, "S:");
    u8g2.drawStr( 20, 48, str2);

    if (power_heater > 0)
    {
      if (power_heater < 20)
      {
        u8g2.drawStr( 61, 52, "-");
      }
      else if (power_heater < 40)
      {
        u8g2.drawStr( 61, 52, "-");
        u8g2.drawStr( 61, 49, "-");
      }
      else if (power_heater < 60)
      {
        u8g2.drawStr( 61, 52, "-");
        u8g2.drawStr( 61, 49, "-");
        u8g2.drawStr( 61, 46, "-");
      }
      else if (power_heater < 80)
      {
        u8g2.drawStr( 61, 52, "-");
        u8g2.drawStr( 61, 49, "-");
        u8g2.drawStr( 61, 46, "-");
        u8g2.drawStr( 61, 43, "-");
      }
      else // 100
      {
        u8g2.drawStr( 61, 52, "-");
        u8g2.drawStr( 61, 49, "-");
        u8g2.drawStr( 61, 46, "-");
        u8g2.drawStr( 61, 43, "-");
        u8g2.drawStr( 61, 40, "-");
      }
    }

    if (digitalRead(btn_up) == LOW)
    {
      u8g2.drawStr( 100, 27, "Up"); // 5

      lock_pump = HIGH;
    }
    else if (digitalRead(btn_dn) == LOW)
    {
      u8g2.drawStr( 100, 27, "Dn"); // 5

      lock_pump = HIGH;
    }
    else
    {
      if (((setpoint_temp > heater_temper) && ((setpoint_temp - heater_temper) > 10)) ||
          ((setpoint_temp < heater_temper) && ((setpoint_temp - heater_temper) < -10)))
      {
        u8g2.drawStr( 107, 27, "W"); // 5

        lock_pump = HIGH;
      }
      else //if (current_setpoint > 0) // heater_temper < setpoint_temp
      {
        lock_pump = LOW;
      }
    }

    if ((digitalRead(pump_input) == LOW) && (lock_pump == LOW))
    {
      u8g2.drawStr( 107, 27, "P");
    }
  }
}