#include <i2c_t3.h>
#include <ADC.h>
//#include <Time.h>
//#include <TimeAlarms.h>
//#include <Bounce.h>

#ifndef converter_lib_h
#define converter_lib_h

#define P_DAC 0b0010010
#define S_DAC 0b0010000
#define ALL_DAC 0b1110011

#define ON 1
#define OFF 0
#define DISABLE -1

#define TIME_MODE 1

const uint8_t pri_switch = 22;
const uint8_t pri_switch_disable = 23;

const uint8_t sec_switch = 20;
const uint8_t sec_switch_disable = 21;

const uint8_t neg_vcc_en = 33;

const uint8_t button1 = 30;
const uint8_t button2 = 31;

const uint8_t load_sense_disable = 17;
const uint8_t load_sense = 16;
extern float load_voltage;

const uint8_t input_sense_disable = 15;
const uint8_t input_sense = 14;
extern float input_voltage;

const uint8_t P1 = 38;
const uint8_t P2 = 37;
const uint8_t S1 = 36;
const uint8_t S2 = 35;

const uint8_t green = 24;
const uint8_t red = 25;
const uint8_t blue = 26;

extern ADC *adc;
const float aref_voltage = 3.1624;
const uint8_t adc_res_bits = 12;
const float adc_res = pow(2, adc_res_bits) - 1;

// waveform generator constants
const float top_margin = 1.03;
const float bot_margin = 0.9;
const int16_t voltage_amplitude = 500;
const float sample_time = 0.00005;
const uint16_t freq = 10;
const uint16_t wave_points = 2000;
extern float sine_wave[wave_points];

void initialize();

float inputVoltage();
void intervalReadInputVoltage();
float loadVoltage();

void timedBoost(float on, float off);
void timedBuck(float on, float off);

void button1Pressed();
void button2Pressed();

void p_curr_peak();
void s_curr_zero();
void p_curr_zero();
void s_curr_peak();

void waveform_gen(float* waveform);

void delayMicroCycles(float microseconds);

extern volatile uint8_t button1_flag;
extern volatile uint8_t button2_flag;

extern volatile uint8_t s_zero;
extern volatile uint8_t p_peak;
extern volatile uint8_t s_peak;
extern volatile uint8_t p_zero;

extern int8_t pri_switch_on;
extern int8_t sec_switch_on;

#endif