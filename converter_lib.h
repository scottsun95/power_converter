#include <i2c_t3.h>
#include <ADC.h>

#ifndef converter_lib_h
#define converter_lib_h

// pin definitions
const uint8_t pri_switch = 22;
const uint8_t pri_switch_disable = 23;

const uint8_t sec_switch = 20;
const uint8_t sec_switch_disable = 21;

const uint8_t diode_disable = 19;
const uint8_t gate_supply_enable = 18;

const uint8_t button1 = 2;
const uint8_t button2 = 3;

const uint8_t load_sense_disable = 17;
const uint8_t load_sense = 16;
extern float load_voltage;

const uint8_t input_sense_disable = 15;
const uint8_t input_sense = 14;
extern float input_voltage;

const uint8_t green = 6;
const uint8_t red = 7;
const uint8_t blue = 8;

// ADC constants
extern ADC *adc;
const float aref_voltage = 3.3;
const uint8_t adc_res_bits = 12;
const float adc_res = pow(2, adc_res_bits) - 1;
const float input_gain = 1.95;
const float load_gain = 190;

// waveform generator constants
const int16_t voltage_amplitude = 400;
const float sample_time = 0.0001;		// sample_time * freq * wave_points = 1
const uint16_t freq = 10;
const uint16_t wave_points = 1000;
extern float sine_wave[wave_points];
extern float saw_wave[wave_points];
extern float square_wave[wave_points];

void initialize();

float inputVoltage();
float loadVoltage();

void timedBoost(float on, float off);
void timedBuck(float on, float off);

void button1Pressed();
void button2Pressed();

void waveform_gen(float* waveform);
void timedSquare(unsigned long on_time_milli, unsigned long off_time_milli, float voltage);

void delayMicroCycles(float microseconds);

extern volatile uint8_t button1_flag;
extern volatile uint8_t button2_flag;

#endif