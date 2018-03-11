#include <i2c_t3.h>
#include <ADC.h>
#include <Bounce.h>

#ifndef converter_lib_h
#define converter_lib_h

#define P_DAC 0b0010010
#define S_DAC 0b0010000
#define ALL_DAC 0b1110011

const int pri_switch = 22;
const int pri_switch_disable = 23;

const int sec_switch = 20;
const int sec_switch_disable = 21;

const int neg_vcc_en = 33;

const int button1 = 30;
const int button2 = 31;

const int load_sense_disable = 17;
const int load_sense = 16;

const int input_sense_disable = 15;
const int input_sense = 14;

const int P1 = 38;
const int P2 = 37;
const int S1 = 36;
const int S2 = 35;

const int green = 24;
const int red = 25;
const int blue = 26;

const float aref_voltage = 3.1624;
const int adc_res_bits = 12;
const float adc_res = pow(2, adc_res_bits) - 1;

void initialize();
float inputVoltage();
float loadVoltage();
void p_curr_peak();
void s_curr_zero();

extern volatile uint8_t s_zero;
extern volatile uint8_t p_peak;

#endif