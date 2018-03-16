#include <i2c_t3.h>
#include <ADC.h>
#include <RingBufferDMA.h>
#include <DMAChannel.h>
#include <Time.h>
#include <TimeAlarms.h>
//#include <Bounce.h>

#ifndef converter_lib_h
#define converter_lib_h

#define P_DAC 0b0010010
#define S_DAC 0b0010000
#define ALL_DAC 0b1110011

const uint8_t pri_switch = 22;
const uint8_t pri_switch_disable = 23;

const uint8_t sec_switch = 20;
const uint8_t sec_switch_disable = 21;

const uint8_t neg_vcc_en = 33;

const uint8_t button1 = 30;
const uint8_t button2 = 31;

const uint8_t load_sense_disable = 17;
const uint8_t load_sense = 16;
extern volatile uint16_t load_adc[1];

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

void initialize();

float inputVoltage();
void intervalReadInputVoltage();
float loadVoltage();

void dmaInit();
void dma_isr(); 

void timedBoost(unsigned int on, unsigned int off);
void timedBuck(unsigned int on, unsigned int off);
void comparatorBoost();
void comparatorBuck();

void button1Pressed();
void button2Pressed();

void p_curr_peak();
void s_curr_zero();
void p_curr_zero();
void s_curr_peak();


extern volatile uint8_t button1_flag;
extern volatile uint8_t button2_flag;

extern volatile uint8_t s_zero;
extern volatile uint8_t p_peak;
extern volatile uint8_t s_peak;
extern volatile uint8_t p_zero;

extern volatile uint8_t pri_switch_on;
extern volatile uint8_t sec_switch_on;

#endif