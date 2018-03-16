#include "converter_lib.h"

//Bounce pushbutton1 = Bounce(button1, 10);  // 10 ms debounce

// voltage sensing
ADC *adc = new ADC();
DMAChannel dma;

float input_voltage = 0;
volatile uint16_t load_adc[1];

// button flags
volatile uint8_t button1_flag = 0;
volatile uint8_t button2_flag = 0;

// comparator flags
volatile uint8_t s_zero = 0;
volatile uint8_t p_peak = 0;
volatile uint8_t s_peak = 0;
volatile uint8_t p_zero = 0;
volatile int8_t pri_switch_on = DISABLE;
volatile int8_t sec_switch_on = DISABLE;

/*
	Initializes circuit board with all supply rails enabled 
	and all active circuitry disabled
*/
void initialize() {
	// Turn Teensy LED on
    pinMode(13, OUTPUT);
    digitalWriteFast(13, HIGH);

    Serial.begin(9600);

    // Disable secondary-side gate drivers
    pinMode(sec_switch, OUTPUT);
    digitalWriteFast(sec_switch, LOW);
    pinMode(sec_switch_disable, OUTPUT);
    digitalWriteFast(sec_switch_disable, HIGH);

    // Disable primary-side gate drivers
    pinMode(pri_switch, OUTPUT);
    digitalWriteFast(pri_switch, LOW);
    pinMode(pri_switch_disable, OUTPUT);
    digitalWriteFast(pri_switch_disable, HIGH);

    // Turn -Vcc rail on
    pinMode(neg_vcc_en, OUTPUT);
    digitalWriteFast(neg_vcc_en, HIGH);

    // configure ADC & DMA
    pinMode(input_sense, INPUT);
    pinMode(load_sense, INPUT);

    dmaInit();

    adc->setReference(ADC_REFERENCE::REF_EXT, ADC_0);
    adc->setReference(ADC_REFERENCE::REF_EXT, ADC_1);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED, ADC_0);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_0);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_1);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_1);
    adc->setAveraging(16, ADC_0);
    adc->setAveraging(4, ADC_1);
    adc->setResolution(adc_res_bits, ADC_0);
    adc->setResolution(adc_res_bits, ADC_1);

    adc->enableDMA(ADC_1);
    adc->startContinuous(load_sense, ADC_1);

    // Turn on load voltage sense
    pinMode(load_sense_disable, OUTPUT); // set to INPUT to turn off
    digitalWriteFast(load_sense_disable, LOW);

    // Set up input voltage sense
    pinMode(input_sense_disable, OUTPUT);
    input_voltage = inputVoltage();
    Alarm.timerRepeat(60, intervalReadInputVoltage);
    
    // configure pushbuttons and attach interrupts
    pinMode(button1, INPUT_PULLUP);
    attachInterrupt(button1, button1Pressed, FALLING);
    pinMode(button2, INPUT_PULLUP);
    attachInterrupt(button2, button2Pressed, FALLING);

    // configure LED
    pinMode(red, OUTPUT);
    digitalWriteFast(red, HIGH);
    pinMode(green, OUTPUT);
    digitalWriteFast(green, HIGH);
    pinMode(blue, OUTPUT);
    digitalWriteFast(blue, HIGH);

    // configure interrupt pins
    pinMode(P1, INPUT);
    attachInterrupt(P1, p_curr_peak, RISING);
    pinMode(P2, INPUT);
    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    attachInterrupt(S1, s_curr_zero, RISING);

    // Configure I2C bus for DACs
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.setDefaultTimeout(200000);

    Wire.beginTransmission(P_DAC);  // Slave address
    Wire.write(0b00100001);         // Set output B of DAC1 
    Wire.write(0b01010000);         // 0b00111110, 0b10000000 for 0.7V, which is actually 0.9A
    Wire.write(0b10000000); 
    Wire.endTransmission();         // Transmit to Slave

    Wire.beginTransmission(P_DAC);  
    Wire.write(0b00100000);         // Set output A of DAC1 
    Wire.write(0b00000001);         
    Wire.write(0b00000000);         
    Wire.endTransmission();         

    Wire.beginTransmission(S_DAC);  
    Wire.write(0b00100001);         // Set output B of DAC2
    Wire.write(0b01110000); 
    Wire.write(0); 
    Wire.endTransmission();
    
    Wire.beginTransmission(S_DAC);  
    Wire.write(0b00100000);       // Set output A of DAC2
    Wire.write(0b01111100); 
    Wire.write(0); 
    Wire.endTransmission();

    pinMode(3, OUTPUT); // trigger for load voltage sense
}

/****************************
    Voltage Sense Functions
****************************/

//	Obtains input voltage reading. 
float inputVoltage() {
	digitalWrite(input_sense_disable, LOW);

	// obtain ADC value and convert to voltage
	float voltage = adc->analogRead(A0, ADC_0) / adc_res * aref_voltage * 2.016;

	digitalWrite(input_sense_disable, HIGH);
	return voltage;
}

void intervalReadInputVoltage() {
    input_voltage = inputVoltage();
    Serial.print("Input Voltage: ");
    Serial.println(input_voltage);
}

// Converts ADC DMA reading to voltage
float loadVoltage() {
    return load_adc[0] / adc_res * aref_voltage * 284.8;
}

/************************
    Switching Functions
*************************/

// timing-based switching 
void timedBoost(unsigned int on, unsigned int off) { // 5ms/4ms -> 5ms/2ms works ok
    digitalWriteFast(pri_switch, HIGH);
    delayMicroseconds(on);
    digitalWriteFast(pri_switch, LOW);
    delayMicroseconds(off);
}

void timedBuck(unsigned int on, unsigned int off) {
    digitalWriteFast(sec_switch, HIGH);
    delayMicroseconds(on);
    digitalWriteFast(sec_switch, LOW);
    delayMicroseconds(off);
}

// comparator based switching
void comparatorBoost() {

}

void comparatorBuck() {

}

/************************
    Interrupt Functions
*************************/

// button press interrupts
void button1Pressed() {
    button1_flag = 1;
}

void button2Pressed() {
    button2_flag = 1;
}

// comparator threshold interrupts
void p_curr_peak() {
    if (pri_switch_on == ON) {
        p_peak = 1;
    }
    else {
        p_peak = 0;
    }
}

void s_curr_zero() {
    if (pri_switch_on == OFF) {
        s_zero = 1;
    }
    else {
        s_zero = 0;
    }
}

void p_curr_zero() {
    if (sec_switch_on == OFF) {
        p_zero = 1;
    }
    else {
        p_zero = 0;
    }
}

void s_curr_peak() {
    if (sec_switch_on == ON) {
        s_peak = 1;
    }
    else {
        s_peak = 0;
    }
}

// Configures DMA for ADC readings
void dmaInit() {
  dma.source(*(uint16_t*) &ADC1_RA);
  dma.destinationBuffer(load_adc, sizeof(load_adc));
  dma.attachInterrupt(dma_isr);
  dma.interruptAtCompletion();
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
  dma.enable();
}

void dma_isr() {
    dma.clearInterrupt();
    digitalWriteFast(3, HIGH); // for checking sample freq.
    digitalWriteFast(3, LOW);
}



