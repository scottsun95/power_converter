#include "converter_lib.h"

//Bounce pushbutton1 = Bounce(button1, 10);  // 10 ms debounce
//Bounce pushbutton2 = Bounce(button2, 10);  // 10 ms debounce

ADC *adc = new ADC();

volatile uint8_t button1_flag = 0;
volatile uint8_t button2_flag = 0;

volatile uint8_t s_zero = 0;
volatile uint8_t p_peak = 0;
volatile uint8_t s_peak = 0;
volatile uint8_t p_zero = 0;

/*
	Initializes circuit board with all supply rails enabled 
	and all active circuitry disabled
*/
void initialize() {
	// Turn Teensy LED on
    pinMode(13, OUTPUT);
    digitalWriteFast(13, HIGH);

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

    // Turn on load voltage sense
    pinMode(load_sense_disable, OUTPUT); // set to INPUT to turn off
    digitalWriteFast(load_sense_disable, LOW);

    // Turn off input voltage sense
    pinMode(input_sense_disable, OUTPUT);
    digitalWriteFast(input_sense_disable, HIGH);

    // configure ADC
    pinMode(input_sense, INPUT);
    pinMode(load_sense, INPUT);

    adc->setReference(ADC_REFERENCE::REF_EXT, ADC_0);
    adc->setReference(ADC_REFERENCE::REF_EXT, ADC_1);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS);
    adc->setAveraging(8);
    adc->setResolution(adc_res_bits, ADC_0);
    adc->setResolution(adc_res_bits, ADC_1);
    adc->startContinuous(load_sense, ADC_1);
    
    // configure pushbuttons
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
    attachInterrupt(S2, s_curr_zero, RISING);

    // Configure I2C bus for DACs
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.setDefaultTimeout(200000);

    // Transmit to Slave
    Wire.beginTransmission(P_DAC);  // Slave address
    Wire.write(0b00101111);       // Set both of DAC1 voltages to the same
    Wire.write(0b00111110); 
    Wire.write(0b10000000); 
    Wire.endTransmission();         // Transmit to Slave

    Wire.beginTransmission(S_DAC);  // Slave address
    Wire.write(0b00100001);       // Write to I2C
    Wire.write(0b01110000); 
    Wire.write(0); 
    Wire.endTransmission();
    
    Wire.beginTransmission(S_DAC);  // Slave address
    Wire.write(0b00100000);       // Write to I2C
    Wire.write(0b01111111); 
    Wire.write(0); 
    Wire.endTransmission();

}


/*
	Obtains input voltage reading. 
	TODO: May want to convert to IntervalTimer non-blocking
*/
float inputVoltage() {
	digitalWriteFast(input_sense_disable, LOW);

	// obtain ADC value and convert to voltage
	float voltage = adc->analogRead(A0, ADC_0) / adc_res * aref_voltage * 2;

	digitalWriteFast(input_sense_disable, HIGH);
	return voltage;
}


/*
	Obtains load voltage reading
*/
float loadVoltage() {
	// obtain ADC value and convert to voltage
	float voltage = adc->analogReadContinuous(ADC_1) / adc_res * aref_voltage * 400;
	return voltage;
}

void button1Pressed() {
    button1_flag = 1;
}

void button2Pressed() {
    button1_flag = 1;
}

void p_curr_peak() {
    p_peak = 1;
    s_zero = 0;
}

void s_curr_zero() {
    s_zero = 1;
}

void p_curr_zero() {
    p_zero = 1;
}

void s_curr_peak() {
    s_peak = 1;
}
