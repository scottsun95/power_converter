#include "converter_lib.h"

//Bounce pushbutton1 = Bounce(button1, 10);  // 10 ms debounce
//Bounce pushbutton2 = Bounce(button2, 10);  // 10 ms debounce

ADC *adc = new ADC();
ADC::Sync_result result;

// global voltage sense values
float input_voltage = 0;
float load_voltage = 0;

// button flags
volatile uint8_t button1_flag = 0;
volatile uint8_t button2_flag = 0;

// switching variables
volatile uint8_t s_zero = 0;
volatile uint8_t p_peak = 0;
volatile uint8_t s_peak = 0;
volatile uint8_t p_zero = 0;
int8_t pri_switch_on = DISABLE;
int8_t sec_switch_on = DISABLE;

float alpha = 0.9;

// defined waveforms
float sine_wave[wave_points];


/*
	Initializes circuit board with all supply rails enabled 
	and all active circuitry disabled
*/
void initialize() {
	// Turn Teensy LED on
    pinMode(13, OUTPUT);
    digitalWriteFast(13, HIGH);

    Serial.begin(115200);

    // Disable secondary-side gate drivers
    pinMode(sec_switch, OUTPUT);
    digitalWriteFast(sec_switch, LOW);
    CORE_PIN20_CONFIG &= ~PORT_PCR_SRE; //slew rate limiting off
    pinMode(sec_switch_disable, OUTPUT);
    digitalWriteFast(sec_switch_disable, HIGH);

    // Disable primary-side gate drivers
    pinMode(pri_switch, OUTPUT);
    digitalWriteFast(pri_switch, LOW);
    CORE_PIN22_CONFIG &= ~PORT_PCR_SRE; //slew rate limiting off
    pinMode(pri_switch_disable, OUTPUT);
    digitalWriteFast(pri_switch_disable, HIGH);

    // Turn -Vcc rail on
    pinMode(neg_vcc_en, OUTPUT);
    digitalWriteFast(neg_vcc_en, HIGH);

    // configure ADC
    pinMode(input_sense, INPUT);
    pinMode(load_sense, INPUT);

    //dmaInit();

    adc->setReference(ADC_REFERENCE::REF_EXT, ADC_0);
    adc->setReference(ADC_REFERENCE::REF_EXT, ADC_1);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_0);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_0);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_1);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_1);
    adc->setAveraging(4, ADC_0);
    adc->setAveraging(4, ADC_1);
    adc->setResolution(adc_res_bits, ADC_0);
    adc->setResolution(adc_res_bits, ADC_1);

    //adc->enableDMA(ADC_0);
    //adc->enableDMA(ADC_1);
    adc->startSynchronizedContinuous(load_sense, load_sense);
    //adc->startContinuous(load_sense, ADC_1);
    //adc->startContinuous(load_sense, ADC_0);

    // Turn on load voltage sense
    pinMode(load_sense_disable, OUTPUT); // set to INPUT to turn off
    digitalWriteFast(load_sense_disable, LOW);

    // Set up input voltage sense
    pinMode(input_sense_disable, OUTPUT);
    input_voltage = inputVoltage();
    //Alarm.timerRepeat(60, intervalReadInputVoltage);
    
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
    pinMode(P2, INPUT);
    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    if (!TIME_MODE) {
        attachInterrupt(P1, p_curr_peak, RISING);
        attachInterrupt(P2, p_curr_zero, RISING);
        attachInterrupt(S1, s_curr_zero, RISING);
        attachInterrupt(S2, s_curr_peak, RISING);
    }

    // Configure I2C bus for DACs
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.setDefaultTimeout(200000);

    // Transmit to Slave
    Wire.beginTransmission(P_DAC);  // Slave address
    Wire.write(0b00100001);         // Set output B of DAC1 
    Wire.write(0b01010000);         // 0b00111110, 0b10000000 for 0.7V, which is actually 0.9A
    Wire.write(0b10000000); 
    Wire.endTransmission();         // Transmit to Slave

    Wire.beginTransmission(P_DAC);  
    Wire.write(0b00100000);         // Set output A of DAC1 
    Wire.write(0b00000010);         
    Wire.write(0b00000000);         
    Wire.endTransmission();         

    Wire.beginTransmission(S_DAC);  
    Wire.write(0b00100001);         // Set output B of DAC2
    Wire.write(0b01110000); 
    Wire.write(0); 
    Wire.endTransmission();
    
    Wire.beginTransmission(S_DAC);  
    Wire.write(0b00100000);       // Set output A of DAC2 to 0.6V below B
    Wire.write(0b00010000); 
    Wire.write(0b00000000); 
    Wire.endTransmission();


    // define sine wave
    for (int i = 0; i < wave_points; i++) {
        sine_wave[i] = voltage_amplitude * (-cos(2*PI*freq*i*sample_time) + 1) / 2;
    }

    // turn on cycle counter
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

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

//	Obtains load voltage reading
float loadVoltage() {
    //return (load_adc[0] + load_adc[1])/ 2.0 / adc_res * aref_voltage * 185;
    result = adc->readSynchronizedContinuous();
    load_voltage = alpha * 0.5*(result.result_adc0 + result.result_adc1) / adc_res * aref_voltage * 193 
        + (1.0-alpha) * load_voltage;
    return load_voltage;
}

/************************
    Switching Functions
*************************/

// timing-based switching 
void timedBoost(float on, float off) { // 5ms/4ms -> 5ms/2ms works ok
    digitalWriteFast(pri_switch, HIGH);
    delayMicroCycles(on);

    digitalWriteFast(pri_switch, LOW);
    delayMicroCycles(off);
}

void timedBuck(float on, float off) {
    digitalWriteFast(sec_switch, HIGH);
    delayMicroCycles(on);

    digitalWriteFast(sec_switch, LOW);
    delayMicroCycles(off);
}

void comparatorBoost() {

}

void comparatorBuck() {

}

void hybridBoost() {

}

void hybridBuck() {

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
        digitalWriteFast(pri_switch, LOW);
        pri_switch_on = OFF;
        s_zero = 0;
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

void s_curr_peak() {
    if (sec_switch_on == ON) {
        digitalWriteFast(sec_switch, LOW);
        sec_switch_on = OFF;
        p_zero = 0;
    }
    else {
        s_peak = 0;
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

/***********************
    Waveform Generator
************************/
void waveform_gen(float* waveform) {
    elapsedMicros loop_timer;
    float error = 0;
    float error_integral = 0; // goes up to 130000 by itself
    float prop_gain = 0.3;
    float int_gain = 0.01;
    float time = 0;

    for (int i = 0; i < wave_points; i++) {
        loop_timer = 0;

        while (loop_timer < sample_time * 1e6) {
            while(!adc->isComplete(ADC_1));
            load_voltage = loadVoltage();
            if (load_voltage > 550) return; // emergency failsafe
            error = waveform[i] - load_voltage;

            error_integral += error;
            time = prop_gain * error + int_gain * error_integral;
            if (time > 0) {
                time = time < 5 ? time : 5;
                if (time > 0.011) {
                    timedBoost(time, 0.5*time); 
                }
                else if (time < 0.011 && time > 0.007) {
                    digitalWriteFast(pri_switch, HIGH);
                    digitalWriteFast(pri_switch, LOW); 
                }
            }
            else if (time < 0) {
                time = -0.001*time;
                time = time < 0.1 ? time : 0.1;
                if (time > 0.011) {
                    timedBuck(time, 2*time); 
                }
                else if (time < 0.011 && time > 0.007) {
                    digitalWriteFast(sec_switch, HIGH);
                    digitalWriteFast(sec_switch, LOW); 
                }
            }
        }
        error_integral = 0;
    }
}

/*********************
    Timing Functions
**********************/
// delays based on number of CPU cycles, disables interrupts
void delayMicroCycles(float microseconds) {
    unsigned long cycles = ARM_DWT_CYCCNT;
    unsigned long num_cycles_delay = microseconds * F_CPU * 1e-6 - 5; //5 offset for added instructions
    /*if (!TIME_MODE) {
        cli();
    }*/
    while(ARM_DWT_CYCCNT < num_cycles_delay + cycles);
    /*if (!TIME_MODE) {
        sei();
    }*/
}


/*******************
    Math Functions
********************/
// takes average of buffer
float average(float* buffer, int window) {
    float sum = 0.;
    for (int i = 0; i < window; i++) {
        sum += buffer[i];
    }
    return sum / window;
}

/*
// Configures DMA for ADC readings
void dmaInit() {
    dma.source(*(uint16_t*) &ADC1_RA);
    dma.destinationBuffer(load_adc, sizeof(load_adc[0]));
    dma.attachInterrupt(dma_isr);
    dma.interruptAtCompletion();
    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
    dma.enable();

    dma2.source(*(uint16_t*) &ADC0_RA);
    dma2.destinationBuffer(load_adc + 1, sizeof(load_adc[1]));
    dma2.attachInterrupt(dma2_isr);
    dma2.interruptAtCompletion();
    dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
    dma2.enable();
}

void dma_isr() {
    dma.clearInterrupt();
    //digitalWriteFast(3, HIGH); // for checking sample freq.
    //digitalWriteFast(3, LOW);
}

void dma2_isr() {
    dma2.clearInterrupt();
    //digitalWriteFast(4, HIGH); // for checking sample freq.
    //digitalWriteFast(4, LOW);
}

*/

