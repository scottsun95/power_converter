#include "converter_lib.h"

// ADC objects
ADC *adc = new ADC();
ADC::Sync_result result;

// global voltage sense values
float input_voltage = 0;
float load_voltage = 0;

// button flags
volatile uint8_t button1_flag = 0;
volatile uint8_t button2_flag = 0;

// define waveforms
float sine_wave[wave_points];
float saw_wave[wave_points];
float square_wave[wave_points];


/*
	Initializes circuit board with all supply rails enabled 
*/
void initialize() {
	// Turn Teensy LED on
    pinMode(13, OUTPUT);
    digitalWriteFast(13, HIGH);

    Serial.begin(115200);

    // Disable diode
    pinMode(diode_disable, OUTPUT);
    digitalWriteFast(diode_disable, HIGH);

    // Enable 5V rail
    pinMode(gate_supply_enable, OUTPUT);
    digitalWriteFast(gate_supply_enable, HIGH);
    delay(100);

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

    // Enable diode
    digitalWriteFast(diode_disable, LOW);

    // configure ADC
    pinMode(input_sense, INPUT);
    pinMode(load_sense, INPUT);

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
    adc->startSynchronizedContinuous(load_sense, load_sense);

    // Turn on load voltage sense
    pinMode(load_sense_disable, OUTPUT); // set to INPUT to turn off
    digitalWriteFast(load_sense_disable, HIGH);

    // Set up input voltage sense
    pinMode(input_sense_disable, OUTPUT);
    input_voltage = inputVoltage();
    
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

    // turn on clock cycle counter
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;


/*********************************************
    WAVEFORM DEFINITIONS
    Define waveforms with phase shift 
    so they start and stop at 0V.
    Constants located in converter_lib.h.
**********************************************/
    // sinusoid wave
    for (int i = 0; i < wave_points; i++) {
        sine_wave[i] = voltage_amplitude * (-cos(2*PI*freq*i*sample_time) + 1) / 2;
    }

    // sawtooth wave
    float duty_cycle = 0.05; // usually stay above 5% and above 95%
    for (int i = 0; i < (int) wave_points * duty_cycle; i++) {
        saw_wave[i] = voltage_amplitude / (wave_points * duty_cycle) * i;
    }
    for (int i = (int) wave_points * duty_cycle; i < wave_points; i++) {
        saw_wave[i] = voltage_amplitude/(1-duty_cycle) - voltage_amplitude / (wave_points * (1-duty_cycle)) * i;
    }

    // square wave
    duty_cycle = 0.5; // usually stay above 5% and below 95%
    for (int i = 0; i < (int) wave_points*(1-duty_cycle)/2; i++) {
        square_wave[i] = 0;
    }
    for (int i = (int) wave_points*(1-duty_cycle)/2; i < (int) (wave_points*(1+duty_cycle)/2); i++) {
        square_wave[i] = voltage_amplitude;
    }
    for (int i = (int) (wave_points*(1+duty_cycle)/2); i < wave_points; i++) {
        square_wave[i] = 0;
    }
}

/****************************
    Voltage Sense Functions
****************************/

//	Obtains input voltage reading. 
float inputVoltage() {
	digitalWrite(input_sense_disable, LOW);

	// obtain ADC value and convert to voltage
	float voltage = adc->analogRead(A0, ADC_0) / adc_res * aref_voltage * input_gain;

	digitalWrite(input_sense_disable, HIGH);
	return voltage;
}

//	Obtains load voltage reading
float loadVoltage() {
    result = adc->readSynchronizedContinuous();
    // exponential moving average of the average of both adc readings
    load_voltage = alpha * 0.5*(result.result_adc0 + result.result_adc1) / adc_res * aref_voltage * load_gain 
        + (1.0-alpha) * load_voltage; // 190 is magic number calibration
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

/**********************************************
    WAVEFORM GENERATOR
    Uses PI controller to adjust switch timing.
***********************************************/
void waveform_gen(float* waveform) {
    elapsedMicros loop_timer;
    float error = 0;
    float error_integral = 0;
    float prop_gain = 0.15;
    float int_gain = 0.06;
    float time = 0;

    // iterates through waveform points
    for (int i = 0; i < wave_points; i++) {
        loop_timer = 0;

        while (loop_timer < sample_time * 1e6) {
            // checks for finished load voltage conversion
            while(!adc->isComplete(ADC_1));
            load_voltage = loadVoltage();
            if (load_voltage > 550) return; // emergency voltage cutoff

            // PI calculation
            error = waveform[i] - load_voltage;
            error_integral += error;
            time = prop_gain * error + int_gain * error_integral;

            // boost mode
            if (time > 0) {
                // primary switch on-time ceiling
                time = time < 5 ? time : 5;

                // switch normally if time long enough, else go to no-delay switching
                // don't switch if load voltage close enough to setpoint
                if (time > 0.011) {
                    timedBoost(time, 2*time); 
                }
                else if (time < 0.011 && time > 0.006) {
                    // no delay switching
                    digitalWriteFast(pri_switch, HIGH);
                    digitalWriteFast(pri_switch, LOW); 
                }
            }
            // buck mode
            else if (time < 0) {
                // secondary switch on-time ceiling and scaling
                time = -0.001*time;
                time = time < 0.5 ? time : 0.5;

                // switch normally if time long enough, else go to no-delay switching
                // don't switch if load voltage close enough to setpoint
                if (time > 0.011) {
                    timedBuck(time, 2*time); 
                }
                else if (time < 0.011 && time > 0.005) {
                    digitalWriteFast(sec_switch, HIGH);
                    digitalWriteFast(sec_switch, LOW); 
                }
            }
        }
        error_integral = 0; // clear after each point to prevent integral windup
    }
}

/*********************
    Timing Functions
**********************/
// delays based on number of CPU cycles
void delayMicroCycles(float microseconds) {
    unsigned long cycles = ARM_DWT_CYCCNT;
    unsigned long num_cycles_delay = microseconds * F_CPU * 1e-6 - 5; // subtract 5 for these instructions

    while(ARM_DWT_CYCCNT < num_cycles_delay + cycles);
}


