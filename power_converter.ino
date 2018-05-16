#include "converter_lib.h"

elapsedMicros comparator_timer;

void setup() {
    initialize();    

    delay(1000);

    digitalWriteFast(13, LOW);
    
    // enable gate drive
    digitalWriteFast(pri_switch_disable, LOW);
    digitalWriteFast(sec_switch_disable, LOW);
}

void loop() {
    if (button1_flag) {
        digitalWriteFast(blue, LOW);
        for (int i = 0; i < 25; i++) {
            waveform_gen(square_wave);
        }
        digitalWriteFast(blue, HIGH);
        button1_flag = 0;
    }
    else if (button2_flag) {
        digitalWriteFast(red, LOW);
        for (int i = 0; i < 5; i++) {
            waveform_gen(sine_wave);
        }
        digitalWriteFast(red, HIGH);
        button2_flag = 0;
    }
}

// generates square wave using open loop timing control method. Use waveform_generator() instead
// except when fine granular control is needed
void timedSquare(unsigned long on_time_milli, unsigned long off_time_milli, float voltage) {
    elapsedMillis pulse_timer;
    float load_voltage = 0;
    uint8_t level = 0;

    // boost up and hold at voltage
    pulse_timer = 0;
    while (pulse_timer < on_time_milli) {
        if (level == 1) {
            while (!adc->isComplete(ADC_1));
            load_voltage = loadVoltage();
            if (load_voltage < 0.98 * voltage) {
                timedBoost(4,8);
            }
        }
        else {
            if (loadVoltage() > 0.90 * voltage) {
                level = 1;
            }
            timedBoost(5,10);
        }
    }
    // buck down and stay at 0
    pulse_timer = 0;
    while (pulse_timer < off_time_milli) {
        while (!adc->isComplete(ADC_1));
        load_voltage = loadVoltage();
        Serial.println(load_voltage);
        if (load_voltage > 400) {
            timedBuck(0.1,0.5); 
        }
        else if (load_voltage > 1) {
            timedBuck(0.3,0.5); 
        }
    }
}



