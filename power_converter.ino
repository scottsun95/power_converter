#include "converter_lib.h"

elapsedMicros comparator_timer;

void setup() {
    initialize();    

    delay(2000);

    digitalWriteFast(13, LOW);
    
    // enable gate drive
    digitalWriteFast(pri_switch_disable, LOW);
    digitalWriteFast(sec_switch_disable, LOW);

/*    s_zero = 0;
    p_peak = 0;
    digitalWriteFast(pri_switch, HIGH);
    pri_switch_on = ON;

    comparator_timer = 0;
    while (comparator_timer < 800) {
        if (loadVoltage() < 200) {
            if (s_zero == 1) {
                digitalWriteFast(pri_switch, HIGH);
                pri_switch_on = ON;
                s_zero = 0;
                p_peak = 0;
            }
        }
        else {
            digitalWriteFast(pri_switch, LOW);
            pri_switch_on = OFF;
        }
    }
    digitalWriteFast(pri_switch, LOW); 
    pri_switch_on = DISABLE;
    p_peak = 0;
    s_zero = 0;
    
    // buck
    digitalWriteFast(sec_switch, HIGH);
    sec_switch_on = ON;

    comparator_timer = 0;
    while (comparator_timer < 300) {
        if (loadVoltage() > 8) {
            if (p_zero == 1) {
                digitalWriteFast(sec_switch, HIGH); // switch-on still raises a ready flag
                sec_switch_on = ON;
                p_zero = 0;
                s_peak = 0;
            } 
        }
        else {
            digitalWriteFast(sec_switch, LOW);
            sec_switch_on = DISABLE;
        }
    }
    digitalWriteFast(sec_switch, LOW); 
    sec_switch_on = DISABLE;
    s_peak = 0;
    p_zero = 0;
    */
}

void loop() {
    // updates alarm timer
    //Alarm.delay(0);

    if (button1_flag) {
        digitalWriteFast(blue, LOW);
        for (int i = 0; i < 20; i++) {
            timedSquare(50, 50, 200);
        }
        digitalWriteFast(blue, HIGH);
        button1_flag = 0;
    }
    else if (button2_flag) {
        digitalWriteFast(red, LOW);
        for (int i = 0; i < 20; i++) {
            waveform_gen(sine_wave);
        }
        digitalWriteFast(red, HIGH);
        button2_flag = 0;
    }
}

// generates square wave using timing control method
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
            if (load_voltage < 0.95 * voltage) {
                timedBoost(3,2);            // reduce for piezo
            }
            else if (load_voltage < 0.98 * voltage) {
                timedBoost(0.5,1);
            }
        }
        else {
            if (loadVoltage() > 0.90 * voltage) {
                level = 1;
            }
            timedBoost(5,3);
        }
    }

    // buck down and stay at 0
    pulse_timer = 0;
    while (pulse_timer < off_time_milli) {
        while (!adc->isComplete(ADC_1));
        load_voltage = loadVoltage();
        if (load_voltage > 400) {
            timedBuck(0.1,0.5); 
        }
        else if (load_voltage > 3) {
            timedBuck(0.3,0.5); 
        }
    }
}


void comparatorSquare() {

}



