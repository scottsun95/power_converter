#include "converter_lib.h"

elapsedMicros comparator_timer;

void setup() {
    initialize();    

    delay(2000);

    digitalWriteFast(13, LOW);
    
    // enable gate drive
    digitalWriteFast(pri_switch_disable, LOW);
    digitalWriteFast(sec_switch_disable, LOW);

    digitalWriteFast(pri_switch, HIGH);
    pri_switch_on = ON;

    comparator_timer = 0;
    while (comparator_timer < 200) {
        if (loadVoltage() < 200) {
            if (s_zero == 1) {
                digitalWriteFast(pri_switch, HIGH);
                pri_switch_on = ON;
                s_zero = 0;
                p_peak = 0;
            }
            else if (p_peak == 1) {
                digitalWriteFast(pri_switch, LOW);
                pri_switch_on = OFF;
                s_zero = 0;
                p_peak = 0;
            }
        }
        else {
            digitalWriteFast(pri_switch, LOW);
            pri_switch_on = OFF;
            p_peak = 0;
            s_zero = 0;
        }
    }
    digitalWriteFast(pri_switch, LOW); 
    pri_switch_on = DISABLE;
    p_peak = 0;
    s_zero = 0;
}

void loop() {
    // updates alarm timer
    Alarm.delay(0);

    // run function if button pressed
    if (button1_flag) {
        digitalWriteFast(blue, LOW);
        for (int i = 0; i < 5; i++) {
            timedSquare(10, 10, 200);
        }
        digitalWriteFast(blue, HIGH);
        button1_flag = 0;
    }
}

// generates square wave using timing control method
void timedSquare(unsigned long on_time_milli, unsigned long off_time_milli, float voltage) {
    elapsedMillis pulse_timer;
    float load_voltage = loadVoltage();

    // boost up and hold at voltage
    pulse_timer = 0;
    while (pulse_timer < on_time_milli) {
        if (adc->isComplete(ADC_1)) {
            load_voltage = loadVoltage();
        }
        if (load_voltage < voltage) {
            if (load_voltage > 0.99 * voltage) {
                timedBoost(2,1);
            }
            else {
                timedBoost(5,2);
            }
        }
    }

    // buck down and stay at 0
    pulse_timer = 0;
    while (pulse_timer < off_time_milli) {
        if (adc->isComplete(ADC_1)) {
            load_voltage = loadVoltage();
        }
        if (loadVoltage() > 8) {
            timedBuck(0,5); // 0, 5 for 500V
        }
    }
}


void comparatorSquare() {

}



