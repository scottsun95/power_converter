#include "converter_lib.h"

elapsedMicros comparator_timer;

void setup() {
    initialize();    

    delay(2000);

    digitalWriteFast(13, LOW);
    
    // enable gate drive
    digitalWriteFast(pri_switch_disable, LOW);
    digitalWriteFast(sec_switch_disable, LOW);

    s_zero = 0;
    p_peak = 0;
    digitalWriteFast(pri_switch, HIGH);
    pri_switch_on = ON;

    comparator_timer = 0;
    while (comparator_timer < 800) {
        if (loadVoltage() < 200) {
            /*if (p_peak == 1) {
                digitalWriteFast(pri_switch, LOW);
                pri_switch_on = OFF;
                s_zero = 0;
                p_peak = 0;
            }
            else*/ if (s_zero == 1) {
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
            /*if (s_peak == 1) {
                digitalWriteFast(sec_switch, LOW); // consider writing in isr only for switch-off
                sec_switch_on = OFF;
                p_zero = 0;
                s_peak = 0;
            }
            else */if (p_zero == 1) {
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
    
}

void loop() {
    // updates alarm timer
    Alarm.delay(0);

    if (button1_flag || button2_flag) {
        digitalWriteFast(blue, LOW);
        timedSquare(10, 10, 200);
        digitalWriteFast(blue, HIGH);
        button1_flag = 0;
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
            if (load_voltage < 0.94 * voltage) {
                timedBoost(2,1);
            }
            else if (load_voltage < 0.98 * voltage) {
                timedBoost(1,1);
                delayMicroseconds(8);
            }
        }
        else {
            if (loadVoltage() > 0.9 * voltage) {
                level = 1;
            }
            timedBoost(5,2);
        }
    }

    // buck down and stay at 0
    pulse_timer = 0;
    while (pulse_timer < off_time_milli) {
        if (adc->isComplete(ADC_1)) {
            load_voltage = loadVoltage();
        }
        if (load_voltage > 8) {
            timedBuck(1,5); // 0, 5 for 500V
        }
    }
}


void comparatorSquare() {

}



