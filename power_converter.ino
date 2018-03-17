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
    while (comparator_timer < 1000) {
        //while(!adc->isComplete(ADC_1));
        if (loadVoltage() < 200) {
            if (p_peak == 1) {
                digitalWriteFast(pri_switch, LOW);
                pri_switch_on = OFF;
                s_zero = 0;
                p_peak = 0;
            }
            else if (s_zero == 1) {
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
    while (comparator_timer < 200) {
        //while(!adc->isComplete(ADC_1));
        if (loadVoltage() > 8) {
            if (s_peak == 1) {
                digitalWriteFast(sec_switch, LOW);
                sec_switch_on = OFF;
                p_zero = 0;
                s_peak = 0;
            }
            else if (p_zero == 1) {
                digitalWriteFast(sec_switch, HIGH);
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

    // run function if button pressed
    if (button1_flag) {
        digitalWriteFast(blue, LOW);
        for (int i = 0; i < 1; i++) {
            timedSquare(10, 10, 200);
        }
        digitalWriteFast(blue, HIGH);
        button1_flag = 0;
    }
}

// generates square wave using timing control method
void timedSquare(unsigned long on_time_milli, unsigned long off_time_milli, float voltage) {
    elapsedMillis pulse_timer;
    float buff[100];
    int counter= 0;
    float load_voltage;

    // boost up and hold at voltage
    pulse_timer = 0;
    while (pulse_timer < on_time_milli) {
        while(!adc->isComplete(ADC_1));
        load_voltage = loadVoltage();
        if (counter < 100) {
            buff[counter] = load_voltage;
            counter++;
        }
        if (load_voltage < voltage) { // TODO: apply hysteresis to this threshold
            if (load_voltage > 0.99 * voltage) {
                timedBoost(2,1);
            }
            else {
                timedBoost(5,2);
            }
        }
        //Serial.println(load_voltage);
    }

    // buck down and stay at 0
    pulse_timer = 0;
    while (pulse_timer < off_time_milli) {
        while(!adc->isComplete(ADC_1));
        if (loadVoltage() > 8) {
            timedBuck(0,5); // 0, 5 for 500V
        }
    }
    for (int i = 0; i < counter; i++) {
        Serial.println(buff[i]);
    }
}


void comparatorSquare() {

}



