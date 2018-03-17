#include "converter_lib.h"

#define DEBUG_MODE 1

elapsedMicros load_sense_timer;

int counter = 0;

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

    // engage primary switch
    for (int i=0; i < 1; i++) {
        load_sense_timer = 0;
        while (load_sense_timer < 100) {
            if (load_voltage < 500) {
                if (s_zero == 1) {
                    digitalWriteFast(pri_switch, HIGH);
                    s_zero = 0;
                }
                else if (p_peak == 1) {
                    digitalWriteFast(pri_switch, LOW);
                    p_peak = 0;
                }
                if (adc->isComplete(ADC_1)) {
                    digitalWrite(3, HIGH);  // trigger pin for testing
                    digitalWrite(3, LOW);
                    load_voltage = loadVoltage(); // convert to modulo for rolling buffer
                }
            }
            else {
                digitalWriteFast(pri_switch, LOW);
                s_zero = 0;
                p_peak = 0;
            }
        }
    }
    digitalWriteFast(pri_switch, LOW); 
    for (int i = 0;i < 20;i++) {
        timedBuck(0,5);
    }
}

void loop() {
    // updates alarm timer
    Alarm.delay(0);

    if (button1_flag || button2_flag) {
        digitalWriteFast(blue, LOW);
        timedSquare(50, 50, 200);
        digitalWriteFast(blue, HIGH);
        button1_flag = 0;
        button2_flag = 0;
    }
}

void timedSquare(unsigned long on_time_milli, unsigned long off_time_milli, float voltage) {
    elapsedMillis pulse_timer;
    //for (int i = 0; i < 300; i++) { //300 for 5nF, 670 for 20nF
    //    timedBoost(5,2);
    //}
    pulse_timer = 0;
    while (pulse_timer < on_time_milli) {
        if (adc->isComplete(ADC_1)) {
            load_voltage = loadVoltage();
        }
        if (load_voltage < voltage) {
            if (load_voltage > 0.95 * voltage) {
                timedBoost(2,1);
            }
            else {
                timedBoost(5,2);
            }
        }
        // delayMicroseconds(600); // 7 for DEA, 600 for 5nF, 850 for 20nF w/ load sense on
    }
    //for (int i = 0; i < 120; i++) { // 100 for 5nF, 320 for 20nF
    pulse_timer = 0;
    while (pulse_timer < off_time_milli) {
        if (adc->isComplete(ADC_1)) {
            load_voltage = loadVoltage();
        }
        if (load_voltage > 10) {
            timedBuck(0,5);
        }
    }
}


void comparatorSquare() {

}



