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
    for (int i=0; i < 17; i++) {
        load_sense_timer = 0;
        while (load_sense_timer < 100) {
            if (loadVoltage() < 150) {
                if (s_zero == 1) {
                    digitalWriteFast(pri_switch, HIGH);
                    s_zero = 0;
                }
                else if (p_peak == 1) {
                    digitalWriteFast(pri_switch, LOW);
                    p_peak = 0;
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
    //Serial.println(samples[0]);
    if (button1_flag || button2_flag) {
        digitalWriteFast(blue, LOW);
        for (int i = 0; i < 50; i++) {
            timedSquare(10, 10, 200);
        }
        digitalWriteFast(blue, HIGH);
        button1_flag = 0;
        button2_flag = 0;
    }
}

// generates square wave using timing control method
void timedSquare(unsigned long on_time_milli, unsigned long off_time_milli, float voltage) {
    elapsedMillis pulse_timer;

    // boost up and hold at voltage
    pulse_timer = 0;
    while (pulse_timer < on_time_milli) {
        float load_voltage = loadVoltage();
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
        if (loadVoltage() > 5) {
            timedBuck(1,5); // 0, 5 for 500V
        }
    }
}


void comparatorSquare() {

}



