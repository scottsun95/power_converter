#include "converter_lib.h"

elapsedMillis timer;
elapsedMicros timer2;

float buff[300];

void setup() {
    initialize();    

    delay(2000);
    digitalWriteFast(13, LOW);

    Serial.begin(9600);

    // enable gate drive
    digitalWriteFast(pri_switch_disable, LOW);
    digitalWriteFast(sec_switch_disable, LOW);

    s_zero = 0;
    p_peak = 0;
    digitalWriteFast(pri_switch, HIGH);
    // engage primary switch
    for (int i=0; i < 300; i++) {
        buff[i] = loadVoltage();
        timer2 = 0;
        while (timer2 < 10) {
            if (buff[i] < 400) {
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

        //digitalWriteFast(pri_switch, HIGH);
        //delayMicroseconds(5);
        //digitalWriteFast(pri_switch, LOW);
        //delayMicroseconds(4);
    }
    digitalWriteFast(pri_switch, LOW); 
    for (int i = 0;i<10;i++) {
        digitalWriteFast(sec_switch, HIGH);
        digitalWriteFast(sec_switch, LOW);
        delayMicroseconds(5);
    }


    //for (int i=0; i < 750; i++) {
    //    boost();
    //}

    //while (timer < 5000) {
    //   boost();
    //    delayMicroseconds(7);
    //}
    for (int i=0;i<300;i++) {
        Serial.println(buff[i]);
    }
}

void loop() {

    //float voltage = loadVoltage();
    //Serial.println(voltage,4);
    //delay(1);
    //voltage = inputVoltage();
    //Serial.println(voltage,2);
    //delay(1);
}

// timing-based boost 5ms/4ms -> 5ms/2ms works ok
void timed_boost(int on, int off) {
    digitalWriteFast(pri_switch, HIGH);
    delayMicroseconds(on);
    digitalWriteFast(pri_switch, LOW);
    delayMicroseconds(off);
}


//float* curr_boost() {
//}



