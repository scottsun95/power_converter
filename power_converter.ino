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




