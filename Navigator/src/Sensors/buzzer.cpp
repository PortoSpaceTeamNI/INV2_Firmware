#include "Sensors/Buzzer.h"

void setup_buzzer() {
    pinMode(BUZZER, OUTPUT);
}

void play_buzzer(int frequency, int duration) {
    tone(BUZZER, frequency, duration);
}

void stop_buzzer() {
    noTone(BUZZER);
}

void play_buzzer_error() {
    for(int i = 0; i < 1; i++) {
        play_buzzer(200, 500); 
        delay(1000);
    }
}

void play_buzzer_success() {
    
    for(int i = 0; i < 3; i++) {
        play_buzzer(2000, 75); 
        delay(150);
    }
}