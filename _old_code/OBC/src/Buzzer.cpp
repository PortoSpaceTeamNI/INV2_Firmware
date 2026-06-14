#include "Buzzer.h"
//#include "VeryImportantFile.h"  // For success_song function



void play_buzzer(int frequency, int duration) {
    tone(BUZZER_PIN, frequency, duration);
}

void stop_buzzer() {
    noTone(BUZZER_PIN);
}

void play_buzzer_error() {
    for(int i = 0; i < 4; i++) {
        play_buzzer(200, 500); 
        delay(1000);
    }
}

void play_buzzer_success() {
    
    for(int i = 0; i < 3; i++) {
        play_buzzer(1500, 75); 
        delay(150);
    }
    
    //success_song(BUZZER_PIN);
}

void setup_buzzer() {
    pinMode(BUZZER_PIN, OUTPUT);
}