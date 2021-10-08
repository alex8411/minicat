/*

======================== roboticsformakers.com MiniCat ======================
==                            First steps                                  ==
==                            Buzzer Test                                  ==
=============================================================================

Copyright (c) 2020 QI Informatique.
Code inspired from Rongzhong Li


Test of the buzzer & Arduino Nano 

This code makes the active buzzer playing a melody.

*/

#define BUZZER 4 // buzzer pin definition


void beep(int note, float duration = 10, int pause = 0, byte repeat = 1) { //function for beeping 
  // buzzer has a certain tone/note (per default duration = 10) 
  if (note == 0) { 
    analogWrite(BUZZER, 0); //we shut off the buzzer
    delay(duration); //we wait for a time = "duration" (in milliseconds)
    return; //we leave the function 
  }

  int freq = 220 * pow(1.059463, note); //known formula for the transition between a tone (note)
  float period = 1000000.0 / freq / 2.0; //known formula for the transition between the freq of the tone and the period
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period * 2) {
      analogWrite(BUZZER, 150);      //we turn on the buzzer
      delayMicroseconds(period);          //we wait the time of a "period" (in microseconds)
      analogWrite(BUZZER, 0);       //we turn off the buzzer
      delayMicroseconds(period);         //we wait the time of a "period" (in microseconds)
    }
    delay(pause);
  }
}

void playMelody(byte m[], int len) { // fonction that plays a list of tones one after another 
  for (int i = 0; i < len; i++)
    beep(m[i], 1000 / m[len + i], 100); // for all tones of the list of tones we play the tone
}
void setup() {
  

  Serial.begin(57600); //initialisation of the serial monitor
  pinMode(BUZZER, OUTPUT); //buzzer defined as an output : we send him commands

}


void loop() {
  
  
  byte melody[] = {8, 13, 10, 13, 8, 0, 5, 8, 3, 5, 8,   // definition of the tones list as a "melody"
                   8, 8, 32, 32, 8, 32, 32, 32, 32, 32, 8
                   //8,8,16,16,8,16,16,16,16,8
                  };
  playMelody(melody, sizeof(melody) / 2); // we play the melody 
}
