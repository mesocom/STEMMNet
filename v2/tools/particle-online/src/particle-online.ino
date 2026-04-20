/*
 * Particle Boron Keep-Alive
 * A simple script to keep the device connected to the Particle Cloud.
 */

#include "Particle.h"

// SEMI_AUTOMATIC mode gives us control over when the modem powers up.
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

void setup() {
    // Explicitly command the cellular modem and cloud connection
    Particle.connect();

    // Use the onboard LED for status feedback
    pinMode(D7, OUTPUT);
}

void loop() {
    // Hearthbeat: Quick blink to show the code is running
    digitalWrite(D7, HIGH);
    delay(100);
    digitalWrite(D7, LOW);
    
    // In AUTOMATIC mode with system thread enabled, 
    // the background connection maintenance is handled for you.
    // We just loop here to keep the process alive.
    delay(10000); 
}
