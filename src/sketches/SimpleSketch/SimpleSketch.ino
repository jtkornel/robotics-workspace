
#include "MeAuriga.h"

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("LED ON");
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("LED OFF");
    delay(2000);
}

