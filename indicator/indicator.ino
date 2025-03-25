#include <Arduino.h>

// Pin Definitions
#define LEFT_BUTTON_PIN   18
#define RIGHT_BUTTON_PIN  19
#define LEFT_LED_PIN      4
#define RIGHT_LED_PIN     5

// State Variables
bool leftLedOn = false;
bool rightLedOn = false;
bool hazardMode = false;

// Timing for debounce and blinking
unsigned long lastLeftButtonPress = 0;
unsigned long lastRightButtonPress = 0;
unsigned long lastBlinkTime = 0;
const unsigned long DEBOUNCE_DELAY = 250;  // Prevent rapid toggling
const unsigned long BLINK_INTERVAL = 500;  // 500ms blink interval for hazard mode

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);

    // Set button pins as inputs with pull-up resistors
    pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);

    // Set LED pins as outputs
    pinMode(LEFT_LED_PIN, OUTPUT);
    pinMode(RIGHT_LED_PIN, OUTPUT);

    // Initial LED states
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);

    Serial.println("Indicator Control System Started");
}

void loop() {
    unsigned long currentTime = millis();

    // Check left button
    if (digitalRead(LEFT_BUTTON_PIN) == LOW) {
        if (currentTime - lastLeftButtonPress > DEBOUNCE_DELAY) {
            // If right button is also pressed, toggle hazard mode
            if (digitalRead(RIGHT_BUTTON_PIN) == LOW) {
                hazardMode = !hazardMode;
                // Reset individual indicators when entering/exiting hazard mode
                leftLedOn = false;
                rightLedOn = false;
                Serial.println(hazardMode ? "Hazard Mode ON" : "Hazard Mode OFF");
            } else {
                // Toggle left LED
                if (!hazardMode) {
                    leftLedOn = !leftLedOn;
                    rightLedOn = false; // Ensure only one LED is on at a time
                    Serial.println("Left Indicator Toggled");
                }
            }
            lastLeftButtonPress = currentTime;
        }
    }

    // Check right button
    if (digitalRead(RIGHT_BUTTON_PIN) == LOW) {
        if (currentTime - lastRightButtonPress > DEBOUNCE_DELAY) {
            // If left button is also pressed, toggle hazard mode
            if (digitalRead(LEFT_BUTTON_PIN) == LOW) {
                hazardMode = !hazardMode;
                // Reset individual indicators when entering/exiting hazard mode
                leftLedOn = false;
                rightLedOn = false;
                Serial.println(hazardMode ? "Hazard Mode ON" : "Hazard Mode OFF");
            } else {
                // Toggle right LED
                if (!hazardMode) {
                    rightLedOn = !rightLedOn;
                    leftLedOn = false; // Ensure only one LED is on at a time
                    Serial.println("Right Indicator Toggled");
                }
            }
            lastRightButtonPress = currentTime;
        }
    }

    // Update LEDs based on mode
    if (hazardMode) {
        // Hazard mode: alternating blink
        if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
            digitalWrite(LEFT_LED_PIN, !digitalRead(LEFT_LED_PIN));
            digitalWrite(RIGHT_LED_PIN, !digitalRead(RIGHT_LED_PIN));
            lastBlinkTime = currentTime;
        }
    } else {
        // Normal mode: individual LED control
        digitalWrite(LEFT_LED_PIN, leftLedOn);
        digitalWrite(RIGHT_LED_PIN, rightLedOn);
    }

    // Small delay to prevent rapid state changes
    delay(50);
}