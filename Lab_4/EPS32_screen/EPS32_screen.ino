/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-oled
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels

// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

String receivedMessage = "";

void setup() {
  Serial.begin(9600);

  // initialize OLED display with I2C address 0x3C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1);
  }

  delay(2000);         // wait two seconds for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(2);         // set text size
  oled.setTextColor(WHITE);    // set text color
  oled.setCursor(0, 10);       // set position to display
  oled.println("    Hi   "); // set text
  oled.println("  welcome ");
  oled.display();              // display on OLED
}

void loop() {

  if (Serial.available() >= 2) {
    // Read two bytes
    uint8_t highByte = Serial.read();
    uint8_t lowByte = Serial.read();

    // Combine into a 16-bit number
    uint16_t value = (highByte << 8) | lowByte;

    // Convert to string
    receivedMessage = String(value);
    if (receivedMessage == "65535"){
      // Print to Serial Monitor
        Serial.print("Button Pressed");
      

      oled.clearDisplay();
      oled.setTextSize(2);
      oled.setTextColor(WHITE);
      oled.setCursor(10, 10);
      oled.println("Button");
      oled.setCursor(10, 30);
      oled.println("Pressed");
      oled.display();
    }
    else if (receivedMessage == "61166"){
      // Print to Serial Monitor
        Serial.print("Button Pressed");
      

      oled.clearDisplay();
      oled.setTextSize(2);
      oled.setTextColor(WHITE);
      oled.setCursor(0, 30);
      oled.println("Processing");
      oled.display();
    }
    else{
        // Print to Serial Monitor
        Serial.print("Received 16-bit value: ");
        Serial.println(receivedMessage);

        // Display on OLED
        oled.clearDisplay();
        oled.setTextSize(2);
        oled.setTextColor(WHITE);
        oled.setCursor(0, 10);
        oled.println("New Rnd No");
        oled.setCursor(20, 40);
        oled.println(receivedMessage);
        oled.display();
    }

  }
}

