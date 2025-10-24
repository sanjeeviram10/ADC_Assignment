#define RX_PIN 16       
#define BAUD_RATE 115200
#define LED_PIN 2       

void setup() {
  pinMode(LED_PIN, OUTPUT);      // Initialize LED
  Serial.begin(BAUD_RATE);       // Debug serial
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, -1); // UART2 RX only
  Serial.println("ESP32 LM35 Receiver Started");
}

void loop() {
  static String inputString = "";  // Store characters from UART

  while (Serial2.available()) {
    char c = Serial2.read();      // Read one character

    if (c == '\n') {              // End of one ADC value
      int adcValue = inputString.toInt();            // Convert string to integer
      float voltage = (adcValue / 4095.0) * 3.3;    // ADC → Voltage
      float temperature = voltage * 100.0;          // LM35 conversion

      Serial.print("ADC: "); Serial.print(adcValue);
      Serial.print(" | Temp: "); Serial.print(temperature); Serial.println(" °C");

      // Simple 2-case LED control
      if (temperature >= 40.0) {   // Hot temperature
        digitalWrite(LED_PIN, HIGH);  
        delay(300);                 
        digitalWrite(LED_PIN, LOW); 
        delay(300);
      } else {                     // Normal temperature
        digitalWrite(LED_PIN, LOW);
      }

      inputString = ""; // Clear string for next reading
    }
    else if (isDigit(c)) {
      inputString += c;  // Add digit to string
    }
  }
}
