# Ultrasonic Distance Meter with Visual Alert using Arduino

This project implements a real-time distance monitoring and alert system using an Arduino UNO and an HC-SR04 ultrasonic sensor. The system measures the distance to nearby objects and provides visual feedback using an LCD display and an LED indicator. It is designed for applications such as obstacle detection, parking assistance, and safety warnings in robotics.

---

🧰 **Hardware Components**

**Ultrasonic Sensor (HC-SR04):**  
Measures distance to nearby objects using ultrasonic sound pulses.

**LCD Display (16x2):**  
Displays the measured distance in both centimeters and inches.

**LED Indicator:**  
Lights up to indicate that an object is within a danger range (less than 10 cm).

**Arduino UNO:**  
Serves as the central microcontroller, handling sensor readings, display output, and LED control.

---

🧱 **Software Architecture**

**Arduino Firmware**  
- Initializes the HC-SR04 ultrasonic sensor, 16x2 LCD, and LED.  
- Continuously measures distance and converts it to centimeters and inches.  
- Displays the distance on the LCD in real time.  
- Activates the LED if the measured distance falls below 10 cm.  
- Operates entirely on the Arduino UNO with no external processing.


---

🚦 **Implementation**

**Arduino Code**  
- Uses the NewPing library (or standard pulseIn functions) to read distance.  
- Displays real-time values to the LCD every 200 milliseconds.  
- Lights the LED when distance < 10 cm.  
- Simple and reliable logic implemented in embedded C using the Arduino IDE.

---

---

📦 **Requirements**

- Arduino IDE (tested with version 1.8.x or 2.x)
- HC-SR04 Ultrasonic Sensor
- 16x2 LCD Display (with I2C module recommended)
- LED and resistor
- Breadboard and jumper wires


---

📜 **License**

This project is open-source and available under the MIT License.
