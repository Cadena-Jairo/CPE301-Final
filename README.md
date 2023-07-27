# CPE301-Final

For my final project, I decided to make a mini telemetry system that will measure roll, pitch, and temperature of the board. I used the GY-521 accelerometer to be able to measure the gyro and acceleration of the board, and using https://forum.arduino.cc/t/mpu-6050-gy-521-mega-2560/549116 forms, I was able to get the calculations to measure the roll and pitch for the board. My initial design was to display only the gyro and accelerometer of the board to the serial monitor, but since we weren't allowed to use that library, we could only print char values to the serial monitor. So, I decided to get the pitch and roll and display that on the LCD and only print the temperatures in the serial monitor since it's only 2 digits. Doing so, I added two buttons that will be the "start" and the "stop" button. When the Arduino and the rest of the circuit get power, a Red LED will be on, telling us that the system is on. I also added a buzzer that will buzz for a couple of seconds and cut off with a TimerInterruptor, and that buzzer will tell us when it's ready to press start. When pressing start, the green and blue LEDs will go on. The Green LED tells us that it's on, and the Blue LED will tell us that the accelerometer is on. The Yellow LED is for when the motor is on, and the motor will turn on when the temperatures of the GY-521 build are above 29 degrees. Once it's 29 degrees or below, the yellow LED and the motor will turn off. These are what I used GPIO, ADC, Timers, UART, and ISR for.

GPIO - Everything that needed a pin mode or analog in/out

ADC - Analog to control the brightness of the LCD using a potentiometer

Timers - Used for delays in the setup

UART - Used for Serial.begin and Serial.print

ISR - Used for the buzzer to buzz for a couple of seconds before start
