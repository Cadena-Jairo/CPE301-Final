# CPE301-Final

For my final project I decided to make a mini telemtry system that will measure roll, pitch, and tempeture of the board. I used GY-521 accelerometer to be able to measure the gyro and acceleration of the board and using https://forum.arduino.cc/t/mpu-6050-gy-521-mega-2560/549116 forms I was able to get the calculations to be able to measure the roll and pitch for the baord. My inital design was to to display only the gyro and accelerometer of the board to the serial monitor, but since we weren't allowed to use that library we could only print char values to the serial monitor, so I decied to get the pitch and poll and display that on the LCD and only print the temps in the serial monitor since its only 2 digits. Doing so I added teo buttons that will be the "start" and the "stop" button. When the arduino and the rest of the circuit gets power a Red LED will be on telling us that the system is off, I also added a buzzer that will buzz for a couple seconds and cut off with a TimerInterruptor and that buzzer will tell us when its ready to press start. When pressing start the green and blue LED will go on. The Green LED tells us that its on and the Blue LED will tell us that the accelerometer is on. The Yellow LED is for when the motor is on and the motor will turn on when the temps of the GY-521 build is above 29 degrees. Once its 29 degrees or below the yellow LED and the motor will turn off. These are what I used GPIO, ADC, Timers, UART, and ISR.

- GPIO - Everything that needed a pinmode or anaolog in/out
- ADC - Anolog to control the brightness of the LCD using a potentiometer
- Timers - Used for delays in the setup
- UART - used for serial.begin and serial.print
- ISR - Used for buzzer to buzz for a couple seconds before start
