//Name: Jairo Cadena-Mendez
//Purpose: CPE301 Final - Telemtry System
//Date: 7/11/23

//LCD
#include <LiquidCrystal.h>
#include <Wire.h>
             

//GPIO
volatile unsigned char* port_F = (unsigned char*) 0x31; 
volatile unsigned char* ddr_F  = (unsigned char*) 0x30; 
volatile unsigned char* pin_F  = (unsigned char*) 0x2F; 

volatile unsigned char* port_G = (unsigned char*)0x34;
volatile unsigned char* ddr_G = (unsigned char*)0x33;

volatile unsigned char* port_E = (unsigned char*)0x2E;
volatile unsigned char* ddr_E = (unsigned char*)0x2D;
volatile unsigned char* pin_E = (unsigned char*)0x2C;

volatile unsigned char* port_H = (unsigned char*)0x102;
volatile unsigned char* ddr_H = (unsigned char*)0x101;
volatile unsigned char* pin_H = (unsigned char*)0x100;

volatile unsigned char* port_K = (unsigned char*) 0x108; 
volatile unsigned char* ddr_K  = (unsigned char*) 0x107;


//Analog
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//timer
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;


//UART
#define RDA 0x80
#define TBE 0x20 
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;


unsigned int currentTicks = 65535;
unsigned char timer_running = 0;

const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int gyroX, gyroY, gyroZ;
long gyroXcalc, gyroYcalc, gyroZcalc;
boolean gyroFlag;
long accX, accY, accZ, accTotalVector;
float angleRollAcc, anglePitchAcc;
float pitch, roll;
float pitchOutput, rollOutput;

long loop_timer;
int temp;

int lastPin1State, lastPin2State;

void setup() {
    U0init(9600);
    lcd.begin(20, 4);
    adc_init();
    setup_mpu_6050_registers();

    *ddr_E |= (1 << 3); // Set bit 3 of Port E as output (speedPin)
    *ddr_G |= (1 << 5); // Set bit 5 of Port G as output (dir1Pin)
    *ddr_E |= (1 << 5); // Set bit 5 of Port E as output (dir2Pin)

    //buttons input
    *ddr_E &= ~(0x01 << 4);
    *ddr_H &= ~(0x01 << 3);
  
    //LED output
    *ddr_K |= (0x01 << 7);
    *ddr_K |= (0x01 << 6);
    *ddr_K |= (0x01 << 5);
    *ddr_K |= (0x01 << 4);
  
     //button pull-ups
    *port_E |= (0x01 << 4);
    *port_H |= (0x01 << 3);
  
    *port_K |= (0x01 << 4); //red LED on

     //interuope
    *ddr_K |= (1 << 2);
    *port_K &= ~(1 << 2);

  //pin A1 in Analog output(GPIO) PF1
    *ddr_F |= (0x01 << 1);
    for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Read the raw acc and gyro data from the 
      read_mpu_6050_data();                                             
      gyroXcalc += gyroX;                                              
      gyroYcalc += gyroY;                                              
      gyroZcalc += gyroZ;       
      my_delay(300);                                                                                                
    }

  // divide by 1000 to get avarage offset
    gyroXcalc /= 1000;                                                 
    gyroYcalc /= 1000;                                                 
    gyroZcalc /= 1000;                                                 
    loop_timer = micros(); 
}

void loop() {
  // read the pushbutton input pin:
  int pin1State = (*pin_E & (0x01 << 4));
  int pin2State = (*pin_H & (0x01 << 3));

  if (pin1State == 0 && lastPin1State != 0 && pin2State != 0)
  {
    *port_K |= (0x01 << 6); // Blue the LED on
    
    do{ 
      *port_K |= (0x01 << 7); // Turn on the LED (PB7)
      *port_K &= ~(0x01 << 4); //red LED off
  
        read_mpu_6050_data();
        unsigned int sensorValue = adc_read(0);   
        double voltage = sensorValue * (5.0 / 1023.0);
    
        char charValue1 = (((temp/ 340.0 + 36.53) / 10) + '0'); // Get the first digit
        char charValue2 = (((int)(temp/ 340.0 + 36.53) % 10) + '0'); //get the Second digit
    
        //prints temps of the acclerometer to serial motitor
        U0putchar('T');U0putchar('e');U0putchar('m');U0putchar('p'); U0putchar(':'); U0putchar(' '); U0putchar(charValue1); U0putchar(charValue2); U0putchar('\n');
        
        
        gyroX -= gyroXcalc;                                                
        gyroY -= gyroYcalc;                                                
        gyroZ -= gyroZcalc;                                                
                  
        pitch += gyroX * 0.0000611;                                  
    
        roll += gyroY * 0.0000611;                   
     
        pitch += roll * sin(gyroZ * 0.000001066);              
        roll -= pitch * sin(gyroZ * 0.000001066);               

        //Accelerometer angle calculations
        accTotalVector = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));  
     
        anglePitchAcc = asin((float)accY/accTotalVector)* 57.296;       //Calculate the pitch angle
     
        angleRollAcc = asin((float)accX/accTotalVector)* -57.296;       //Calculate the roll angle
      
        anglePitchAcc -= 0.00;                                              //Accelerometer calibration value for pitch
        angleRollAcc -= 0.0;                                               //Accelerometer calibration value for roll
    
        if(gyroFlag){                                                 
          pitch = pitch * 0.9996 + anglePitchAcc * 0.0004;     
      
          roll = roll * 0.9996 + angleRollAcc * 0.0004;     
        }
        else
        {                                                            //At first start
          pitch = anglePitchAcc;                 //Set the gyro pitch angle equal to the accelerometer pitch angle 
          roll = angleRollAcc;                     //Set the gyro roll angle equal to the accelerometer roll angle 
          gyroFlag = true;                              //Set the IMU started flag
        }
      
        pitchOutput = pitchOutput * 0.9 + pitch * 0.1;   
        rollOutput = rollOutput * 0.9 + roll * 0.1;
        
    
        // set the cursor to column 0, line 1
        // (note: line 1 is the second row, since counting begins with 0):
        *pin_F = voltage;
        lcd.setCursor(0, 0);
        //Pitch
        lcd.print("P:");
        lcd.print(pitchOutput);
        lcd.setCursor(8, 0);
    
        //roll
        lcd.print("R:");
        lcd.print(rollOutput);
        
        lcd.setCursor(0, 1);
        lcd.print("ADC Voltage: ");
        lcd.print(voltage);
        lcd.print(" V");
    
    
        //temp check so if higher than 29 then it will turn on "fan" (motor)
        if((int)(temp/ 340.0 + 36.53) > 29)
        {
          *port_G |= (1 << 5);  // dir1Pin to HIGH
          *port_E &= ~(1 << 5); // dir2Pin to LOW
    
          *port_E |= (1 << 3);
    
          *port_K |= (0x01 << 5); // Yellow the LED on
    
        }
        else //turns off when its below 29
        {
          *port_G &= ~(1 << 5);  // dir1Pin to low
          *port_E |= (1 << 5); // dir2Pin to high
    
          // Set motor speed using PWM 
          *port_E &= ~(1 << 3);
    
          *port_K &= ~(0x01 << 5); // Yellow the LED off
        }
    }while(*pin_H & (0x01 << 3));
  }
  else if (pin2State == 0 && lastPin2State != 0 && pin1State != 0)
  {
    *port_K &= ~(0x01 << 7); // Turn off the LED (PB7)
    *port_K |= (0x01 << 4); //red LED on
    *port_K &= ~(0x01 << 6); //Blue light off
  }

  lastPin1State = pin1State;
  lastPin2State = pin2State;

}



//UART
void U0init(unsigned long U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE) == 0);
  *myUDR0 = U0pdata;
}


//timer
void my_delay(unsigned int freq)
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1A = 0x0;
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   
  *myTIFR1 |= 0x01;
}


//Analog
void adc_init()
{
  *my_ADCSRA |= 0b10000000; 
  *my_ADCSRA &= 0b11011111; 
  *my_ADCSRA &= 0b11110111; 
  *my_ADCSRA &= 0b11111000;

  *my_ADCSRB &= 0b11110111; 
  *my_ADCSRB &= 0b11111000; 

  *my_ADMUX  &= 0b01111111; 
  *my_ADMUX  |= 0b01000000; 
  *my_ADMUX  &= 0b11011111; 
  *my_ADMUX  &= 0b11100000; 
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  if(adc_channel_num > 7)
  {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect)
{
  // Stop the Timer
  *myTCCR1B &= 0xF8;
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  // Start the Timer
  *myTCCR1B |= 0x01;
  // if it's not the STOP amount
  if(currentTicks != 65535)
  {
    // XOR to toggle PB6
    *port_K ^= (1 << 2);
  }
}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                         //Send the requested starting register
  Wire.write(0x08);                                                          //Set the requested starting register
  Wire.endTransmission();                                             
}


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                           //Send the requested starting register
  Wire.endTransmission();                                                 //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                          //Wait until all the bytes are received
  accX = Wire.read()<<8|Wire.read();                                  
  accY = Wire.read()<<8|Wire.read();                                  
  accZ = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyroX = Wire.read()<<8|Wire.read();                                 
  gyroY = Wire.read()<<8|Wire.read();                                 
  gyroZ = Wire.read()<<8|Wire.read();                                
}
