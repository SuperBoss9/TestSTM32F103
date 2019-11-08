#include <Arduino.h>
#include <stm32f1xx_ll_adc.h>
#include <Wire.h>

#define LED_internal PC12 // KDS2 Controller Build In LED
#define UART_RX PA10
#define UART_TX PA9

// PWM Settings - ШИМ нужно подавать на ON_A1, а разрешение на PWM_A1
#define ON_A1 PC6
#define ON_A2 PC7
#define PWM_A1 PA15
#define PWM_A2 PB3

#define ON_B1 PC8
#define ON_B2 PC9
#define PWM_B1 PB10
#define PWM_B2 PB11

#define Zummer PC3
#define VREFINT 1200
#define ADC_RANGE 4096

void ScanI2C();
void I2CRead_Write();
byte EEPROM_ReadByte(int dev, byte Address);

TIM_TypeDef *Instance = TIM4; 
HardwareTimer *MyTim = new HardwareTimer(Instance);

void Update_IT_callback(HardwareTimer *) // Internal blinker
{
  digitalWrite(LED_internal, !digitalRead(LED_internal));
}
void setup()
{
  // put your setup code here, to run once:
  Serial.setRx(UART_RX);
  Serial.setTx(UART_TX);
  Serial.begin(115200);

  pinMode(LED_internal, OUTPUT);           // Build-in LED
  MyTim->setMode(4, TIMER_OUTPUT_COMPARE); 
  MyTim->setOverflow(1, HERTZ_FORMAT);     // 1 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();

  // Init Read& Write
  analogWriteResolution(12);
  analogWriteFrequency(10000);
  analogReadResolution(10); // 0-1023
  
  // Set up PWM pins
  pinMode(ON_A1, OUTPUT);
  pinMode(ON_A2, OUTPUT);
  pinMode(PWM_A1, OUTPUT);
  pinMode(PWM_A2, OUTPUT);

  pinMode(ON_B1, OUTPUT);
  pinMode(ON_B2, OUTPUT);
  pinMode(PWM_B1, OUTPUT);
  pinMode(PWM_B2, OUTPUT);
  
  Serial.println("\n\rHello the new day!");

  // PWM HardCore
  Serial.println("Drop voltage to Zero at A & B");
  digitalWrite(ON_A1, LOW);
  digitalWrite(ON_A2, LOW);
  digitalWrite(PWM_A1, LOW);
  digitalWrite(PWM_A2, LOW);
  delay(10000);
  
  int iPWM=0;
  Serial.println("A-Step 100");
  digitalWrite(PWM_A1,HIGH);
  while(iPWM<4096){
    analogWrite(ON_A1,iPWM);
    Serial.println("  "+(String)iPWM);
    delay(1500);
    iPWM+=100;
  }
  Serial.println("Back to 0");
  digitalWrite(PWM_A1,LOW);
  analogWrite(ON_A1,0);

  Serial.println("B-Step 100");
  iPWM=0;
  digitalWrite(PWM_B2,HIGH);
  while(iPWM<4096){
    analogWrite(ON_B2,iPWM);
    Serial.println("  "+(String)iPWM);
    delay(1500);
    iPWM+=100;
  }
  Serial.println("Back to 0");
  digitalWrite(PWM_B1,LOW);
  analogWrite(ON_B1,0);  
  //End of PWM

  // I2C EEPROM
  ScanI2C();
  I2CRead_Write();
  // End I2C
  tone(Zummer, 400, 10);
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.print(".");
}
void I2CRead_Write()
{
  Wire.requestFrom(2, 6); // request 6 bytes from slave device #2

  while (Wire.available()) // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);      // print the character
  }
  Serial.println("\n\rRead Done.");
  Wire.beginTransmission(2); // transmit to device #4
  Wire.write("123456");      // sends five bytes
  Wire.endTransmission();    // stop transmitting
  Serial.println("\n\rWrite Done.");
  // Once More Write & Read
  for (int i = 0; i < 512; i++) // Wrtie
  {                             //У 24C04 512 байт памяти
    //EEPROM_WriteByte(DEVICE_1, i, 0xAB);
    Wire.beginTransmission(0x50);
    Wire.write(i);
    //Wire.write(0xAB);
    Wire.write(51);
    delay(5); //Не знаю точно, но в Datasheet описана задержка записи в 5мс, поправьте меня, если я не прав.
    Wire.endTransmission();
  }
  Serial.println("AltWriteEnd");
  int val = 0;
  for (int i = 0; i < 512; i++)
  {
    Serial.print(EEPROM_ReadByte(0x50, i), HEX);
    Serial.print(" ");
    val++;
    if (val >= 8)
    {
      val = 0;
      Serial.println();
    } //Для удобства, в строчку по 8 байт
  }
  Serial.println("AltReadEnd");
}
byte EEPROM_ReadByte(int dev, byte Address)
{
  byte rdata = 0xFF;
  Wire.beginTransmission(dev);
  Wire.write(Address);
  Wire.endTransmission();
  Wire.requestFrom(dev, 1);
  if (Wire.available())
    rdata = Wire.read();
  return rdata;
}
void ScanI2C()
{
  byte error, address;
  int nDevices;
  // I2C
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
}