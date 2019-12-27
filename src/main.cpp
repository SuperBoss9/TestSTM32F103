#include <Arduino.h>
#include <stm32f1xx_ll_adc.h>
#include <Wire.h>
#include <analog.h>

#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 5
#define BCOEFFICIENT 3988
#define SERIESRESISTOR 10000


#define LED_internal PB3 // KDS2 Controller Build In LED
#define UART_RX PA10
#define UART_TX PA9

// PWM Settings - ШИМ нужно подавать на ON_A1, а разрешение на PWM_A1

#define VREFINT 1200
#define ADC_RANGE 4096

#define Temperature_channel_1 PA6
#define Temperature_channel_2 PA7

#define I2C_SCL PB8
#define I2C_SDA PB9

#define LED_channel_1_anode PB6  // +
#define LED_channel_1_catode PB7 // -

#define LED_channel_2_anode PB4  // +
#define LED_channel_2_catode PB5 // -

//Buttons
#define Button_1 PC6
#define Button_2 PB13

#define Encoder_1_A PC8
#define Encoder_1_B PC7

#define Encoder_2_A PB15
#define Encoder_2_B PB14

// Power
#define Power_1_ShortCircuit PC0
#define Power_2_ShortCircuit PB10

#define Power_1_PositivePolarity PC1
#define Power_1_NegativePolarity PC2
#define Power_2_PositivePolarity PB2
#define Power_2_NegativePolarity PC5

#define Power_1_SetVoltage PA4
#define Power_2_SetVoltage PA5

#define Power_1_ATAC PC4
#define Power_2_ATAC PC3

#define Power_1_Current PA2
#define Power_2_Current PA3

#define Power_1_Voltage PA1
#define Power_2_Voltage PA0

#define LightMeter_1 PB0
#define LightMeter_2 PB1


void ScanI2C();
void I2CRead_Write();
byte EEPROM_ReadByte(int dev, byte Address);

TIM_TypeDef *Instance = TIM4;
HardwareTimer *MyTim = new HardwareTimer(Instance);

void Update_IT_callback(HardwareTimer *) // Internal blinker
{
  digitalWrite(LED_internal, !digitalRead(LED_internal));
}
float thermistorRead(int iPin)
{
  int samples[NUMSAMPLES];
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++)
  {
    samples[i] = analogRead(iPin);
    delay(10);
  }
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++)
  {
    average += samples[i];
  }
  average /= NUMSAMPLES;
  //Serial.println("Average analog reading: " + (String)average);

  // convert the value to resistance
  average = 4095 / average - 1; // Don't forget to use correct resolution
  average = SERIESRESISTOR / average;

  //Serial.println("Thermistor resistance " + (String)average);

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;          // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C

  //Serial.println("Temperature " + (String)steinhart + " *C");
  return steinhart;
}
float voltageRead(int iPin)
{
  int samples[NUMSAMPLES];
  uint8_t i;
  float average;
  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++)
  {
    samples[i] = analogRead(iPin);
    delay(10);
  }
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++)
  {
    average += samples[i];
  }
  average /= NUMSAMPLES;
  Serial.println("Average analog reading: " + (String)average);

  average = average * 4095 / 3300; // Don't forget to change the numbers

  Serial.println("Measured voltage is: " + (String)average);

  return average;
}
void buttons()
{
  pinMode(Button_1, INPUT);
  bool bButton1Pressed = digitalRead(Button_1);
  while (bButton1Pressed == digitalRead(Button_1))
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("Button 1 pressed");
  delay(1000);
  pinMode(Button_2, INPUT);
  bool bButton2Pressed = digitalRead(Button_2);
  while (bButton2Pressed == digitalRead(Button_2))
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("Button 2 pressed");
  delay(1000);  
}
void encoders(){
  pinMode(Encoder_1_A, INPUT);
  pinMode(Encoder_1_B, INPUT);
  bool bENC1APressed = digitalRead(Encoder_1_A);
  bool bENC1BPressed = digitalRead(Encoder_1_B);
  Serial.println("1-Enc A cond: " + (String)digitalRead(Encoder_1_A));
  Serial.println("1-Enc B cond: " + (String)digitalRead(Encoder_1_B));
  Serial.println("Rotate ENC 1");
  while (bENC1APressed == digitalRead(Encoder_1_A))
  {
    delay(1);
  }
  Serial.println("Enc 1 A Pressed");
  Serial.println("Rotate Enc 1 More");
  while (bENC1BPressed == digitalRead(Encoder_1_B))
  {
    delay(1);
  }
  Serial.println("Enc 1 B Pressed");
  // ---------------------
  pinMode(Encoder_2_A, INPUT);
  pinMode(Encoder_2_B, INPUT);
  bool bENC2APressed = digitalRead(Encoder_2_A);
  bool bENC2BPressed = digitalRead(Encoder_2_B);
  Serial.println("2-Button A cond: " + (String)digitalRead(Encoder_2_A));
  Serial.println("2-Button B cond: " + (String)digitalRead(Encoder_2_B));
  Serial.println("Rotate ENC 2");
  while (bENC2APressed == digitalRead(Encoder_2_A))
  {
    delay(1);
  }
  Serial.println("Enc 2 A Pressed");
  Serial.println("Rotate Enc 2 More");
  while (bENC2BPressed == digitalRead(Encoder_2_B))
  {
    delay(1);
  }
  Serial.println("Enc 2 B Pressed");
}
void LEDs(){
  // External LEDs
  // Ch #1
  pinMode(LED_channel_1_anode, OUTPUT);
  pinMode(LED_channel_1_catode, OUTPUT);
  digitalWrite(LED_channel_1_anode, LOW);
  digitalWrite(LED_channel_1_catode, LOW);
  Serial.println("Ch #1 LED OFF");
  HAL_Delay(1000);
  digitalWrite(LED_channel_1_anode, LOW);
  digitalWrite(LED_channel_1_catode, HIGH);
  Serial.println("Ch #1 LED RED");
  HAL_Delay(1000);
  digitalWrite(LED_channel_1_catode, LOW);
  digitalWrite(LED_channel_1_anode, HIGH);
  Serial.println("Ch #1 LED Green");
  HAL_Delay(1000);
  digitalWrite(LED_channel_1_catode, LOW);
  digitalWrite(LED_channel_1_anode, LOW);

  // Ch #2
  pinMode(LED_channel_2_anode, OUTPUT);
  pinMode(LED_channel_2_catode, OUTPUT);
  digitalWrite(LED_channel_2_anode, LOW);
  digitalWrite(LED_channel_2_catode, LOW);
  Serial.println("Ch #2 LED OFF");
  HAL_Delay(1000);
  digitalWrite(LED_channel_2_anode, LOW);
  digitalWrite(LED_channel_2_catode, HIGH);
  Serial.println("Ch #2 LED RED");
  HAL_Delay(1000);
  digitalWrite(LED_channel_2_catode, LOW);
  digitalWrite(LED_channel_2_anode, HIGH);
  Serial.println("Ch #2 LED Green");
  HAL_Delay(1000);
  digitalWrite(LED_channel_2_catode, LOW);
  digitalWrite(LED_channel_2_anode, LOW);
}
void LightMeters(){
  // Light Meters
  pinMode(LightMeter_1,INPUT);
  pinMode(LightMeter_2,INPUT);
  Serial.println("Light meters "+(String)analogRead(LightMeter_1)+":"+(String)analogRead(LightMeter_2));
}
void setup()
{
  // put your setup code here, to run once:
  Serial.setRx(UART_RX);
  Serial.setTx(UART_TX);
  Serial.begin(115200);

  pinMode(LED_internal, OUTPUT); // Build-in LED
  MyTim->setMode(4, TIMER_OUTPUT_COMPARE);
  MyTim->setOverflow(1, HERTZ_FORMAT); // 1 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();

  // Init Read& Write
  analogWriteResolution(12);
  analogWriteFrequency(10000);
  analogReadResolution(12); // 0-4095

  Serial.println("\r\nShow must go on!\r\n");

  // I2C EEPROM
      //ScanI2C(); // Какая-то пробелма с инициализацией, если ставить в конец, то не инициализируется Wire по всей видимости
      //I2CRead_Write();
  // End I2C
  pinMode(Temperature_channel_1,INPUT_ANALOG);
  pinMode(Temperature_channel_2,INPUT_ANALOG);
  Serial.println("Temp: "+(String)analogRead(Temperature_channel_1)+":"+(String)analogRead(Temperature_channel_2));
  //POWER
    // Init
  pinMode(Power_1_ShortCircuit, OUTPUT);
  pinMode(Power_2_ShortCircuit, OUTPUT);
  pinMode(Power_1_PositivePolarity,OUTPUT);
  pinMode(Power_1_NegativePolarity,OUTPUT);
  pinMode(Power_2_PositivePolarity,OUTPUT);
  pinMode(Power_2_NegativePolarity,OUTPUT);
  pinMode(Power_1_SetVoltage,OUTPUT);
  pinMode(Power_2_SetVoltage,OUTPUT);
  pinMode(Power_1_ATAC,INPUT);
  pinMode(Power_2_ATAC,INPUT);
  pinMode(Power_1_Current,INPUT);
  pinMode(Power_2_Current,INPUT);  

      // ShortCircuit ON
  Serial.println("Short Circuit ON");
  digitalWrite(Power_1_ShortCircuit, LOW);
  digitalWrite(Power_2_ShortCircuit, LOW);
  delay(1000);
  Serial.println("ATAC: "+(String)digitalRead(Power_1_ATAC)+":"+(String)digitalRead(Power_2_ATAC));
  Serial.println("Voltage: " + (String)(0.000805861*analogRead(Power_1_Voltage)*1.98)+":"+ (String)(0.000805861*analogRead(Power_2_Voltage)*1.98));
  Serial.println("Current: " + (String)(0.000805861*analogRead(Power_1_Current)*0.375*7.1048)+":"+ (String)(0.000805861*analogRead(Power_2_Current)*0.375*7.2134));
  Serial.println("ADC: "+(String)analogRead(Temperature_channel_1)+":"+(String)analogRead(Temperature_channel_2));
    
  Serial.println("Short Circuit OFF");
  digitalWrite(Power_1_ShortCircuit, HIGH);
  digitalWrite(Power_2_ShortCircuit, HIGH);
  delay(1000);
  Serial.println("ATAC: "+(String)digitalRead(Power_1_ATAC)+":"+(String)digitalRead(Power_2_ATAC));
  Serial.println("Voltage: " + (String)(0.000805861*analogRead(Power_1_Voltage)*1.98)+":"+ (String)(0.000805861*analogRead(Power_2_Voltage)*1.98));
  Serial.println("Current: " + (String)(0.000805861*analogRead(Power_1_Current)*0.375*7.1048)+":"+ (String)(0.000805861*analogRead(Power_2_Current)*0.375*7.2134)+"\r\n");
  
  Serial.println("Positive polarity ON");
  digitalWrite(Power_1_PositivePolarity, HIGH);
  digitalWrite(Power_1_NegativePolarity, LOW);
  digitalWrite(Power_2_PositivePolarity, HIGH);
  digitalWrite(Power_2_NegativePolarity, LOW);
  delay(1000);
  Serial.println("ATAC: "+(String)digitalRead(Power_1_ATAC)+":"+(String)digitalRead(Power_2_ATAC));
  Serial.println("Voltage: " + (String)(0.000805861*analogRead(Power_1_Voltage)*1.98)+":"+ (String)(0.000805861*analogRead(Power_2_Voltage)*1.98));
  Serial.println("Current: " + (String)(0.000805861*analogRead(Power_1_Current)*0.375*7.1048)+":"+ (String)(0.000805861*analogRead(Power_2_Current)*0.375*7.2134)+"\r\n");
  
  Serial.println("Negative polarity ON");
  digitalWrite(Power_1_PositivePolarity, LOW);
  digitalWrite(Power_1_NegativePolarity, HIGH);
  digitalWrite(Power_2_PositivePolarity, LOW);
  digitalWrite(Power_2_NegativePolarity, HIGH);
  delay(1000);
  Serial.println("ATAC: "+(String)digitalRead(Power_1_ATAC)+":"+(String)digitalRead(Power_2_ATAC));
  Serial.println("Voltage: " + (String)(0.000805861*analogRead(Power_1_Voltage)*1.98)+":"+ (String)(0.000805861*analogRead(Power_2_Voltage)*1.98));
  Serial.println("Current: " + (String)(0.000805861*analogRead(Power_1_Current)*0.375*7.1048)+":"+ (String)(0.000805861*analogRead(Power_2_Current)*0.375*7.2134)+"\r\n");
  
  Serial.println("Set Voltage ###");
  //analogWrite(Power_1_SetVoltage, 2500);
  analogWrite(Power_1_SetVoltage, 2500);
  //pwm_start(PA_4,1000,500);
  //dac_write_value(PA_4,500,1);
  //analogWrite(Power_1_SetVoltage, 595);
  analogWrite(Power_2_SetVoltage, 4095);
  delay(1000);
  Serial.println("ATAC: "+(String)digitalRead(Power_1_ATAC)+":"+(String)digitalRead(Power_2_ATAC));
  Serial.println("Voltage: " + (String)(0.000805861*analogRead(Power_1_Voltage)*1.98)+":"+ (String)(0.000805861*analogRead(Power_2_Voltage)*1.98));
  Serial.println("Current: " + (String)(0.000805861*analogRead(Power_1_Current)*0.375*7.1048)+":"+ (String)(0.000805861*analogRead(Power_2_Current)*0.375*7.2134)+"\r\n");
  
  analogWrite(Power_1_SetVoltage, 0);
  analogWrite(Power_2_SetVoltage, 0);
  Serial.println("Set Voltage 0");
  delay(1000);
  Serial.println("ATAC: "+(String)digitalRead(Power_1_ATAC)+":"+(String)digitalRead(Power_2_ATAC));
  Serial.println("Voltage: " + (String)(0.000805861*analogRead(Power_1_Voltage)*1.98)+":"+ (String)(0.000805861*analogRead(Power_2_Voltage)*1.98));
  Serial.println("Current: " + (String)(0.000805861*analogRead(Power_1_Current)*0.375*7.1048)+":"+ (String)(0.000805861*analogRead(Power_2_Current)*0.375*7.2134)+"\r\n");
    

  
        //buttons(); // Check Buttons
        //encoders(); // Check Encoders
        //LEDs();
        //LightMeters();

  

  Serial.println("\n\rEverything done");

  
  
  /*tone(Zummer, 400, 10);*/
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(1000);
  //Serial.print(".");
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
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  //Wire.setSCL(62);
  //Wire.setSDA(61);
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