//this is a copy and paste job made by F2k
#include <Servo.h>
#include "se8r01.h"
byte gtemp[5];
byte k=0;
//***************************************************
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  17  // 32 unsigned chars TX payload

Servo SteeringServo;  //Create servo object representing SteeringServo

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0xD0,0xD0,0xD0,0xD0,0xD0
}; // Define a static TX address

unsigned char rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value

uint8_t steering, gas, tombol_start, tombol_select, tombol_l1, tombol_l2, tombol_r1, tombol_r2, tombol_s, tombol_b, tombol_x, tombol_k, tombol_up, tombol_down, tombol_left, tombol_right;

int GasSetting = 0; // Setting stop default
int GasSpeed = 60; // Setting speed default
int PinGas = 6;
int PinGasb = 7;

int StrServoSetting = 90; //Setting for the Steering Servo
int BelokKiri = 25; // 0 derajat
int BelokKanan = 160; // 180 derajat
int PinServo = 8;

int PinLampu = 2;
int PinSr = 3;
int PinSl = 4;
int PinSx = 5;

int ledState = LOW;
int ledStati = HIGH;
unsigned long previousMillis = 0;
const long interval = 400;

//***************************************************
void setup() 
{
  pinMode(CEq,  OUTPUT);
  pinMode(SCKq, OUTPUT);
  pinMode(CSNq, OUTPUT);
  pinMode(MOSIq,  OUTPUT);
  pinMode(MISOq, INPUT);
  //pinMode(IRQq, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  init_io();                        // Initialize IO port
  unsigned char status=SPI_Read(STATUS);

  SteeringServo.attach(PinServo);// attaches the Steering Servo to pin 2
  SteeringServo.write(StrServoSetting);
  delay(50);

  pinMode(PinGas,OUTPUT);
  pinMode(PinLampu,OUTPUT);
  pinMode(PinSl,OUTPUT);
  pinMode(PinSr,OUTPUT);
  pinMode(PinSx,OUTPUT);
  
  Serial.print("Status = ");    
  Serial.println(status,HEX);     
  Serial.println("Radio starting...");

  digitalWrite(CEq, 0);
  delay(1);
  se8r01_powerup();
  se8r01_calibration();
  se8r01_setup();
  radio_settings();
  
  //rx mode
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_CONFIG, 0x3f); 
  Serial.println("RX mode..."); 
  
  digitalWrite(CEq, 1);
}

void loop() 
{

 // if(digitalRead(IRQq)==LOW)
  //{
    delay(1);      //read reg too close after irq low not good
    unsigned char status = SPI_Read(STATUS);
    unsigned long currentMillis = millis();  
  
    if(status&STA_MARK_RX)                                                 // if receive data ready (TX_DS) interrupt
    {
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);             // read playload to rx_buf
      SPI_RW_Reg(FLUSH_RX,0); // clear RX_FIFO
     //Serial.print("rx_buf[i]");      
      //for(byte i=0; i<TX_PLOAD_WIDTH; i++)
      //{
      //    Serial.print(" : ");
      //    Serial.print(rx_buf[i]);                              // print rx_buf
      //}

      gas = rx_buf[1];
      steering = rx_buf[2];
      tombol_start = rx_buf[3];
      tombol_select = rx_buf[4];
      tombol_l1 = rx_buf[5];
      tombol_l2 = rx_buf[6];
      tombol_r1 = rx_buf[7];
      tombol_r2 = rx_buf[8];
      tombol_s = rx_buf[9];
      tombol_b = rx_buf[10];
      tombol_x = rx_buf[11];
      tombol_k = rx_buf[12];
      tombol_up = rx_buf[13];
      tombol_down = rx_buf[14];
      tombol_left = rx_buf[15];
      tombol_right = rx_buf[16];
      
      //Serial.print(data3); // analog kanan atas bawah
      //Serial.print(" "); 
      //Serial.println(tombol_right); // analog kiri kanan kiri

      //StrServoSetting = map(steering,-256,256,0,179);
      //StrServoSetting = steering;
      StrServoSetting = map(steering,0,255,BelokKiri,BelokKanan);
      if(StrServoSetting < 90){
        //StrServoSetting = 114;
        
      } else if (StrServoSetting > 90){
        //StrServoSetting = 180;
        
      }
      
      if(tombol_left != 0){
        StrServoSetting = BelokKiri;
      } else if(tombol_right != 0){
        StrServoSetting = BelokKanan;
      }
      
      //Write it to the Servos
      SteeringServo.write(StrServoSetting);
      if(StrServoSetting != 90){
        Serial.println(StrServoSetting);
      }
      //delay(5);

      // lampu
      digitalWrite(PinLampu, LOW);
      if(tombol_k != 0){
        Serial.println(tombol_k);
        digitalWrite(PinLampu, HIGH);
      }
       // sein kiri
      if(tombol_l2 != 0){
        //Serial.println(tombol_b);
        // led status, blinking light
        if (currentMillis - previousMillis >= interval) {
          // save the last time you blinked the LED
          previousMillis = currentMillis;
      
          // if the LED is off turn it on and vice-versa:
          if (ledState == LOW) {
            ledState = HIGH;
          } else {
            ledState = LOW;
          }
      
          // set the LED with the ledState of the variable:
          digitalWrite(PinSl, ledState);
        }
      }
      // sein kanan
      if(tombol_r2 != 0){
        //Serial.println(tombol_b);
        // led status, blinking light
        if (currentMillis - previousMillis >= interval) {
          // save the last time you blinked the LED
          previousMillis = currentMillis;
      
          // if the LED is off turn it on and vice-versa:
          if (ledState == LOW) {
            ledState = HIGH;
          } else {
            ledState = LOW;
          }
      
          // set the LED with the ledState of the variable
          digitalWrite(PinSr, ledState);
        }
      }
      // lampu Hazard
      if(tombol_b != 0){
        //Serial.println(tombol_b);
        // led status, blinking light
        if (currentMillis - previousMillis >= interval) {
          // save the last time you blinked the LED
          previousMillis = currentMillis;
      
          // if the LED is off turn it on and vice-versa:
          if (ledState == LOW) {
            ledState = HIGH;
          } else {
            ledState = LOW;
          }

          if (ledStati == LOW) {
            ledStati = HIGH;
          } else {
            ledStati = LOW;
          }
      
          // set the LED with the ledState of the variable:
          digitalWrite(PinSl, ledState);
          digitalWrite(PinSr, ledState);
          digitalWrite(PinLampu, ledStati);
        }
      }
      
      // Gas gas
      // neutral 0, up 255, down -255
      GasSetting = map(gas, 128 , 0, 0 , 255);
      //GasSetting = gas;
      
      if(tombol_up != 0){
        GasSetting = GasSpeed;
      } else if(tombol_down != 0){
        GasSetting = -GasSpeed;
      }
      
      if(GasSetting != 0){
        Serial.println(GasSetting);
        digitalWrite(PinSx, LOW);
      } else {
        digitalWrite(PinSx, HIGH);
      }
      //delay(5);
      
      if(GasSetting > 40 ){
        digitalWrite(PinGasb, LOW);
        digitalWrite(PinGas, HIGH);
        analogWrite(PinGas, GasSetting);
        //Serial.println(GasSetting);
        //delay(30);
      } else if(GasSetting < -40 ){
        digitalWrite(PinGasb, HIGH);
        digitalWrite(PinGas, LOW);
        analogWrite(PinGas, GasSetting);
        //Serial.println(GasSetting);
        //delay(30);
      } else {
        analogWrite(PinGas, 0);
        digitalWrite(PinGasb, LOW);
        digitalWrite(PinGas, LOW);
      }

      SPI_RW_Reg(WRITE_REG+STATUS,0xff);
    }
    else
    {
      SPI_RW_Reg(WRITE_REG+STATUS,0xff);
    }
        
  //}
  delay(1);
  
}

void radio_settings()
{
        
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_AA, 0x01);          //enable auto acc on pip 1
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0x01);      //enable pip 1
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);        //4 byte adress
  
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_SETUP_RETR, B00001010);        //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit Delay
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_RF_CH, 40);
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_RF_SETUP, 0x4f);        //2mps 0x4f
  //SPI_RW_Reg(WRITE_REG|iRF_BANK0_DYNPD, 0x01);          //pipe0 pipe1 enable dynamic payload length data
  //SPI_RW_Reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
  
  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);  //from tx
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
        
}
