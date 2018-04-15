//this is a copy and paste job made by F2k
#include <PS2X_lib.h>  //for v1.6
#include "se8r01.h"
byte gtemp[5];
byte k=0;
//***************************************************
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  17  // 32 unsigned chars TX payload

#define PS2_DAT      2  //13  //14    
#define PS2_CMD      4  //11  //15
#define PS2_SEL      3  //10  //16
#define PS2_CLK      5  //12  //17

//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

byte type = 0;
byte vibrate = 0;

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0xD0,0xD0,0xD0,0xD0,0xD0
}; // Define a static TX address

unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};// initialize value

unsigned int tombol_s = 0;
unsigned int tombol_b = 0;
unsigned int tombol_x = 0;
unsigned int tombol_k = 0;

unsigned int tombol_up = 0;
unsigned int tombol_down = 0;
unsigned int tombol_left = 0;
unsigned int tombol_right = 0;

unsigned int tombol_l1 = 0;
unsigned int tombol_l2 = 0;
unsigned int tombol_r1 = 0;
unsigned int tombol_r2 = 0;

unsigned int tombol_start = 0;
unsigned int tombol_select = 0;

//***************************************************
void setup() 
{
  pinMode(CEq,  OUTPUT);
  pinMode(SCKq, OUTPUT);
  pinMode(CSNq, OUTPUT);
  pinMode(MOSIq,  OUTPUT);
  pinMode(MISOq, INPUT);
  pinMode(IRQq, INPUT);

  Serial.begin(9600);
  init_io();                        // Initialize IO port
  unsigned char status=SPI_Read(STATUS);
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  Serial.print("status = ");    
  Serial.println(status,HEX);     
  Serial.println("*******************Radio starting*****************");
 

  digitalWrite(CEq, 0);
  
  delay(1);
  se8r01_powerup();
  se8r01_calibration();
  se8r01_setup();
  radio_settings();
  
  //tx mode
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_CONFIG, 0x3E);
  Serial.println("*******************TX****************************");
  
  
  digitalWrite(CEq, 1);
}

void loop() 
{
  ps2x.read_gamepad(); //read controller and set large motor to spin at 'vibrate' speed

  // gas maju mundur     
  tx_buf[1] = ps2x.Analog(PSS_LY);
  //Serial.println(ps2x.Analog(PSS_LY));
  // setir kiri kanan  
  tx_buf[2] = ps2x.Analog(PSS_RX);
  //Serial.println(ps2x.Analog(PSS_RX));
  
  if ( ps2x.ButtonPressed(PSB_START) ) {
    //Serial.println("Start pressed");
    tombol_start = !tombol_start;
  }
  tx_buf[3] = tombol_start;
  
  if ( ps2x.ButtonPressed(PSB_SELECT) ) {
    //Serial.println("Select pressed");
    tombol_select = !tombol_select;
  }
  tx_buf[4] = tombol_select;
  
  if ( ps2x.ButtonPressed(PSB_L1) ) {
    //Serial.println("L1 pressed");
    tombol_l1 = !tombol_l1;
  }
  tx_buf[5] = tombol_l1;
  
  if ( ps2x.ButtonPressed(PSB_L2) ) {
    //Serial.println("L2 pressed");
    tombol_l2 = !tombol_l2;
  }
  tx_buf[6] = tombol_l2;

  tombol_r1 = 0;
  if ( ps2x.Button(PSB_R1) ) {
    //Serial.println("R1 pressed");
    tombol_r1 = 1;
  }
  tx_buf[7] = tombol_r1;

  tombol_r2 = 0;
  if ( ps2x.Button(PSB_R2) ) {
    //Serial.println("R2 pressed");
    tombol_r2 = 1;
  }
  tx_buf[8] = tombol_r2;
  
  if ( ps2x.ButtonPressed(PSB_GREEN) ) {
    //Serial.println("Segitiga pressed");
    tombol_s = !tombol_s;
  }
  tx_buf[9] = tombol_s;
  
  if ( ps2x.ButtonPressed(PSB_RED) ) {
    //Serial.println("Bunder pressed");
    tombol_b = !tombol_b;
  }
  tx_buf[10] = tombol_b;
  
  if ( ps2x.ButtonPressed(PSB_BLUE) ) {
    //Serial.println("X pressed");
    tombol_x = !tombol_x;
  }
  tx_buf[11] = tombol_x;
  
  if ( ps2x.ButtonPressed(PSB_PINK) ) {
    //Serial.println("Kotak pressed");
    tombol_k = !tombol_k;
  }
  tx_buf[12] = tombol_k;

  tombol_up = 0;
  if ( ps2x.Button(PSB_PAD_UP) ) {
    //Serial.println("Up pressed");
    tombol_up = 1;
  }
  tx_buf[13] = tombol_up;

  tombol_down = 0;
  if ( ps2x.Button(PSB_PAD_DOWN) ) {
    //Serial.println("Down pressed");
    tombol_down = 1;
  }
  tx_buf[14] = tombol_down;

  tombol_left = 0;
  if ( ps2x.Button(PSB_PAD_LEFT) ) {
    //Serial.println("Left pressed");
    tombol_left = 1;
  }
  tx_buf[15] = tombol_left;

  tombol_right = 0;
  if ( ps2x.Button(PSB_PAD_RIGHT) ) {
    //Serial.println("Right pressed");
    tombol_right = 1;
  }
  tx_buf[16] = tombol_right;
        
  unsigned char status = SPI_Read(STATUS); 
 
  SPI_RW_Reg(FLUSH_TX,0);
  SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);     
    
  SPI_RW_Reg(WRITE_REG+STATUS,0xff);   // clear RX_DR or TX_DS or MAX_RT interrupt flag
  //Serial.println(tombol_right);    
  delay(10);
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
