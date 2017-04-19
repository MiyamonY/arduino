#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/**
   PCF8574TとLCDの接続
   PCF8574T 	P0 	P1 	P2 	P4 	 P4 	P5 	P6 	P7
   LCD 	      RS 	RW 	EN 	N/C  D4 	D5 	D6 	D7
 */

#define HILETGO_2004A_ADDR 0x27

#define BLACK_LIGHT_ON 0x08
#define BLACK_LIGHT_OFF 0x00
static uint8_t blackligh = BLACK_LIGHT_ON;

#define CONTROL_BYTE(data) (data)
#define CONTROL_BIT_RS_DATA (0x01<<0)
#define CONTROL_BIT_RS_COM (0x00<<0)
#define CONTROL_BIT_RW_READ (0x01<<1)
#define CONTROL_BIT_RW_WRITE (0x00<<1)
#define CONTROL_BIT_EN_ENABLE (0x01<<2)
#define CONTROL_BIT_EN_DISABLE (0x00<<2)

#define CLEAR_DISPLAY() (0x01<<0)
#define RETURN_HOME() (0x01<<1)

#define ENTRY_MODE_SET(data) ((0x01<<2) | data)
#define ENTRY_MODE_SET_ID_INCR (0x01 << 1)
#define ENTRY_MODE_SET_ID_DECR (0x00 << 1)
#define ENTRY_MODE_SET_S_SHIFT (0x01 << 0)
#define ENTRY_MODE_SET_S_NOT_SHIFT (0x00 << 0)

#define DISPLAY_ON_OFF_CTRL(data) ((0x01<<3) | data)
#define DISPLAY_ON_OFF_CTRL_D_ON (0x01 << 2)
#define DISPLAY_ON_OFF_CTRL_D_OFF (0x00 << 2)
#define DISPLAY_ON_OFF_CTRL_C_CURSOR_ON (0x01 << 1)
#define DISPLAY_ON_OFF_CTRL_C_CURSOR_OFF (0x00 << 1)
#define DISPLAY_ON_OFF_CTRL_B_BLINKS_ON (0x01 << 0)
#define DISPLAY_ON_OFF_CTRL_B_BLINKS_OFF (0x00 << 0)

#define FUNCTION_SET(data) ((0x01<<5) | (data))
#define FUNCTION_SET_DL_8BIT (0x01<<4)
#define FUNCTION_SET_DL_4BIT (0x00<<4)
#define FUNCTION_SET_N_2LINE (0x01<<3)
#define FUNCTION_SET_N_1LINE (0x00<<3)
#define FUNCTION_SET_DH_10DOT (0x01<<2)
#define FUNCTION_SET_DH_7DOT (0x00<<2)
#define FUNCTION_SET_IS_ON  (0x01<<0)
#define FUNCTION_SET_IS_OFF (0x00<<0)

#define DDRAM_ADDR_SET(data) (0x1<<7 | (data))

void lcd_send_enable(uint8_t data)
{
  Wire.beginTransmission(HILETGO_2004A_ADDR);
  Wire.write(data | CONTROL_BIT_EN_ENABLE | blackligh);
  Wire.endTransmission();
  delayMicroseconds(1);

  Wire.beginTransmission(HILETGO_2004A_ADDR);
  Wire.write((data & (~CONTROL_BIT_EN_ENABLE)) | blackligh);
  Wire.endTransmission();
  delayMicroseconds(50);

  return;
}

void lcd_send_4bits(uint8_t data)
{
  Serial.print("send data: 0b");
  Serial.println(data, 2);
  Wire.beginTransmission(HILETGO_2004A_ADDR);
  Wire.write(data | blackligh);
  Wire.endTransmission();

  /* 先にRX, R/Wを上げておいてE(nable)端子をH/Lする */
  lcd_send_enable(data);
}

void lcd_send_com(uint8_t data)
{
  uint8_t high = data & 0xf0;
  lcd_send_4bits(high | CONTROL_BIT_RS_COM | CONTROL_BIT_RW_WRITE);
  uint8_t low = (data<<4) & 0xf0;
  lcd_send_4bits(low | CONTROL_BIT_RS_COM | CONTROL_BIT_RW_WRITE);
  return;
}

void lcd_send_data(uint8_t data)
{
  uint8_t high = data & 0xf0;
  lcd_send_4bits(high | CONTROL_BIT_RS_DATA | CONTROL_BIT_RW_WRITE);
  uint8_t low = (data<<4) & 0xf0;
  lcd_send_4bits(low | CONTROL_BIT_RS_DATA | CONTROL_BIT_RW_WRITE);

  return;
}

static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
void lcd_move_cursor(uint8_t row, uint8_t col)
{
	lcd_send_com(DDRAM_ADDR_SET(col + row_offsets[row]));
}

void lcd_initialize(void)
{
  delay(50);
  lcd_send_4bits(0x03 << 4);
  delayMicroseconds(4500); // wait min 4.1ms
  lcd_send_4bits(0x03 << 4);
  delayMicroseconds(4500); // wait min 4.1ms
  lcd_send_4bits(0x03 << 4);
  delayMicroseconds(150);
  lcd_send_4bits(0x01<<5);

  lcd_send_com(FUNCTION_SET(FUNCTION_SET_DL_4BIT | FUNCTION_SET_N_2LINE | FUNCTION_SET_DH_7DOT));

  lcd_send_com(DISPLAY_ON_OFF_CTRL(DISPLAY_ON_OFF_CTRL_D_ON));
  /* lcd_send_com(ENTRY_MODE_SET(ENTRY_MODE_SET_ID_INCR | ENTRY_MODE_SET_S_SHIFT)); */
  lcd_send_com(CLEAR_DISPLAY());
  lcd_send_com(RETURN_HOME());

  return;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  lcd_initialize();
  lcd_move_cursor(0,0);
  lcd_send_data((uint8_t)'W');
  lcd_send_data((uint8_t)'e');
  lcd_send_data((uint8_t)'l');
  lcd_send_data((uint8_t)'c');
  lcd_send_data((uint8_t)'o');
  lcd_send_data((uint8_t)'m');
  lcd_send_data((uint8_t)'e');
  lcd_move_cursor(1,0);
  lcd_send_data((uint8_t)'H');
  lcd_send_data((uint8_t)'e');
  lcd_send_data((uint8_t)'l');
  lcd_send_data((uint8_t)'l');
  lcd_send_data((uint8_t)'o');
  lcd_move_cursor(2,0);
  lcd_send_data((uint8_t)'W');
  lcd_send_data((uint8_t)'o');
  lcd_send_data((uint8_t)'r');
  lcd_send_data((uint8_t)'l');
  lcd_send_data((uint8_t)'d');
  lcd_send_data((uint8_t)'!');

  lcd_move_cursor(0,0);
  for(uint8_t i = 0xB1; i < 0xff; i++){
    lcd_send_data((uint8_t)i);
  }

  /* test(); */
  return;
}

void loop()
{
}