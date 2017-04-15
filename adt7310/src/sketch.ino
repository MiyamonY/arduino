#include <SPI.h>

#define SPI_CS_PIN 10

#define ADT7310_ADDR_STATUS (0x00 << 3)
#define ADT7310_ADDR_CONFIG (0x1 << 3)
#define ADT7310_ADDR_TEMP (0x02 << 3)

#define ADT7310_CONFIG_MODE_CONTINUOUS (0x00 << 2)
#define ADT7310_CONFIG_MODE_ONE_SHOT (0x01 << 2)
#define ADT7310_CONFIG_MODE_1SPS (0x10 << 2)
#define ADT7310_CONFIG_MODE_SHUTDOWN (0x11<<2)

#define ADT7310_CMD_READ (0x01 << 6)
#define ADT7310_CMD_WRITE (0x00 << 6)
#define ADT7310_CMD_CONTINUOUS (0x01 << 2)


static inline void SPI_CS_ACTIVE(void)
{
  digitalWrite(SPI_CS_PIN, LOW);
  return;
}

static inline void SPI_CS_IDLE(void)
{
  digitalWrite(SPI_CS_PIN, HIGH);
  return;
}

uint8_t transfer(uint8_t data)
{
  SPI_CS_ACTIVE();
  uint8_t ret = SPI.transfer(data);
  SPI_CS_IDLE();

  return ret;
}

void adt7310_reset(void)
{
  uint8_t reset[] = {0xFF, 0xFF, 0xFF, 0xFF};

  SPI_CS_ACTIVE();
  SPI.transfer(reset, sizeof(reset)/sizeof(reset[0]));
  SPI_CS_IDLE();
  return;
}

void adt73010_write(uint8_t addr, uint8_t data)
{
  SPI_CS_ACTIVE();
  SPI.transfer(addr | ADT7310_CMD_WRITE);
  SPI.transfer(data);
  SPI_CS_IDLE();
  return;
}

uint8_t adt7310_read(uint8_t addr, bool continuous)
{
  SPI_CS_ACTIVE();
  uint8_t command = (addr | ADT7310_CMD_READ) | (continuous? ADT7310_CMD_CONTINUOUS:0x00);
  SPI.transfer(command);
  uint8_t ret = SPI.transfer(0x00);
  SPI_CS_IDLE();
  return ret;
}

void adt7310_set_temp_read_continuous(void)
{
  SPI_CS_ACTIVE();
  uint8_t command = ADT7310_ADDR_TEMP | ADT7310_CMD_READ | ADT7310_CMD_CONTINUOUS;
  SPI.transfer(command);
  SPI_CS_IDLE();
  return;
}

uint16_t adt7310_temp_read_continuous(void)
{
  SPI_CS_ACTIVE();
  uint16_t ret = SPI.transfer16(0x0000);
  SPI_CS_IDLE();
  return ret;
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("START");

  pinMode(SPI_CS_PIN, OUTPUT);
  SPI_CS_IDLE();

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();

  adt7310_reset();
  adt73010_write(ADT7310_ADDR_CONFIG, 0x80);
  delay(500);
  adt7310_set_temp_read_continuous();
  delay(240);

  return;
}

void loop(void)
{
  uint16_t val = adt7310_temp_read_continuous();

  if(val & 0x8000) {
    val -= 32768;
  }
  double tmp = val / 128.0;
  Serial.println(tmp, 2);

  delay(500);

  return;
}
