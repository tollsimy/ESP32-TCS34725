/**
 *  @file ESP32_TCS34725.c
 *
 *  @mainpage Driver for the TCS34725 digital color sensors for ESP32
 *
 *  @author Tollsimy, KTOWN (Adafruit Industries)
 * 
 *  @section license License
 * 
 *  BSD (see LICENSE)
 *
 *  @version v0.1
 * 
 */

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"

#include "ESP32_TCS34725.h"

// SUPPORT FUNCTIONS

/**
 *  @brief  Implements missing powf function
 *  @param  x
 *          Base number
 *  @param  y
 *          Exponent
 *  @return x raised to the power of y
 */
float powf(const float x, const float y) {
  return (float)(pow((double)x, (double)y));
}

/**
 *  @brief  Initializes I2C and configures the sensor
 * 
 *  @param  *TCS
 *
 *  @return True if initialization was successful, otherwise false.
 */
esp_err_t TCS_init(ESP32_TCS34725 *TCS) {

  TCS->_tcs34725Initialised = true;
  TCS->_tcs34725Gain = TCS34725_GAIN_1X;
  TCS->_tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_2_4MS;

  TCS->conf.mode = I2C_MODE_MASTER;
  TCS->conf.sda_io_num = SDA_PIN;
  TCS->conf.scl_io_num = SCL_PIN;
  TCS->conf.sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
  TCS->conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  TCS->conf.master.clk_speed = 400000;               //I2C Full Speed

  ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &(TCS->conf))); //set I2C Config

  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

  /* Make sure we're actually connected */
  uint8_t x = read8(TCS,TCS34725_ID);
  if ((x != 0x4d) && (x != 0x44) && (x != 0x10)) {
    return false;
  }
  TCS->_tcs34725Initialised = true;

  /* Set default integration time and gain */
  setIntegrationTime(TCS, TCS->_tcs34725IntegrationTime);
  setGain(TCS, TCS->_tcs34725Gain);

  /* Note: by default, the device is in power down mode on bootup */
  TCS_enable(TCS);

  return ESP_OK;
}

/**
 *  @brief  Delete the device
 */
esp_err_t TCS_delete(){
  ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
  return ESP_OK;
}

/**
 *  @brief  Enables the device
 */
void TCS_enable(ESP32_TCS34725* TCS) {
  write8(TCS,TCS34725_ENABLE, TCS34725_ENABLE_PON);
  vTaskDelay(3/portTICK_PERIOD_MS);
  write8(TCS,TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  vTaskDelay(((256 - TCS->_tcs34725IntegrationTime) * 12 / 5 + 1)/portTICK_PERIOD_MS);
}

/**
 *  @brief  Disables the device (putting it in lower power sleep mode)
 */
void TCS_disable(ESP32_TCS34725* TCS) {
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS,TCS34725_ENABLE);
  write8(TCS,TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/**
 *  @brief  Sets interrupt for TCS34725
 *  @param  i
 *          Interrupt (True/False)
 */
void setInterrupt(ESP32_TCS34725* TCS, bool i) {
  uint8_t r = read8(TCS,TCS34725_ENABLE);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  write8(TCS,TCS34725_ENABLE, r);
}

/**
 *  @brief  Sets inerrupt limits
 *  @param  low
 *          Low limit
 *  @param  high
 *          High limit
 */
void setIntLimits(ESP32_TCS34725* TCS, uint16_t low, uint16_t high) {
  write8(TCS, 0x04, low & 0xFF);
  write8(TCS, 0x05, low >> 8);
  write8(TCS, 0x06, high & 0xFF);
  write8(TCS, 0x07, high >> 8);
}

/**
 *  @brief  Clears inerrupt for TCS34725
 */
void clearInterrupt(ESP32_TCS34725* TCS) {
  uint8_t buffer[1] = {TCS34725_COMMAND_BIT | 0x66};
  ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_RATE_MS));
}

// READ/WRITE ETC.

/**
 *  @brief  Writes a register and an 8 bit value over I2C
 *  @param  reg
 *  @param  value
 */
void write8(ESP32_TCS34725* TCS, uint8_t reg, uint32_t value) {
  uint8_t buffer[2] = {TCS34725_COMMAND_BIT | reg, value & 0xFF};

  /*
  TCS->cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(TCS->cmd));
  ESP_ERROR_CHECK(i2c_master_write_byte(TCS->cmd,TCS34725_ADDRESS,true));
  ESP_ERROR_CHECK(i2c_master_write(buffer,2,true));
  ESP_ERROR_CHECK(i2c_master_stop(TCS->cmd));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT,TCS->cmd,1000 / portTICK_RATE_MS));
  ESP_ERROR_CHECK(i2c_cmd_link_delete(TCS->cmd));
  */
 
  ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, TCS34725_ADDRESS, buffer, 2, 1000 / portTICK_RATE_MS));
}

/**
 *  @brief  Reads an 8 bit value over I2C
 *  @param  reg
 *  @return value
 */
uint8_t read8(ESP32_TCS34725* TCS, uint8_t reg) {
  uint8_t buffer[1] = {TCS34725_COMMAND_BIT | reg};
  ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_RATE_MS));
  ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_PORT, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_RATE_MS));

  return buffer[0];
}

/**
 *  @brief  Reads a 16 bit values over I2C
 *  @param  reg
 *  @return value
 */
uint16_t read16(ESP32_TCS34725* TCS, uint8_t reg) {
  uint8_t buffer[2] = {TCS34725_COMMAND_BIT | reg, 0};
  ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_RATE_MS));
  ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_PORT, TCS34725_ADDRESS, buffer, 2, 1000 / portTICK_RATE_MS));

  return (((uint16_t)buffer[1]) << 8) | (((uint16_t)buffer[0]) & 0xFF);
}

/**
 *  @brief  Sets the integration time for the TC34725
 *  @param  it
 *          Integration Time
 */
void setIntegrationTime(ESP32_TCS34725* TCS, uint8_t it) {
  if (!TCS->_tcs34725Initialised)
    TCS_init(TCS);

  /* Update the timing register */
  write8(TCS, TCS34725_ATIME, it);

  /* Update value placeholders */
  TCS->_tcs34725IntegrationTime = it;
}

/**
 *  @brief  Adjusts the gain on the TCS34725
 *  @param  gain
 *          Gain (sensitivity to light)
 */
void setGain(ESP32_TCS34725* TCS, tcs34725Gain_t gain) {
  if (!TCS->_tcs34725Initialised)
    TCS_init(TCS);

  /* Update the timing register */
  write8(TCS,TCS34725_CONTROL, gain);

  /* Update value placeholders */
  TCS->_tcs34725Gain = gain;
}

/**
 *  @brief  Reads the raw red, green, blue and clear channel values
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void getRawData(ESP32_TCS34725* TCS, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  if (!TCS->_tcs34725Initialised)
    TCS_init(TCS);

  *c = read16(TCS,TCS34725_CDATAL);
  *r = read16(TCS,TCS34725_RDATAL);
  *g = read16(TCS,TCS34725_GDATAL);
  *b = read16(TCS,TCS34725_BDATAL);

  /* Set a delay for the integration time */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  vTaskDelay(((256 - TCS->_tcs34725IntegrationTime) * 12 / 5 + 1)/portTICK_PERIOD_MS);
}

/**
 *  @brief  Reads the raw red, green, blue and clear channel values in
 *          one-shot mode (e.g., wakes from sleep, takes measurement, enters
 *          sleep)
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void getRawDataOneShot(ESP32_TCS34725* TCS, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  if (!TCS->_tcs34725Initialised)
    TCS_init(TCS);

  TCS_enable(TCS);
  getRawData(TCS, r, g, b, c);
  TCS_disable(TCS);
}

/**
 *  @brief  Read the RGB color detected by the sensor.
 *  @param  *r
 *          Red value normalized to 0-255
 *  @param  *g
 *          Green value normalized to 0-255
 *  @param  *b
 *          Blue value normalized to 0-255
 */
void getRGB(ESP32_TCS34725* TCS, float *r, float *g, float *b) {
  uint16_t red, green, blue, clear;
  getRawData(TCS, &red, &green, &blue, &clear);
  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = *g = *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;
}

/**
 *  @brief  Converts the raw R/G/B values to color temperature in degrees Kelvin
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Color temperature in degrees Kelvin
 */
uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
  float X, Y, Z; /* RGB to XYZ correlation      */
  float xc, yc;  /* Chromaticity co-ordinates   */
  float n;       /* McCamy's formula            */
  float cct;

  if (r == 0 && g == 0 && b == 0) {
    return 0;
  }

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct =
      (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**
 *  @brief  Converts the raw R/G/B values to color temperature in degrees
 *          Kelvin using the algorithm described in DN40 from Taos (now AMS).
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @param  c
 *          Clear channel value
 *  @return Color temperature in degrees Kelvin
 */
uint16_t calculateColorTemperature_dn40(ESP32_TCS34725* TCS, uint16_t r,
                                                           uint16_t g,
                                                           uint16_t b,
                                                           uint16_t c) {
  uint16_t r2, b2; /* RGB values minus IR component */
  uint16_t sat;    /* Digital saturation level */
  uint16_t ir;     /* Inferred IR content */

  if (c == 0) {
    return 0;
}

  /* Analog/Digital saturation:
   *
   * (a) As light becomes brighter, the clear channel will tend to
   *     saturate first since R+G+B is approximately equal to C.
   * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
   *     time, up to a maximum values of 65535. This means analog
   *     saturation can occur up to an integration time of 153.6ms
   *     (64*2.4ms=153.6ms).
   * (c) If the integration time is > 153.6ms, digital saturation will
   *     occur before analog saturation. Digital saturation occurs when
   *     the count reaches 65535.
   */
  if ((256 - TCS->_tcs34725IntegrationTime) > 63) {
    /* Track digital saturation */
    sat = 65535;
  } else {
    /* Track analog saturation */
    sat = 1024 * (256 - TCS->_tcs34725IntegrationTime);
  }

  /* Ripple rejection:
   *
   * (a) An integration time of 50ms or multiples of 50ms are required to
   *     reject both 50Hz and 60Hz ripple.
   * (b) If an integration time faster than 50ms is required, you may need
   *     to average a number of samples over a 50ms period to reject ripple
   *     from fluorescent and incandescent light sources.
   *
   * Ripple saturation notes:
   *
   * (a) If there is ripple in the received signal, the value read from C
   *     will be less than the max, but still have some effects of being
   *     saturated. This means that you can be below the 'sat' value, but
   *     still be saturating. At integration times >150ms this can be
   *     ignored, but <= 150ms you should calculate the 75% saturation
   *     level to avoid this problem.
   */
  if ((256 - TCS->_tcs34725IntegrationTime) <= 63) {
    /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
    sat -= sat / 4;
  }

  /* Check for saturation and mark the sample as invalid if true */
  if (c >= sat) {
    return 0;
  }

  /* AMS RGB sensors have no IR channel, so the IR content must be */
  /* calculated indirectly. */
  ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

  /* Remove the IR component from the raw RGB values */
  r2 = r - ir;
  b2 = b - ir;

  if (r2 == 0) {
    return 0;
  }

  /* A simple method of measuring color temp is to use the ratio of blue */
  /* to red light, taking IR cancellation into account. */
  uint16_t cct = (3810 * (uint32_t)b2) / /** Color temp coefficient. */
                     (uint32_t)r2 +
                 1391; /** Color temp offset. */

  return cct;
}

/**
 *  @brief  Converts the raw R/G/B values to lux
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Lux value
 */
uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
