#include "luna_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define luna_delay_ms   sys_sleep_ms

#define LUNA_INIT_PIN   12

// 等待 i2c 空闲的最大时间
#define LUNA_I2C_WAIT_FREE_MS_MAX     100

static luna_interrupt_handler_t   *luna_platform_int_handler;



//extern void luna_interrupt_handler(void);
static void gpiote_input_pin_handler_callback(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if(pin == LUNA_INIT_PIN && action == NRF_GPIOTE_POLARITY_TOGGLE)
//  if(pin == LUNA_INIT_PIN && action == NRF_GPIOTE_POLARITY_LOTOHI)
//  if(pin == LUNA_INIT_PIN && action == NRF_GPIOTE_POLARITY_HITOLO)
  {
    // low -> high 上升沿
    if(nrf_drv_gpiote_in_is_set(pin) == true)
    {
//      NRF_LOG_WARNING("Luna GPIOTE high level\r\n");
      luna_platform_int_handler(true);
    }
    // high -> low 下降沿
    else
    {
//      NRF_LOG_WARNING("Luna GPIOTE low level\r\n");
      luna_platform_int_handler(false);
    }

  }
}

int8_t luna_WriteReg(uint8_t dev, uint8_t reg, uint8_t data)
{
  // timeout for wait free, return -1
  if(i2c_wait_for_free(LUNA_I2C_WAIT_FREE_MS_MAX) != true)
    return -1;

  uint8_t tx[2];
  tx[0] = reg;
  tx[1] = data;

  // set i2c busy
  i2c_set_busy();
  // nrf twi xfer tx
  ret_code_t ret = i2c_xfer_tx(dev, tx, 2);
  // set i2c free
  i2c_set_free();

  if(ret != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return -1;
  }

  return 0;
}

int8_t luna_WriteRegs(uint8_t dev, uint8_t reg, uint8_t *pdata, uint8_t size)
{
  if(pdata == NULL || size == 0 || size == 255)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return -1;
  }

  // timeout for wait free, return -1
  if(i2c_wait_for_free(LUNA_I2C_WAIT_FREE_MS_MAX) != true)
    return -1;

  uint8_t len = size + 1;
  uint8_t tx[len]; // 变长数组, 需要 C99 + VLA 支持
  tx[0] = reg;
  memcpy(&tx[1], pdata, size);

  // set i2c busy
  i2c_set_busy();
  // nrf twi xfer tx rx
  ret_code_t ret = i2c_xfer_tx(dev, tx, len);
  // set i2c free
  i2c_set_free();

  if(ret != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return -1;
  }

  return 0;
}

int8_t luna_ReadRegs(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t size)
{
  // timeout for wait free, return -1
  if(i2c_wait_for_free(LUNA_I2C_WAIT_FREE_MS_MAX) != true)
    return -1;

  // set i2c busy
  i2c_set_busy();
  // nrf twi xfer tx rx
  ret_code_t ret = i2c_xfer_txrx(dev, &reg, 1, data, size);
  // set i2c free
  i2c_set_free();

  if(ret != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return -1;
  }

  return 0;
}

bool luna_i2c_init(void)
{
  return i2c_init();
}

void luna_platform_int_enable(bool enable)
{
  if(enable == true)
    nrf_drv_gpiote_in_event_enable(LUNA_INIT_PIN, true);
  else
    nrf_drv_gpiote_in_event_disable(LUNA_INIT_PIN);
}

bool luna_interrupt_init(luna_interrupt_handler_t handler)
{
  luna_platform_int_handler = handler;

  ret_code_t err_code;

  // gpiote int init
  if (!nrf_drv_gpiote_is_init())
  {
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
//  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
//  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
//  in_config.pull = NRF_GPIO_PIN_PULLDOWN;

  err_code = nrf_drv_gpiote_in_init(LUNA_INIT_PIN, &in_config, gpiote_input_pin_handler_callback);
  APP_ERROR_CHECK(err_code);

  if(err_code == NRF_SUCCESS)
    return true;

  return false;
}

void luna_interrrupt_uninit(void)
{
  nrf_drv_gpiote_in_event_disable(LUNA_INIT_PIN);
  nrf_drv_gpiote_in_uninit(LUNA_INIT_PIN);
}
