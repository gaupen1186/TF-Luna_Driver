/* 本代码基于 TF-LUNA 的 fw v3.1.3 实现
 */

#include <string.h>
#include <math.h>
#include "tf_luna.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define LUNA_DELAY_MS     sys_sleep_ms


/*实测功耗数据
  100Hz 连续测距模式          66mA @ 5V
  100Hz 连续测距模式下，关闭IR雷达 从 66mA 降到 25mA
  低功耗模式                  12mA @ 5V
  手动单次触发模式（不触发）  8.5mA @ 5V
*/


bool luna_get_distance_cm(uint16_t *distance, uint16_t *amp)
{
  uint8_t val[4];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_DIST_LOW, val, 4) == 0)
  // if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_AMP_LOW, val, 2) == 0)
  {
    *distance = (val[1] << 8) | val[0];
    *amp = (val[3] << 8) | val[2];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

// temp, 0.01 摄氏度
bool luna_get_internal_temprature(int16_t *temp)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_TEMP_LOW, val, 2) == 0)
  {
	int16_t temp001 = (val[1] << 8) | val[0];
    *temp = (int16_t)round(temp001 / 100.0);
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_timestamp(uint16_t *ms)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_TICK_LOW, val, 2) == 0)
  {
    *ms = (val[1] << 8) | val[0];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_error_code(uint16_t *error)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_ERR_LOW, val, 2) == 0)
  {
    *error = (val[1] << 8) | val[0];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_version_sn(luna_version_t *version, uint8_t retry_cnt)
{
  if(version == NULL)
    return false;

  int32_t ret;
  uint8_t cnt = 0;
  do
  {
    cnt ++;
    ret = 0;
    ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_VER_MAJOR, &version->major, 1);
    ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_VER_MINOR, &version->minor, 1);
    ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_VER_REV, &version->rev, 1);
    ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_SN, version->sn, LUNA_SN_LENGTH);
    if(ret == 0)
    {
      NRF_LOG_WARNING("[%s]: TF-Luna Ver:%d.%d.%d. Try %u\r\n", (uint32_t)__func__,
                      version->major, version->minor, version->rev, cnt);
      return true;
    }
    // sleep & retry
    LUNA_DELAY_MS(10);
  }
  while(cnt <= retry_cnt);

  memset(version, 0x00, sizeof(luna_version_t));
  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

// 设置单双频模式, 保存并重启后生效，调用后需要约 300 ~ 400ms 后才能完成
bool luna_set_frq_mode(luna_frq_mode_t frq_mode)
{
  // params check
  if(frq_mode != LUNA_FRQ_SINGLE && frq_mode != LUNA_FRQ_DOUBLE)
    return false;

  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_DEALIAS_EN, frq_mode) != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return false;
  }
  return true;
}

bool luna_get_frq_mode(luna_frq_mode_t *frq_mode)
{
  uint8_t val;
  int8_t ret = luna_ReadRegs(LUNA_DEVICE_ADDR, REG_DEALIAS_EN, &val, 1);

  if(ret != 0 && *frq_mode != LUNA_FRQ_SINGLE && *frq_mode != LUNA_FRQ_DOUBLE)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return false;
  }

  *frq_mode = (luna_frq_mode_t)val;
  return true;
}

bool luna_save_settings(void)
{
  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_SAVE, 0x01) == 0)
  {
//    NRF_LOG_WARNING("[%s]: OK\r\n", (uint32_t)__func__);
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_reboot(uint8_t retry_cnt)
{
  uint8_t cnt = 0;
  do
  {
    cnt ++;
    if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_SHUTDOWN, 0x02) == 0)
    {
      NRF_LOG_WARNING("[%s]: OK! Try %u\r\n", (uint32_t)__func__, cnt);
      return true;
    }
    // sleep & retry
    LUNA_DELAY_MS(10);
  }
  while(cnt <= retry_cnt);

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_slave_addr(uint8_t addr)
{
  if(addr < 0x08 || addr > 0x77)
    return false;

  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_SLAVE_ADDR, addr) == 0)
  {
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_slave_addr(uint8_t *addr)
{
	uint8_t val;
	if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_SLAVE_ADDR, &val, 1) == 0)
	{
		*addr = val;
		return true;
	}

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_mode(luna_mode_t mode)
{
  if(mode != LUNA_MODE_CONTINUOUS && mode != LUNA_MODE_TRIGGER)
    return false;

  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_MODE, mode) != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return false;
  }

#if NRF_LOG_ENABLED != 0
  if(mode == LUNA_MODE_CONTINUOUS)
  {
//    NRF_LOG_WARNING("[%s]: LUNA_MODE_CONTINUOUS\r\n", (uint32_t)__func__);
  }
  else if(mode == LUNA_MODE_TRIGGER)
  {
//    NRF_LOG_WARNING("[%s]: LUNA_MODE_TRIGGER\r\n", (uint32_t)__func__);
  }
#endif
  return true;
}

bool luna_get_mode(luna_mode_t *mode)
{
	uint8_t val;
	if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_MODE, &val, 1) == 0)
	{
		*mode = (luna_mode_t) val;
		return true;
	}

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_start_one_shot_range(void)
{
  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_TRIG, 0x01) == 0)
  {
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_ir_radar(bool enable)
{
  uint8_t data;
  if(enable == false)
    data = 0x00;
  else
    data = 0x01;

  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_ENABLE, data) == 0)
  {
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

/* Set data ouput in Hz
   only valid in LUNA_MODE_CONTINUOUS mode
*/
bool luna_set_data_fps(uint16_t fps)
{
  int8_t ret = 0;
  uint8_t data[2];
  data[0] = (uint8_t)(fps & 0x00ff);
  data[1] = (uint8_t)((fps >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_FPS_LOW, data, 2);
  if(ret == 0)
  {
//    NRF_LOG_WARNING("set DATA FPS = %d Hz\r\n", fps);
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_data_fps(uint16_t *fps)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_FPS_LOW, val, 2) == 0)
  {
    *fps = (val[1] << 8) | val[0];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_low_power(void)
{
  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_LOW_POWER, 0x01) == 0)
  {
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_exit_low_power(void)
{
  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_LOW_POWER, 0x00) == 0)
  {
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

/* 超低功耗设置
   enable: true = 进入超低功耗, false = 退出
   根据 Datasheet, 进入低功耗后需保存设置
*/
bool luna_ultra_low_power(bool enable)
{
  int8_t ret = 0;
  uint8_t data[3];
  if(enable)
  {
    data[0] = 0x01;
  }
  else
  {
    data[0] = 0x00;
  }
  data[1] = 0x01;
  data[2] = 0x02;

  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_ULTAR_LOW_POWER, data, 3);
  if(ret == 0)
  {
//    NRF_LOG_WARNING("set DATA FPS = %d Hz\r\n", fps);
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

// need wait sometimes to take effect after reset defaults, 实测需要约 400ms 才能真正生效
bool luna_reset_defaults(void)
{
  if(luna_WriteReg(LUNA_DEVICE_ADDR, REG_RST_DEFAULTS, 0x01) == 0)
  {
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_amp_threshold(uint16_t amp)
{
  int8_t ret = 0;
  uint8_t data[2];
  data[0] = (uint8_t)(amp & 0x00ff);
  data[1] = (uint8_t)((amp >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_AMP_THR_LOW, data, 2);
  if(ret == 0)
    return true;

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_amp_threshold(uint16_t *amp)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_AMP_THR_LOW, val, 2) == 0)
  {
    *amp = (val[1] << 8) | val[0];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_dummy_distance(uint16_t dummy_cm)
{
  int8_t ret = 0;
  uint8_t data[2];
  data[0] = (uint8_t)(dummy_cm & 0x00ff);
  data[1] = (uint8_t)((dummy_cm >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_DUMMY_DIST_LOW, data, 2);
  if(ret == 0)
    return true;

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_dummy_distance(uint16_t *dummy_cm)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_DUMMY_DIST_LOW, val, 2) == 0)
  {
    *dummy_cm = (val[1] << 8) | val[0];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_min_distance(uint16_t min_cm)
{
  int8_t ret = 0;
  uint16_t val = min_cm * 10;
  uint8_t data[2];
  data[0] = (uint8_t)(val & 0x00ff);
  data[1] = (uint8_t)((val >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_MIN_DIST_LOW, data, 2);
  if(ret == 0)
    return true;

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_min_distance(uint16_t *min_cm)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_MIN_DIST_LOW, val, 2) == 0)
  {
    *min_cm = (val[1] << 8) | val[0];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_set_max_distance(uint16_t max_cm)
{
  int8_t ret = 0;
  uint16_t val = max_cm * 10;

  uint8_t data[2];
  data[0] = (uint8_t)(val & 0x00ff);
  data[1] = (uint8_t)((val >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_MAX_DIST_LOW, data, 2);
  if(ret == 0)
    return true;

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_max_distance(uint16_t *max_cm)
{
  uint8_t val[2];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_MAX_DIST_LOW, val, 2) == 0)
  {
    *max_cm = ((val[1] << 8) | val[0]) / 10;
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_onoff_mode_enable(luna_onoff_cfg_t const *cfg)
{
  if(cfg == NULL || (cfg->mode != LUNA_ONOFF_MODE_NEAR_HIGH && cfg->mode != LUNA_ONOFF_MODE_NEAR_LOW) ||
     cfg->zone_cm >= cfg->dist_cm)
    return false;

  uint16_t dist = cfg->dist_cm * 10;
  uint16_t zone = cfg->zone_cm * 10;

  uint8_t data[9];
  data[0] = (uint8_t)(dist & 0x00ff);
  data[1] = (uint8_t)((dist >> 8) & 0x00ff);
  data[2] = (uint8_t)(zone & 0x00ff);
  data[3] = (uint8_t)((zone >> 8) & 0x00ff);
  data[4] = (uint8_t)(cfg->delay1_ms & 0x00ff);
  data[5] = (uint8_t)((cfg->delay1_ms >> 8) & 0x00ff);
  data[6] = (uint8_t)(cfg->delay2_ms & 0x00ff);
  data[7] = (uint8_t)((cfg->delay2_ms >> 8) & 0x00ff);
  data[8] = (uint8_t)cfg->mode;
  if(luna_WriteRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DIST_LOW, data, 9) != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return false;
  }
  return true;

  /*
  int8_t ret = 0;
  uint8_t data[2];

  // write dist reg
  data[0] = (uint8_t)(dist & 0x00ff);
  data[1] = (uint8_t)((dist >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DIST_LOW, data, 2);

  // write zone reg
  data[0] = (uint8_t)(zone & 0x00ff);
  data[1] = (uint8_t)((zone >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_ZONE_LOW, data, 2);

  // write delay1 reg
  data[0] = (uint8_t)(cfg->delay1_ms & 0x00ff);
  data[1] = (uint8_t)((cfg->delay1_ms >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DELAY1_LOW, data, 2);

  // write delay2 reg
  data[0] = (uint8_t)(cfg->delay2_ms & 0x00ff);
  data[1] = (uint8_t)((cfg->delay2_ms >> 8) & 0x00ff);
  ret += luna_WriteRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DELAY2_LOW, data, 2);

  // write mode
  ret += luna_WriteReg(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_EN, (uint8_t)cfg->mode);

  if(ret == 0)
    return true;

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
  */
}

bool luna_onoff_mode_disable(void)
{
  int8_t ret = 0;
  ret += luna_WriteReg(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_EN, LUNA_ONOFF_MODE_DISABLE);
  if(ret == 0)
    return true;

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;
}

bool luna_get_onoff_mode_cfg(luna_onoff_cfg_t *cfg)
{
  if(cfg == NULL)
    return false;

  uint8_t val[9];
  if(luna_ReadRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DIST_LOW, val, 9) == 0)
  {
    cfg->dist_cm = ((uint16_t) (val[1] << 8) | val[0]) / 10;
    cfg->zone_cm = ((uint16_t) (val[3] << 8) | val[2]) / 10;
    cfg->delay1_ms = ((uint16_t) (val[5] << 8) | val[4]) / 10;
    cfg->delay2_ms = ((uint16_t) (val[7] << 8) | val[6]) / 10;
    cfg->mode = (luna_onoff_mode_t) val[8];
    return true;
  }

  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
  return false;

//  int8_t ret = 0;
////  uint8_t val_low, val_high;
//  uint8_t val[2];
//
//  // read dist reg
//  ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DIST_LOW, val, 2);
//  cfg->dist_cm = ((uint16_t) (val[1] << 8) | val[0]) / 10;
//  // read zone reg
//  ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_ZONE_LOW, val, 2);
//  cfg->zone_cm = ((uint16_t) (val[1] << 8) | val[0]) / 10;
//  // read delay1 reg
//  ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DELAY1_LOW, val, 2);
//  cfg->delay1_ms = ((uint16_t) (val[1] << 8) | val[0]) / 10;
//  // read delay2 reg
//  ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_DELAY2_LOW, val, 2);
//  cfg->delay2_ms = ((uint16_t) (val[1] << 8) | val[0]) / 10;
//  // read mode
//  ret += luna_ReadRegs(LUNA_DEVICE_ADDR, REG_ONOFF_MODE_EN, (uint8_t *)&cfg->mode, 1);
//
//  if(ret == 0)
//    return true;
//
//  NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
//  return false;
}
