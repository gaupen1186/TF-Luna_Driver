#ifndef __TF_LUNA_H__
#define __TF_LUNA_H__

#include <stdint.h>
#include <stdbool.h>
#include "luna_platform.h"


#define LUNA_DEVICE_ADDR    0x10


/* Registers definition */
#define REG_DIST_LOW                0x00    // 距离，cm
#define REG_DIST_HIGH               0x01
#define REG_AMP_LOW                 0x02
#define REG_AMP_HIGH                0x03
#define REG_TEMP_LOW                0x04    // 温度，0.01摄氏度
#define REG_TMEP_HIGH               0x05
#define REG_TICK_LOW                0x06    // 时间戳
#define REG_TICK_HIGH               0x07
#define REG_ERR_LOW                 0x08    // 错误状态码
#define REG_ERR_HIGH                0x09
#define REG_VER_REV                 0x0A    // 修订版本
#define REG_VER_MINOR               0x0B    // 次版本
#define REG_VER_MAJOR               0x0C    // 主版本
#define REG_SN                      0x10    // 生产编码，ASSCII码，14bytes，0x10 ~ 0x1D
#define REG_DEALIAS_EN              0X1E    // 写 0x00 单频模式, 写 0x01 双频模式
#define REG_ULTAR_LOW_POWER         0x1F    // 超低功耗,写1启用,写0回到标准模式
#define REG_SAVE                    0x20    // 写 0x01 保存当前设置的寄存器值
#define REG_SHUTDOWN                0x21    // 写 0x02 重启
#define REG_SLAVE_ADDR              0x22    // 范围 0x08 ~ 0x77
#define REG_MODE                    0x23    // 0x00 连续工作模式，0x01 指令触发模式
#define REG_TRIG                    0x24    // 0x01 触发一次测距，仅 REG_MODE = 0x01 时有效
#define REG_ENABLE                  0x25    // 0x01 雷达开启， 0x01 雷达关闭
#define REG_FPS_LOW                 0x26    // 帧率
#define REG_FPS_HIGH                0x27
#define REG_LOW_POWER               0x28    // 0x00 标准模式，0x01 低功耗模式
#define REG_RST_DEFAULTS            0x29    // 写 0x01 恢复出厂默认值
#define REG_AMP_THR_LOW             0x2A    // AMP阈值， AMP低于阈值后，距离固定为 DUMMY_DIST
#define REG_AMP_THR_HIGH            0x2B
#define REG_DUMMY_DIST_LOW          0x2C    // 当AMP低于 AMP_THR输出的距离值，cm
#define REG_DUMMY_DIST_HIGH         0x2D
#define REG_MIN_DIST_LOW            0x2E    // 最小距离值 mm, DUMMY_DIST不受该限制
#define REG_MIN_DIST_HIGH           0x2F
#define REG_MAX_DIST_LOW            0x30    // 最大距离值 mm, DUMMY_DIST不受该限制
#define REG_MAX_DIST_HIGH           0x31
/* 开关量模式 */
#define REG_ONOFF_MODE_DIST_LOW     0x32    // 开关量模式的临界值(滞回区间的近端点值), mm
#define REG_ONOFF_MODE_DIST_HIGH    0x33
#define REG_ONOFF_MODE_ZONE_LOW     0x34    // 开关量模式的滞回区间大小, mm
#define REG_ONOFF_MODE_ZONE_HIGH    0x35
// 开关量模式的防抖延时时间1, 单位ms, 当距离由远变近超过近端阈值，且保持 Delay1 ms一直小于近端阈值，才切换电平
#define REG_ONOFF_MODE_DELAY1_LOW   0x36
#define REG_ONOFF_MODE_DELAY1_HIGH  0x37
// 开关量模式的防抖延时时间2, 单位ms, 当距离由近变远超过远端阈值，且保持 Delay2 ms一直大于远端阈值，才切换电平
#define REG_ONOFF_MODE_DELAY2_LOW   0x38
#define REG_ONOFF_MODE_DELAY2_HIGH  0x39
#define REG_ONOFF_MODE_EN           0x3A    // 开关量模式开关，0=关闭, 1=近高远低, 2=近低远高
/*************/
#define REG_IAP_OUT                 0x40    // 固件升级时外部读取 TF-Luna 响应的寄存器
#define REG_IAP_IN                  0x41    // 固件升级时外部发送给 TF-Luna 命令或数据的寄存器


#define LUNA_SN_LENGTH                   14      // SN 长度为 14bytes


typedef struct
{
  uint8_t major;
  uint8_t minor;
  uint8_t rev;
  uint8_t sn[LUNA_SN_LENGTH];
  uint8_t dummy;
}
luna_version_t;


typedef enum
{
  LUNA_MODE_CONTINUOUS  = 0x00,
  LUNA_MODE_TRIGGER     = 0x01,
}
luna_mode_t;

typedef enum
{
  LUNA_FRQ_SINGLE       = 0x00,   // 单频模式
  LUNA_FRQ_DOUBLE       = 0x01,   // 双频模式
}
luna_frq_mode_t;

typedef enum
{
  LUNA_ONOFF_MODE_DISABLE   = 0,  // 开关量模式关闭
  LUNA_ONOFF_MODE_NEAR_HIGH = 1,  // 开关量模式，近输出高电平远输出低电平
  LUNA_ONOFF_MODE_NEAR_LOW  = 2,  // 开关量模式，近输出低电平远输出高电平
}
luna_onoff_mode_t;

typedef struct
{
  luna_onoff_mode_t   mode;
  uint16_t            dist_cm;
  uint16_t            zone_cm;
  uint16_t            delay1_ms;
  uint16_t            delay2_ms;
}
luna_onoff_cfg_t;


bool luna_get_distance_cm(uint16_t *distance, uint16_t *amp);
bool luna_get_internal_temprature(int16_t *temp);
bool luna_get_timestamp(uint16_t *ms);
bool luna_get_error_code(uint16_t *error);
bool luna_get_version_sn(luna_version_t *version, uint8_t retry_cnt);
bool luna_set_frq_mode(luna_frq_mode_t frq_mode);
bool luna_get_frq_mode(luna_frq_mode_t *frq_mode);
bool luna_save_settings(void);
bool luna_reboot(uint8_t retry_cnt);
bool luna_set_slave_addr(uint8_t addr);
bool luna_get_slave_addr(uint8_t *addr);
bool luna_set_mode(luna_mode_t mode);
bool luna_get_mode(luna_mode_t *mode);
bool luna_start_one_shot_range(void);
bool luna_set_ir_radar(bool enable);
bool luna_set_data_fps(uint16_t fps);
bool luna_get_data_fps(uint16_t *fps);
bool luna_set_low_power(void);
bool luna_exit_low_power(void);
bool luna_ultra_low_power(bool enable);
bool luna_reset_defaults(void);
bool luna_set_amp_threshold(uint16_t amp);
bool luna_get_amp_threshold(uint16_t *amp);
bool luna_set_dummy_distance(uint16_t dummy_cm);
bool luna_get_dummy_distance(uint16_t *dummy_cm);
bool luna_set_min_distance(uint16_t min_cm);
bool luna_get_min_distance(uint16_t *min_cm);
bool luna_set_max_distance(uint16_t max_cm);
bool luna_get_max_distance(uint16_t *max_cm);
bool luna_onoff_mode_enable(luna_onoff_cfg_t const *cfg);
bool luna_onoff_mode_disable(void);
bool luna_get_onoff_mode_cfg(luna_onoff_cfg_t *cfg);


#endif // __TF_LUNA_H__
