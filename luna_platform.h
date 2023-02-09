#ifndef __LUNA_PLATFORM_H__
#define __LUNA_PLATFORM_H__

#include <stdint.h>
#include <stdbool.h>


typedef void (luna_interrupt_handler_t) (bool is_rise_edge);


int8_t luna_WriteReg(uint8_t dev, uint8_t reg, uint8_t data);
int8_t luna_WriteRegs(uint8_t dev, uint8_t reg, uint8_t *pdata, uint8_t size);
int8_t luna_ReadRegs(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t size);

bool luna_i2c_init(void);
void luna_platform_int_enable(bool enable);
bool luna_interrupt_init(luna_interrupt_handler_t handler);
void luna_interrrupt_uninit(void);


#endif // __LUNA_PLATFORM_H__
