#ifndef _ST7789_H
#define _ST7789_H

#include "display.h"
#include "display_st7789_reg.h"

// st7789 对象
typedef struct {
	display_t base;
	
	// st7789 移植接口
	int x_offset;
	int y_offset;
	void (*write_raw)(uint8_t *data, size_t len);
	void (*set_dc_cs)(uint32_t new_dc, uint32_t new_cs);
	void (*set_res)(uint32_t new_res);
	void (*set_blacklight)(uint32_t brightness);
	void (*sleep_ms)(uint32_t ms);
} st7789_obj_t;

void st7789_obj_init(st7789_obj_t *self);



#endif
