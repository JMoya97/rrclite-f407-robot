#include "display_st7789.h"
#include "lvgl.h"
static inline void write_command(st7789_obj_t *self, uint8_t command)
{
    self->set_dc_cs(0, 0);
    self->write_raw(&command, 1);
}

static inline void write_data(st7789_obj_t *self, uint8_t *data, size_t len)
{
    self->set_dc_cs(1, 0);
    self->write_raw(data, len);
}

static void write_a_byte(st7789_obj_t *self, uint8_t data) {
	self->set_dc_cs(1, 0);
	self->write_raw(&data, 1);
}

void st7789_hard_reset(st7789_obj_t *self)
{
    self->set_res(1);
    self->sleep_ms(50);
    self->set_res(0);
    self->sleep_ms(50);
    self->set_res(1);
    self->sleep_ms(150);
}


void st7789_soft_reset(st7789_obj_t *self)   // Soft reset display.
{
    write_command(self, ST7789_SWRESET);
    self->sleep_ms(150);
}

void st7789_sleep_mode(st7789_obj_t *self, bool value) //Enable(1) or disable(0) display sleep mode.
{
    if (value) {
        write_command(self, ST7789_SLPIN);
    } else {
        write_command(self, ST7789_SLPOUT);
    }
}
void st7789_inversion_mode(st7789_obj_t *self, bool value)  //Enable(1) or disable(0) display inversion mode.
{
    if (value) {
        write_command(self, ST7789_INVON);
    } else {
        write_command(self, ST7789_INVOFF);
    }
}

void st7789_set_color_mode(st7789_obj_t *self, uint8_t mode)  //Set display color mode.
{
    write_command(self, ST7789_COLMOD);
    write_data(self, &mode, 1);
}

void st7789_rotation(st7789_obj_t *self, int rotation)
{
    /*Set display rotation.
        Args:
            rotation (int):
                - 0-Portrait
                - 1-Landscape
                - 2-Inverted Portrait
                - 3-Inverted Landscape*/
    write_command(self, ST7789_MADCTL);
    uint8_t byte = 0;
    switch (rotation) {
        case 0:
            byte = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB;
            break;
        case 1:
            byte = ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB;
            break;
        case 2:
            byte = ST7789_MADCTL_RGB;
            break;
        case 3:
            byte = ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_BGR;
            break;
        default:
            break;
    }
    write_data(self, &byte, 1);
}

void st7789_set_columns(st7789_obj_t *self, int start, int end)
{
    start += self->x_offset;
    end += self->x_offset;
    uint8_t data[] = {start >> 8, start & 0xFF, end >> 8, end & 0xFF};
    write_command(self, ST7789_CASET);
    write_data(self, data, 4);
}
void st7789_set_rows(st7789_obj_t *self, int start, int end)
{
    start += self->y_offset;
    end += self->y_offset;
    uint8_t data[] = {start >> 8, start & 0xFF, end >> 8, end & 0xFF};
    write_command(self, ST7789_RASET);
    write_data(self, data, 4);
}

void st7789_set_window(st7789_obj_t *self, int x0, int y0, int x1, int y1)
{
    st7789_set_columns(self, x0, x1);
    st7789_set_rows(self, y0, y1);
    write_command(self, ST7789_RAMWR);
}


static void st7789_draw_pixel(st7789_obj_t *self, int x, int y, uint32_t color)
{
    //设置指定像素为指定颜色
    //Args: x (int): X轴坐标, Y (int): Y轴坐标, color (int): 565格式颜色
    st7789_set_window(self, x, y, x, y);
    uint8_t data[] = {color >> 8, (uint8_t)color};
    write_data(self, data, 2);
}

void st7789_draw_bitmap(st7789_obj_t *self, int x1, int y1, int x2, int y2, uint8_t *data)
{
    //复制缓冲区到屏幕的指定位置
    //Args:data (bytes): 要显示的数据
    //x (int): 缓冲区在屏幕位置的左上角X轴坐标, Y (int): 缓冲区在屏幕位置的左上角Y轴坐标
    st7789_set_window(self, x1, y1, x2, y2);
    write_data(self, data, 2 * ((x2 - x1) + 1) * ((y2 - y1) + 1));
}

void st7789_fill_rect(st7789_obj_t *self, int x, int y, int width, int height, uint32_t color)
{
    //Draw a rectangle at the given location, size and filled with color.
    //Args: x (int): Top left corner x coordinate, y (int): Top left corner y coordinate
    //width (int): Width in pixels, height (int): Height in pixels
    //color (int): 565 encoded color
    st7789_set_window(self, x, y, 30, 30);

    //uint16_t pixel16[1];
    //pixel16[0] = color;
    uint8_t pixel[2];
    pixel[0] = (color >> 8);
    pixel[1] = color & 0xff;

 //   div_t output;
 //   output = div(width *= height, 128);
//    int chunks = output.quot;

    self->set_dc_cs(1, 0); //Open to write
    size_t send_buffer = 128 * 2;
    uint8_t draw_pixel[send_buffer];

    for (int i = 0; i < send_buffer; i++) {
        draw_pixel[i] = pixel[i % 2];
    }

    for (int i = 128; i != 0; i--) {
        write_data(self, draw_pixel, send_buffer);
    }
}


void st7789_fill(st7789_obj_t *self, uint32_t color)
{
    uint16_t i;
    st7789_set_window(self, 0, 0, self->base.width - 1, self->base.height - 1);
    uint16_t j;
    for (i = 0; i < self->base.width; i++) {
        for (j = 0; j < self->base.height; j++) {
            uint8_t data[] = {color >> 8, color & 0xFF};
            write_data(self, data, 2);
        }
    }
}
void st7789_fill_16(st7789_obj_t *self, lv_color_t color)
{
    uint16_t i;
    st7789_set_window(self, 0, 0, self->base.width - 1, self->base.height - 1);
    uint16_t j;
		//printf("\r\nCOLOR: %0.4X\r\n", color.full); 
		//printf("CH_R: %0.2X\tCH_G:%0.2X\tCH_B:%0.2X\r\n", color.ch.red, color.ch.green, color.ch.blue);
    for (i = 0; i < self->base.width; i++) {
        for (j = 0; j < self->base.height; j++) {
            write_data(self, (uint8_t*)&color.full, 2);
        }
    }
}


void st7789_reset(st7789_obj_t *self)
{
    self->set_blacklight(0);
    st7789_hard_reset(self);
    st7789_soft_reset(self);
    st7789_sleep_mode(self, false);
    st7789_set_color_mode(self, COLOR_MODE_65K | COLOR_MODE_16BIT);
    st7789_rotation(self, 3);
    st7789_inversion_mode(self, true);

   // *** ST7789V Frame rate setting ***
   write_command(self, ST7789_PORCTRL);
   write_a_byte(self, 0x0c);
   write_a_byte(self, 0x0c);
   write_a_byte(self, 0x00);
   write_a_byte(self, 0x33);
   write_a_byte(self, 0x33);
 
   write_command(self, ST7789_GCTRL);  // Voltages: VGH / VGL
   write_a_byte(self, 0x35);
 
   // *** ST7789V Power setting ***
   write_command(self, ST7789_VCOMS);
   write_a_byte(self, 0x28);  // JLX240 display datasheet
 
   write_command(self, ST7789_LCMCTRL);
   write_a_byte(self, 0x0C);
 
//   write_command(self, ST7789_VDVVRHEN);
//   write_a_byte(self, 0x01);


   write_command(self, ST7789_VRHS);  // voltage VRHS
   write_a_byte(self, 0x10);
 
   write_command(self, ST7789_VDVSET);
   write_a_byte(self, 0x20);
 
   write_command(self, ST7789_FRCTRL2);
   write_a_byte(self, 0x0f);
 
   write_command(self, ST7789_PWCTRL1);
   write_a_byte(self, 0xa4);
   write_a_byte(self, 0xa1);
 
   // *** ST7789V gamma setting ***
   write_command(self, ST7789_PVGAMCTRL);
   write_a_byte(self, 0xd0);
   write_a_byte(self, 0x15);
   write_a_byte(self, 0x1A);
   write_a_byte(self, 0x0B);
   write_a_byte(self, 0x17);
   write_a_byte(self, 0x2A);
   write_a_byte(self, 0x3F);
   write_a_byte(self, 0x55);
   write_a_byte(self, 0x4C);
   write_a_byte(self, 0x2F);
   write_a_byte(self, 0x1F);
   write_a_byte(self, 0x1F);
   write_a_byte(self, 0x20);
   write_a_byte(self, 0x23);
 
   write_command(self, ST7789_NVGAMCTRL);
   write_a_byte(self, 0xd0);
   write_a_byte(self, 0x15);
   write_a_byte(self, 0x10);
   write_a_byte(self, 0x0B);
   write_a_byte(self, 0x17);
   write_a_byte(self, 0x2A);
   write_a_byte(self, 0x3F);
   write_a_byte(self, 0x44);
   write_a_byte(self, 0x51);
   write_a_byte(self, 0x2F);
   write_a_byte(self, 0x1F);
   write_a_byte(self, 0x1F);
   write_a_byte(self, 0x20);
   write_a_byte(self, 0x23);

    write_command(self, ST7789_NORON);
    st7789_fill_16(self, LV_COLOR_WHITE);
    write_command(self, ST7789_DISPON);
    self->set_blacklight(100);
    self->sleep_ms(50);

}

void st7789_display_on(st7789_obj_t *self)
{
    write_command(self, ST7789_DISPON);
}

void st7789_display_off(st7789_obj_t *self)
{
    write_command(self, ST7789_DISPOFF);
}

void st7789_set_blacklight(st7789_obj_t *self, uint32_t brightness)
{
    self->set_blacklight(brightness);
}

void st7789_obj_init(st7789_obj_t *self)
{
    self->base.draw_pixel = (void(*)(void*, int, int, uint32_t)) st7789_draw_pixel;
    self->base.draw_bitmap =  (void(*)(void*, int, int, int, int, uint8_t*))st7789_draw_bitmap;
    self->base.reset = (void(*)(void*))st7789_reset;
    self->base.display_on = (void(*)(void*))st7789_display_on;
    self->base.display_off =  (void(*)(void*))st7789_display_off;
    self->base.set_backlight =  (void(*)(void*, uint32_t))st7789_set_blacklight;

    self->base.reset(self);
}
