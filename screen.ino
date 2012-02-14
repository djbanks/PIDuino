#include "PCD8544.h"

int disp_clk = 7;   //Serial clock out (SCLK)
int disp_d_c = 6;   //Data/Command select (D/C)
int disp_led = 5;   // LED Backlight
int disp_din = 4;   // Serial data out (DIN)

PCD8544 nokia = PCD8544(disp_clk, disp_din, disp_d_c, disp_ncs, disp_rst);

static const byte numbers_1[] = {0xc0, 0xf0, 0x18, 0x18, 0x18}
static const byte numbers_8[] = {0x00, 0x70, 0xf8, 0xdc, 0x8c,
                                0x8c, 0x8c, 0xf8, 0x70, 0x00,
                                0x1e, 0x3f, 0x7f, 0x71, 0x61,
                                0x61, 0x61, 0x7f, 0x3f, 0x1e}
                                
void init()
{
  
  font[
  
  nokia.init();
  nokia.setContrast(50);
  nokia.clear();
  nokia.setPixel(10, 10, BLACK);
  nokia.display();
  
}
