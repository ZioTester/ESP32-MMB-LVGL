////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//////
////// Info, guides and tutorials on:
////// https://ziotester.github.io
//////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <Arduino.h>
#include "TFT_eSPI.h"
#include <esp_task_wdt.h>
#include "lvgl.h"

#if LAYOUT_TABVIEW
  #include "gui_layouts/tabview.h"
#endif
#if LAYOUT_BUTTON_MATRIX
  #include "gui_layouts/button_matrix.h"
#endif

TFT_eSPI tft = TFT_eSPI();

/*

debug

*/
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

scrive l'immagine nel buffer sul display

*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void displayFlush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

gestisce le coordinate del touchscreen

*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void touchscreen_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data ) {
  uint16_t touchX, touchY;
  bool touched = tft.getTouch( &touchX, &touchY, 600);
  if( !touched )
  {
      data->state = LV_INDEV_STATE_REL;
  }
  else
  {
      data->state = LV_INDEV_STATE_PR;
      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;
      /*
      Serial.print( "Data x " );
      Serial.println( touchX );

      Serial.print( "Data y " );
      Serial.println( touchY );
      */
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

setup del buffer, del display, dell'input, del tema

*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lvSetup(){
  /*

  init

  */
  lv_init();

  #if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
  #endif

  /*

  setup del buffer

  */
  static lv_disp_draw_buf_t display_buffer;
  static lv_color_t image_buffer[MONITOR_HOR_RES  * MONITOR_VER_RES / 10];
  lv_disp_draw_buf_init(&display_buffer, image_buffer, NULL, MONITOR_HOR_RES  * MONITOR_VER_RES / 10);

  /*

  setup del dislpay

  */
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);  
  disp_drv.hor_res = MONITOR_HOR_RES;
  disp_drv.ver_res = MONITOR_VER_RES;
  disp_drv.flush_cb = displayFlush;
  disp_drv.draw_buf = &display_buffer;
  lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

  /*

  setup dell'input (touchscreen)

  */
  static lv_indev_drv_t indev_drv;           /*Descriptor of a input device driver*/
  lv_indev_drv_init(&indev_drv);             /*Basic initialization*/
  indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
  indev_drv.read_cb = touchscreen_read;      /*Set your driver function*/
  lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

  /*

  Setup del tema

  */
  //lv_theme_t * th = lv_theme_basic_init(disp);
  lv_theme_t * th = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, LV_FONT_DEFAULT);
  lv_disp_set_theme(disp, th);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

Task separato dal resto dell'applicazione per la gestione della UI.

1) Setup della libreria TFT_eSPI
2) Setup LVGL
3) lvTask loop - loop infinito che va in parallelo con il main loop();

*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void guiTask(void *pvParameters) {
  /*

  setup TFT_eSPI

  */
  tft.begin();
  tft.setRotation(MMB_ROTATION);
  ledcAttachPin(TFT_BL, 1);      // assign TFT_BL pin to channel 1
  ledcSetup(1, 12000, 8);        // 12 kHz PWM, 8-bit resolution
  ledcWrite(1, TFT_BRIGHTNESS);  // brightness 0 - 255

  /*

  valori di calibrazione del touchscreen
  se il touch non funziona con queste impostazioni, è necessaria la calibrazione

  */
  // orizzontale  
  //uint16_t calData[5] = { 484, 3387, 329, 3256, 1 };
  uint16_t calData[5] = { 403, 3442, 419, 3251, 1 };
  // verticale
  //uint16_t calData[5] = { 295, 3493, 320, 3602, 2 };
  // orizzontale invertito
  //uint16_t calData[5] = {418, 3404, 334, 3324, 7};  

  tft.setTouch(calData);

  /*

  lvgl setup
 
  */
  lvSetup();
  
  /*

  disegno il layout scelto da src/gui_layouts/)

  */
  lvLayoutSetup();
  
  /*

  loop infinito parallelo

  */
  while (1) {
    lv_task_handler();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////