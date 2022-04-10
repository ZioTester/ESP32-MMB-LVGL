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
#include "lvgl.h"

// possible tab position: LV_DIR_TOP - LV_DIR_BOTTOM - LV_DIR_LEFT - LV_DIR_RIGHT
#define TAB_DIR LV_DIR_TOP

// width size for left and right - height for top and bottom
#define TAB_SIZE 50

void lvLayoutSetup(void)
{
    /*Create a Tab view object*/
    lv_obj_t *tabview;
    tabview = lv_tabview_create(lv_scr_act(), TAB_DIR, TAB_SIZE);

    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Tab 1");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Tab 2");
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Tab 3");

    /*Add content to the tabs*/
    lv_obj_t * label = lv_label_create(tab1);
    lv_label_set_text(label, "ESP32 + LVGL Base Project\n"
                             "By ZioTester Lab\n\n"
                             "https://ziotester.github.io");

    label = lv_label_create(tab2);
    lv_label_set_text(label, "Second tab\n\n"
                             "If the content\n"
                             "of a tab\n"
                             "becomes too\n"
                             "longer\n"
                             "than the\n"
                             "container\n"
                             "then it\n"
                             "automatically\n"
                             "becomes\n"
                             "scrollable.\n"
                             "\n"
                             "\n"
                             "\n"
                             "Can you see it?");

    label = lv_label_create(tab3);
    lv_label_set_text(label, "Third tab");
}