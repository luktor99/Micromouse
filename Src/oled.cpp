/*
 * oled.cpp
 *
 *  Created on: Aug 26, 2016
 *      Author: luktor99
 */

#include <oled.h>
#include <common.h>
#include <bitmaps/bitmap_battery.h>
#include <bitmaps/bitmap_bluetooth.h>

Menu::Menu(uint8_t menu_size) : max_pos(menu_size), menu_pos(0), cursor_pos(0), draw_pos(0), items_count(0) {
}

void Menu::set_menu(uint8_t parent) {
	current_parent = parent;
}

void Menu::add_text(const char *text) {
	item it = item(current_parent, text, NULL, 0);
	tree.push_back(it);
}

void Menu::add_func(const char *text, void (*func)(Menu *, uint8_t)) {
	item it = item(current_parent, text, func, 0);
	tree.push_back(it);
}

void Menu::add_goto(const char *text, uint8_t destination) {
	item it = item(current_parent, text, fgoto, destination);
	tree.push_back(it);
}

void Menu::handle(char key) {
        uint8_t redraw_menu=0;
        if(key==ENTER) {
                // check if handler function is declared and call it
                if(current_item->func != NULL) {
                        current_item->func(this, current_item->param);
                }
                redraw_menu=1;
        } else if(key==UP) {
                if(cursor_pos>0) cursor_pos--;
                redraw_menu=1;
        } else if(key==DOWN) {
                if(cursor_pos<items_count-1) cursor_pos++;
                redraw_menu=1;
        } else if(key==RIGHT) {
        } else if(key==LEFT) {
        }

        if(redraw_menu) draw();
}

void Menu::draw(void) {
		u8g_SetFont(&u8g, u8g_font_6x12);
		//u8g_SetFont(&u8g, u8g_font_profont12);
		u8g_SetFontRefHeightText(&u8g);
		u8g_SetFontPosTop(&u8g);
		font_h = u8g_GetFontAscent(&u8g)-u8g_GetFontDescent(&u8g);
		font_w = u8g_GetWidth(&u8g);

        // draw header
        //mvprintw(0, 0, "##### MENU #####");

        // count elements in the current (sub)menu
        items_count=count_items(menu_pos);

        // scroll the list if needed
        if(cursor_pos<draw_pos) draw_pos--;
        else if(cursor_pos>=draw_pos+max_pos) draw_pos++;

        // decide which element will be drawn as the last one
        uint8_t draw_end_pos;
        if(items_count <= max_pos) {
                draw_end_pos=items_count;
        } else if(draw_pos+max_pos > items_count) {
                draw_end_pos=items_count-max_pos;
        } else {
                draw_end_pos=draw_pos+max_pos;
        }

        // draw elements
        u8g_FirstPage(&u8g);
        do {
			uint8_t menu_i=0;
			for(uint8_t i=0; i<tree.size(); i++) {
					// check if the element is from the current (sub)menu
					if(tree[i].parent == menu_pos) {
							if(menu_i >= draw_pos) {
									if(cursor_pos==menu_i) { //highlight
											// save current element in case it's entered
											current_item=&tree[i];
											draw_element(menu_i-draw_pos+1, tree[i].text, 1);
									} else {
											draw_element(menu_i-draw_pos+1, tree[i].text, 0);
									}
							}

							menu_i++;

							// was it the last element to be drawn?
							if(menu_i==draw_end_pos) break;
					}
			}

        	// draw statusbar
        	draw_statusbar();
        } while (u8g_NextPage(&u8g));
}

uint8_t Menu::count_items(uint8_t parent) {
        uint8_t count=0;
        for(uint8_t i=0; i<tree.size(); i++) count+=(tree[i].parent==parent);
        return count;
}

void Menu::fgoto(Menu *m, uint8_t parent) {
        m->menu_pos=parent;
        m->draw_pos=0;
        m->cursor_pos=0;
}

void Menu::draw_element(uint8_t pos, const char *text, uint8_t highlight) {
	uint8_t width=u8g_GetStrWidth(&u8g, text);
	uint8_t d = (font_w-width)/2;
	u8g_SetDefaultForegroundColor(&u8g);
	if(highlight) {               // current selected menu item
		u8g_DrawBox(&u8g, 0, pos*font_h+1, font_w, font_h);     // draw cursor bar
		u8g_SetDefaultBackgroundColor(&u8g);
	}
	u8g_DrawStr(&u8g, d, pos*font_h, text);
}

void draw_statusbar(void) {
	char buffer[10];
	sprintf(buffer, "%u.%02uV", adc_lipo[0]/100, adc_lipo[0]%100);
	//u8g_SetFont(&u8g, u8g_font_baby);
	u8g_SetDefaultForegroundColor(&u8g);
	u8g_DrawStr(&u8g, 1, 0, "BluRay");
	u8g_DrawStr(&u8g, 97, 0, buffer);

	// battery icon
	u8g_DrawBitmap(&u8g, 82, 1, bitmap_battery_cnt, bitmap_battery_h, bitmap_battery);
	int8_t batt=(adc_lipo[0]-640)/27;
	u8g_DrawBox(&u8g, 84, 3, (batt>7)?7:((batt>0)?batt:0), 3);

	// Bluetooth icon
	if(BT_STATE_GPIO_Port->IDR & BT_STATE_Pin)
		u8g_DrawBitmap(&u8g, 73, 0, bitmap_bluetooth_cnt, bitmap_bluetooth_h, bitmap_bluetooth);

}
