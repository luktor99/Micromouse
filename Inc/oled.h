/*
 * oled.h
 *
 *  Created on: Aug 26, 2016
 *      Author: luktor99
 */

#ifndef OLED_H_
#define OLED_H_

#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include "u8g_arm.h"

using namespace std;

#define MENU_ITEMS 6

void create_menu(void);
void draw_statusbar(void);

class Menu {
public:
	enum key_events {ENTER=0, UP, DOWN, LEFT, RIGHT} key_events;

	// menu item definition
	struct item {
			item(uint8_t parent, const char *text, void (*func)(Menu *, uint8_t), uint8_t param) : parent(parent), text(text), func(func), param(param) {}
			uint8_t parent;         // menu ID
	        const char *text;       // menu text
	        void (*func)(Menu *, uint8_t);  // handler function
	        uint8_t param;          // parameter passed to the function
	};

	// functions used to define the menu
	void set_menu(uint8_t parent);
	void add_text(const char *);
	void add_func(const char *, void (*)(Menu *, uint8_t));
	void add_goto(const char *, uint8_t);

	// menu drawing
	void draw(void);
	// key press handling
	void handle(char key);

	static void fgoto(Menu *, uint8_t);

	// default constructor
	Menu(uint8_t);

	// pointer to the currently selected item
	item *current_item;

//private:
	uint8_t count_items(uint8_t parent);

	// tree that holds menu structure
	vector<item> tree;
	// menu drawing positions
	uint8_t menu_pos, cursor_pos, draw_pos, items_count;

	// menu size (number of displayed items)
	uint8_t max_pos;

	uint8_t current_parent;
	void draw_element(uint8_t pos, const char *text, uint8_t highlight);
	uint8_t font_w, font_h;
};

extern Menu menu;

#endif /* OLEDMENU_H_ */
