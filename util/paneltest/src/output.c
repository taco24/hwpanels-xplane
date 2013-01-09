/*
 * output.c
 *
 *  Created on: Mar 4, 2011
 *      Author: max
 */
#include "output.h"

void initConsole() {
	initscr(); /* Start curses mode 		  */
	printw("Press 'q' to quit."); /* Print Hello World		  */
	refresh(); /* Print it on to the real screen */

}

void closeConsole() {
	endwin(); /* End curses mode		  */
}

void writeConsole(int row, int col, const char *text) {
	mvprintw(row, 0, "                                       ");
	mvprintw(row, col, "%s", text);
	refresh();
}


