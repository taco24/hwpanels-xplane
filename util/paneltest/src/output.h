/*
 * output.h
 *
 *  Created on: Mar 4, 2011
 *      Author: max
 */
#ifndef OUTPUT_H_
#define OUTPUT_H_

#if IBM
#include <curses.h>			/* ncurses.h includes stdio.h */
#else
#include <ncurses.h>			/* ncurses.h includes stdio.h */
#endif


void initConsole();

void closeConsole();

void writeConsole(int row, int col, const char *text);

#endif /* OUTPUT_H_ */

