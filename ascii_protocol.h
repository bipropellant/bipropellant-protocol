/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once


#include "protocol.h"
// used in main_ascii_init, external to this file`
extern int enable_immediate;

extern void ascii_add_immediate( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char byte,  char *ascii_out), char* description );
extern void ascii_add_line_fn( unsigned char letter, int (*fn)(PROTOCOL_STAT *s, char *line, char *ascii_out), char *description );
extern int ascii_init();
