#ifndef MIX_H
#define MIX_H

// args and return val are pwm pulsewidth
int map_right_top(int arl, int ele, int rud, int brk);
int map_right_bottom(int arl, int ele, int rud, int brk);
int map_left_top(int arl, int ele, int rud, int brk);
int map_left_bottom(int arl, int ele, int rud, int brk);

#endif