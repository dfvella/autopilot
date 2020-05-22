#include "mix.h"

int map_right_top(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return 3000 - arl - ele - rud + brk;
}

int map_right_bottom(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return arl + ele - rud + brk;
}

int map_left_top(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return 1500 - ((arl - ele + rud) - 1500) - brk;
}

int map_left_bottom(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return arl - ele - rud - brk;
}