#include "mix.h"

int Mix::right_top(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return 3000 - arl - ele - rud + brk;
}

int Mix::right_bottom(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return arl + ele - rud + brk;
}

int Mix::left_top(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return 1500 - ((arl - ele + rud) - 1500) - brk;
}

int Mix::left_bottom(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return arl - ele - rud - brk;
}