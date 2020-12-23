/*
* 均值滤波器
* 蒋晓岗<kerndev@foxmail.com>
*/
#include "filter.h"

//均值滤波器
int filter_input(filter_t *flt, int value)
{
    flt->sum += value;
    flt->sum -= flt->buff[flt->index];
    flt->buff[flt->index] = value;
    flt->index = (flt->index + 1) & (flt->order - 1);
    return (flt->sum >> flt->shift);
}

//均值滤波器
void filter_init(filter_t *flt, int order)
{
    flt->index = 0;
    flt->sum   = 0;
    flt->order = order;
    flt->shift = 0;
    while(order > 1)
    {
        flt->shift++;
        order >>= 1;
    }
}
