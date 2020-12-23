#ifndef __FILTER_H
#define __FILTER_H

struct filter
{
    int buff[8];
    int order;
    int shift;
    int sum;
    int index;
};

typedef struct filter filter_t;

void filter_init(filter_t *flt, int order);
int  filter_input(filter_t *flt, int value);

#endif
