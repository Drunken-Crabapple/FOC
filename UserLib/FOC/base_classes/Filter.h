#ifndef FILTER_H
#define FILTER_H

typedef struct Filter Filter_t;

struct Filter
{
    float (*apply)(Filter_t *filter,float value);   //调用二阶低通函数
    float (*get_ts)(Filter_t *filter);

    void *context;
};

#endif
