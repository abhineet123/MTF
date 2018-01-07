#pragma once
#include <OS_specific.h>


#if OS_type==2
#define round(x) ( (x) >= 0 ? (x)+0.5 : (x)-0.5 )
#define min(x,y) (x > y ? y : x)
#define max(x,y) (x > y ? x : y)

#define snprintf _snprintf
#endif
#define sign(x) (((x)>=0)? 1.0f : -1.0f)
#define FLOAT_MAX 3.4e38f
#define FLOAT_MIN 1.2e-38f

