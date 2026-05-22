#include "algorithm_lib/user_maths.hpp"


float user_val_limit(float val, float min, float max)
{
    (val) = (val) < (min) ? (min) : (val);
    (val) = (val) > (max) ? (max) : (val);
    return val;
}
