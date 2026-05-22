#pragma once


#define user_value_limit(val, min, max)   \
        {                                \
            (val) = (val) < (min) ? (min) : (val); \
            (val) = (val) > (max) ? (max) : (val); \
        }
        
float user_val_limit(float val, float min, float max);