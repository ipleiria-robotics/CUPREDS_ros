/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*/

#ifndef UTILS_H_
#define UTILS_H_

#include "ros/ros.h"
#include <chrono>

class Utils {
    public:
        static long long getCurrentTimeMillis();
        static long getAgeInSecs(long long timestamp);
        static long long getMaxTimestampForAge(long age);
};

#endif