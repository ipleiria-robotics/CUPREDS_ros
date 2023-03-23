/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*/

#include "Utils.h"

long long Utils::getCurrentTimeMillis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

long Utils::getAgeInSecs(long long timestamp) {
    return (Utils::getCurrentTimeMillis() - timestamp) / 1000;
}

long long Utils::getMaxTimestampForAge(long age) {
    return Utils::getCurrentTimeMillis() - (age * 1000);
}