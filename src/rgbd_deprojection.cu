/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* These are the CUDA kernel and functions used to deproject the RGBD image
* to a pointcloud with better performance and higher floating point precision.
*/

#include <cuda_runtime.h>

// TODO: create global function to call from the host

// TODO: create the deprojection kernel, which is a device function