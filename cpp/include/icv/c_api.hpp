#pragma once
#include "icv/control.hpp"
#include <cstddef>

#if defined(_WIN32)
#if defined(ICV_NATIVE_EXPORTS)
#define ICV_API __declspec(dllexport)
#else
#define ICV_API __declspec(dllimport)
#endif
#else
#define ICV_API __attribute__((visibility("default")))
#endif

extern "C" {
ICV_API int icv_abi_version();
ICV_API const char* icv_last_error();
ICV_API void* icv_route_create(const double* xy, std::size_t count, int direction);
ICV_API void icv_route_destroy(void* route);
ICV_API int icv_steering(const icv::State*, const void* route, double progress, const icv::Vehicle*, double* output);
ICV_API int icv_speed(const icv::State*, double desired, int direction, const icv::Vehicle*, int emergency, double* output);
ICV_API int icv_obstacles(const icv::State*, const void* route, double progress, const double* obstacles,
                          std::size_t count, const icv::Vehicle*, double* limit, int* emergency, double* gap);
ICV_API int icv_advance(const icv::State*, double acceleration, double steering, double dt, const icv::Vehicle*, icv::State* output);
}
