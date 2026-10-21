"""Call the CMake/MSVC-built control DLL from the existing scenario harness."""
import ctypes as C
from dataclasses import fields
from pathlib import Path

import numpy as np

from .model import State, Vehicle


class NativeState(C.Structure):
    _fields_ = [(f.name, C.c_double) for f in fields(State)]


class NativeVehicle(C.Structure):
    _fields_ = [(f.name, C.c_double) for f in fields(Vehicle)]


class NativeBackend:
    name = "offline_cpp_rk4"

    def __init__(self, library=None):
        root = Path(__file__).resolve().parents[1]
        if library is None:
            candidates = [root / "build" / name / "bin" / "Release" / "icv_native.dll"
                          for name in ("vs2022", "vs2022-core")]
            library = next((p for p in candidates if p.is_file()), candidates[0])
        self.path = Path(library).resolve()
        if not self.path.is_file():
            raise FileNotFoundError(f"Build the C++ library first: cmake --preset vs2022; "
                                    f"cmake --build --preset vs2022-release (missing {self.path})")
        self.dll = C.CDLL(str(self.path))
        self.routes = {}
        state, vehicle, number = C.POINTER(NativeState), C.POINTER(NativeVehicle), C.POINTER(C.c_double)
        self.dll.icv_abi_version.restype = C.c_int
        if self.dll.icv_abi_version() != 1:
            raise RuntimeError("Unsupported native DLL ABI; rebuild the current source")
        self.dll.icv_last_error.restype = C.c_char_p
        self.dll.icv_route_create.argtypes = [number, C.c_size_t, C.c_int]
        self.dll.icv_route_create.restype = C.c_void_p
        self.dll.icv_route_destroy.argtypes = [C.c_void_p]
        self.dll.icv_route_destroy.restype = None
        self.dll.icv_steering.argtypes = [state, C.c_void_p, C.c_double, vehicle, number]
        self.dll.icv_speed.argtypes = [state, C.c_double, C.c_int, vehicle, C.c_int, number]
        self.dll.icv_obstacles.argtypes = [state, C.c_void_p, C.c_double, number, C.c_size_t,
                                           vehicle, number, C.POINTER(C.c_int), number]
        self.dll.icv_advance.argtypes = [state, C.c_double, C.c_double, C.c_double, vehicle, state]

    def _check(self, ok):
        if not ok:
            raise RuntimeError(self.dll.icv_last_error().decode("utf-8", errors="replace"))

    def _route(self, route):
        if route not in self.routes:
            points = np.ascontiguousarray(route.points[:, :2], dtype=np.float64)
            handle = self.dll.icv_route_create(points.ctypes.data_as(C.POINTER(C.c_double)),
                                               len(points), route.direction)
            self._check(handle)
            self.routes[route] = handle
        return self.routes[route]

    @staticmethod
    def _state(state):
        return NativeState(*(getattr(state, name) for name, _ in NativeState._fields_))

    @staticmethod
    def _vehicle(vehicle):
        return NativeVehicle(*(getattr(vehicle, name) for name, _ in NativeVehicle._fields_))

    def steering_control(self, state, route, progress, vehicle):
        output = C.c_double()
        self._check(self.dll.icv_steering(C.byref(self._state(state)), self._route(route), progress,
                                         C.byref(self._vehicle(vehicle)), C.byref(output)))
        return output.value

    def speed_control(self, state, desired, direction, vehicle, emergency=False):
        output = C.c_double()
        self._check(self.dll.icv_speed(C.byref(self._state(state)), desired, direction,
                                      C.byref(self._vehicle(vehicle)), emergency, C.byref(output)))
        return output.value

    def obstacle_speed(self, state, route, progress, obstacles, vehicle):
        objects = np.ascontiguousarray(obstacles, dtype=np.float64).reshape((-1, 6))
        limit, gap, emergency = C.c_double(), C.c_double(), C.c_int()
        self._check(self.dll.icv_obstacles(C.byref(self._state(state)), self._route(route), progress,
                      objects.ctypes.data_as(C.POINTER(C.c_double)), len(objects),
                      C.byref(self._vehicle(vehicle)), C.byref(limit), C.byref(emergency), C.byref(gap)))
        return limit.value, bool(emergency.value), gap.value

    def advance(self, state, acceleration, steering, dt, vehicle):
        output = NativeState()
        self._check(self.dll.icv_advance(C.byref(self._state(state)), acceleration, steering, dt,
                                        C.byref(self._vehicle(vehicle)), C.byref(output)))
        return State(*(getattr(output, name) for name, _ in NativeState._fields_))

    def close(self):
        for handle in self.routes.values():
            self.dll.icv_route_destroy(handle)
        self.routes.clear()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
