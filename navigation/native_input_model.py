"""Simulation adapter to the exact production C++ SpeedResponse input ramp.

Used only by a synthetic plant. Real motor/receiver interfaces are absent.
"""
import ctypes
from pathlib import Path

class NativeInputModel:
    def __init__(self, library=None):
        if library is None:
            from ament_index_python.packages import get_package_prefix
            library=Path(get_package_prefix('p2_nav_controller'))/'lib/libp2_nav_input_model.so'
        self.lib=ctypes.CDLL(str(library))
        self.lib.p2_input_model_abi.restype=ctypes.c_int
        if self.lib.p2_input_model_abi()!=1:raise ValueError('native_input_model_abi_mismatch')
        self.lib.p2_input_model_create.restype=ctypes.c_void_p
        self.lib.p2_input_model_destroy.argtypes=[ctypes.c_void_p]
        self.lib.p2_input_model_step.argtypes=[ctypes.c_void_p,*([ctypes.c_double]*5),ctypes.c_ulonglong,ctypes.c_ulonglong,ctypes.c_int,ctypes.POINTER(ctypes.c_double)]
        self.lib.p2_input_model_step.restype=ctypes.c_uint
        self.handle=self.lib.p2_input_model_create()
        if not self.handle:raise MemoryError('native_input_model_create')
        self.reason=0
    def step(self, target, measured, dt, source_ns, now_ns, owned=True):
        out=(ctypes.c_double*2)()
        self.reason=self.lib.p2_input_model_step(self.handle,*target,*measured,dt,source_ns,now_ns,int(owned),out)
        return tuple(out)
    def close(self):
        if self.handle:self.lib.p2_input_model_destroy(self.handle);self.handle=None
    def __del__(self):
        if getattr(self,'handle',None):self.close()
