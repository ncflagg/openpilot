from cffi import FFI
import subprocess
try:
    subprocess.check_call(["make", "-j4"], cwd="/data/openpilot/selfdrive/virtualZSS")
except:
    pass

def get_wrapper():  # initialize df model and process angle predictions
    libmpc_fn = "/data/openpilot/selfdrive/virtualZSS/virtualZSS.so"

    ffi = FFI()
    ffi.cdef("""
    float run_model(float shitty_angle, float t_output);
    void init_model();
    float run_model_time_series(float inputData[40]);
    """)

    return ffi.dlopen(libmpc_fn)
