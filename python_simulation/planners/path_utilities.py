import numpy as np
import scipy.ndimage
from scipy import LowLevelCallable
from numba import cfunc, carray
from numba.types import intc, intp, float64, voidptr
from numba.types import CPointer

from scipy.interpolate import CubicSpline

import matplotlib.pyplot as plt

@cfunc(intc(CPointer(float64), intp, CPointer(float64), voidptr))
def map_filter_fn(values_ptr, len_values, result, user_data):
    # Get values from pointer
    values = carray(values_ptr, (len_values,), dtype=float64)

    filter_out = True

    for v in zip(values):
        if not np.isnan(v):
            if(v == 0):
                filter_out = True
                break


    result[0] = filter_out
    return 1


def filter_map(map, size):
        map_filtered = scipy.ndimage.generic_filter(
            map,
            LowLevelCallable(map_filter_fn.ctypes),
            footprint=np.ones((size,size)), mode='constant', cval=np.nan,
        )
        #slope_bools = edge <= self.inclination_threshold


def interpolate_path(path, dt):
    time = np.arange(0, len(path), 1)
    spline = CubicSpline(time,np.array(path))
    xs = np.arange(0, len(path), dt)
    return spline, xs


def draw_map(map, state, goal, resolution):
    rgb_image = np.stack([map]*3, axis=2)/255.
    rgb_image[int(state[0]/resolution)][int(state[1]/resolution)] = [1,0,0]
    rgb_image[int(goal[0]/resolution)][int(goal[1]/resolution)] = [0,0,1]
    f = plt.figure() 
    plt.imshow(rgb_image)
    plt.show()
