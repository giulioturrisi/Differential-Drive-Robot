import numpy as np
import scipy.ndimage
from scipy import LowLevelCallable
from numba import cfunc, carray
from numba.types import intc, intp, float64, voidptr
from numba.types import CPointer

@cfunc(intc(CPointer(float64), intp, CPointer(float64), voidptr))
def map_filter_fn(values_ptr, len_values, result, user_data):
    # Get values from pointer
    values = carray(values_ptr, (len_values,), dtype=float64)

    slopes_sum = 0
    slopes_sum_square = 0
    n_elems = 0
    mid_idx = int(np.floor(len_values / 2.))
    mid_value = values[mid_idx]

    filter_out = True

    for v in zip(values):
        if not np.isnan(v):
            if(v == 0):
                filter_out = True
                break


    result[0] = filter_out

    return 1


def filter_map(map):
        edge = scipy.ndimage.generic_filter(
            map,
            LowLevelCallable(map_filter_fn.ctypes),
            footprint=self._footprint, mode='constant', cval=np.nan,
        )
        slope_bools = edge <= self.inclination_threshold