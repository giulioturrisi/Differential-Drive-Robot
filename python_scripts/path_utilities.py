import numpy as np

import scipy.ndimage
from scipy import LowLevelCallable
from scipy.interpolate import CubicSpline
from numba import cfunc, carray
from numba.types import intc, intp, float64, voidptr
from numba.types import CPointer

import matplotlib.pyplot as plt

# Make cell occupied even if only one element is black ---------------------------------------
@cfunc(intc(CPointer(float64), intp, CPointer(float64), voidptr))
def map_filter_fn(values_ptr, len_values, result, user_data):
    values = carray(values_ptr, (len_values,), dtype=float64)
    
    filter = 254 
    for v in values:
        if(v == 0):
            filter = 0
            break
    result[0] = filter
    return 1


def filter_map(map, size):
    """Filtering the map to increae clearence with obstacles
    Args:
        map (np.array): multidimansional array representing the map
        size (int): size of the map
    Returns:
        (np.array): multidimansional array representing the filtered map
    """
    return scipy.ndimage.generic_filter(map, LowLevelCallable(map_filter_fn.ctypes), footprint=np.ones((size, size)), mode='constant', cval=np.nan,)



# Spline interpolation ---------------------------------------
def interpolate_path(path, dt):
    """Spline interpolation of the planned path
    Args:
        path (list): path found by the planned
        dt (float): sampling time
    Returns:
        (CubicSpline): interpolated spline
        (np.array): grid of points over the path
    """
    time = np.arange(0, len(path), 1)
    spline = CubicSpline(time,np.array(path))
    xs = np.arange(0, len(path), dt)
    return spline, xs



# Draw map for debug ---------------------------------------
def draw_map(map, state, goal, resolution):
    """Draw a given map for debug
    Args:
        map (np.array): multidimensional array containing the map
        start (np.array): actual state of the robot
        goal (np.array): desired goal 
        resolution (float): dimension of each cell
    """
    rgb_image = np.stack([map]*3, axis=2)/255.
    rgb_image[int(state[0]/resolution)][int(state[1]/resolution)] = [1,0,0]
    rgb_image[int(goal[0]/resolution)][int(goal[1]/resolution)] = [0,0,1]
    f = plt.figure() 
    plt.imshow(rgb_image)
    plt.show()
