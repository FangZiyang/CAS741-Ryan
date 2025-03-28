from scipy.ndimage import gaussian_filter1d
import numpy as np

def smooth_angle_pairs(theta1_list, theta2_list, sigma=1.2):
    """
    对角度序列 (theta1, theta2) 作为一对进行联合平滑。
    防止各自平滑后时间不同步带来的跳跃问题。
    """
    theta_array = np.array([theta1_list, theta2_list])
    smoothed = gaussian_filter1d(theta_array, sigma=sigma, axis=1)
    return smoothed[0], smoothed[1]
