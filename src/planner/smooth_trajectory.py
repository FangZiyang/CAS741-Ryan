from scipy.ndimage import gaussian_filter1d

def smooth_trajectory(theta_list, sigma=1.2):
    """
    对角度序列进行高斯滤波平滑处理。

    参数：
        theta_list (list or np.array): 角度序列（单位：弧度）
        sigma (float): 高斯滤波的标准差，sigma 越大，平滑效果越明显

    返回：
        np.array: 平滑处理后的角度序列
    """


    return gaussian_filter1d(theta_list, sigma=sigma)
