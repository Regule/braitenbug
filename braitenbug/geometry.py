import numpy as np
import numpy.typing as npt


def polar_to_cartesian_matrix(r: npt.NDArray[np.float64],
                              theta: npt.NDArray[np.float64]
                              )-> npt.NDArray[np.float64]:    
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    
    cartesian_matrix = np.vstack((x, y))
    return cartesian_matrix

def angle_ros2_to_pygame(angle: float)-> float:
    return np.pi + np.pi/2 - angle
