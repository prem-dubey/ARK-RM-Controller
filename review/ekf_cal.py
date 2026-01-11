# imu_data.py
from dataclasses import dataclass
from typing import Tuple , Optional
import numpy as np

@dataclass
class ImuData:
    accel: Tuple[float, float, float]
    gyro: Tuple[float, float, float]
    accel_dt: float
    gyro_dt: float
    timestamp: float
    accelerometer_clipping: int
    gyro_clipping: int



def ekf_step(x , p_var , imu , alt):
    # x = roll , pitch , yaw_rate , altitude , vertical_velocity
    # p -> roll rate and q -> pitch rate
    gyro_dt = imu.gyro_dt
    accel_dt = imu.accel_dt
    # x[0]
    phi = x[0]
    p = imu.gyro[0]
    # x[1]
    theta = x[1]
    q = imu.gyro[1]
    # x[2]
    r = x[2]
    # x[3]
    z = x[3]
    # x[4]
    vz = x[4]
    az = imu.accel[2]

    g = 9.81


    x_predicted = np.array([
        phi + p * gyro_dt,
        theta + q * gyro_dt,
        r,
        z + vz * accel_dt,
        vz + (az + g) * accel_dt
    ])

    F = np.array([
        [1 , 0 , 0 , 0 , 0],
        [0 , 1 , 0 , 0 , 0],
        [0 , 0 , 1 , 0 , 0],
        [0 , 0 , 0 , 1 , accel_dt],
        [0 , 0 , 0 , 0 , 1]
    ])

    Q = np.array([
        [(0.002 * 0.002)*accel_dt , 0 , 0 , 0 , 0],
        [0 , (0.002 * 0.002)*accel_dt , 0 , 0 , 0],
        [0 , 0 , (0.01 * 0.01)*accel_dt , 0 , 0],
        [0 , 0 , 0 , (0.05 * 0.05)*accel_dt*accel_dt , 0],
        [0 , 0 , 0 , 0 , (0.2 * 0.2)*accel_dt],
    ])

    # Covariance Predictin (Important)
    p_pred_var =  F@p_var@F.T + Q

    # Now the prediction part is over and we are goin for the
    # Accelerometer Update 
    z_acc = np.array([
        [imu.accel[0]],
        [imu.accel[1]]
    ])
    # using the newly updated or predicted angles for calculation
    phi = x_predicted[0]
    theta = x_predicted[1]

    h_acc = np.array([
        [g * np.sin(theta)],
        [-g * np.sin(phi)]
    ])

    y_acc = z_acc - h_acc

    H_acc = np.array([
        [0, g * np.cos(theta), 0, 0, 0],
        [-g * np.cos(phi), 0, 0, 0, 0]
    ])

    R_acc = np.array([
        [0.25 , 0],
        [0 , 0.25]
    ])

    # Innovation Covariance 
    S_acc = H_acc@p_pred_var@H_acc.T + R_acc

    # Kalmann Gain the most important one 
    K_acc = p_pred_var@H_acc.T@np.linalg.inv(S_acc)

    # writing Identity matrix 
    I = np.identity(5)

    x_new = x_predicted + (K_acc @ y_acc).flatten()
    p_new = (I -K_acc@H_acc)@p_pred_var

    # Writing the code for updating r specially 
    # gyro measurement
    r_meas = imu.gyro[2]

    # predicted measurement
    r_pred = x_predicted[2]

    # residual
    y_r = np.array([[r_meas - r_pred]])

    H_r = np.array([[0, 0, 1, 0, 0]])
    R_r = np.array([[0.01**2]])

    S = H_r @ p_new @ H_r.T + R_r
    K = p_new @ H_r.T @ np.linalg.inv(S)

    x_new = x_new + (K @ y_r).flatten()
    p_new = (I - K @ H_r) @ p_new

    # Till now everything was based on the IMU data means only of roll and pitch where correct 
    # Now we will also be updating everything based on the altimeter data when it arrives
    if alt is not None:
        z_meas, vz_meas, z_reset = alt
        h_alt = x_new[3]
        y_alt = np.array([[z_meas - h_alt]])
        H_alt = np.array([
            [0 , 0 , 0 , 1 , 0]
        ])
        R_alt = np.array([[0.3**2]])

        S_alt = H_alt @ p_new @ H_alt.T + R_alt
        K_alt = p_new @ H_alt.T @ np.linalg.inv(S_alt)

        x_new = x_new + (K_alt @ y_alt).flatten()
        p_new = (I - K_alt@H_alt)@p_new

    return x_new , p_new


