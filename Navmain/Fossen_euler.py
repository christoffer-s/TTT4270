import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.linalg import expm
import skew as sk


# % Inputs:
# %   x_ins[k] : INS state vector at step k, includes position, velocity,
# %              accelerometer biases, attitude (Euler angles), and gyro biases.
# %   P_prd[k] : 15x15 covariance matrix of the prediction step.
# %   
# %   h        : Sampling time in seconds.
# %   Qd, Rd   : Process and measurement noise covariance matrices for the
# %              Kalman filter.
# %   f_imu[k] : Specific force measurements from the IMU.
# %   w_imu[k] : Angular rate measurements from the IMU.
# %   
# %   y_pos[k] : Slow position measurements aids the filter.
# %   y_vel[k] : (Optionally) Slow velocity measurements aids the filter.

# % Outputs:
# %   x_ins[k+1] - Updated INS state vector after propagation.
# %   P_prd[k+1] - Updated prediction covariance matrix after propagation.



def updateKalmanFilter(x_ins, P_prd, h, Qd, Rd, f_imu, w_imu, y_pos=None, y_vel=None):
    T_acc = 1000
    T_ars = 500 # Dobbelt sjekk disse

    # ESKF states and matrices
    p_ins = x_ins[0]
    v_ins = x_ins[1]
    b_acc_ins = x_ins[2]
    theta_ins = x_ins[3]
    b_ars_ins = x_ins[4]

    # Gravity vector
    g_n = np.array([0,0, 9.81]).T

    # Constants
    O3 = np.zeros((3,3))
    I3 = np.eye(3)

    # Rotation matrix
    r = Rot.from_euler('zyx', [theta_ins[0],theta_ins[1],theta_ins[2]], degrees=False)
    R = np.array(r.as_matrix())

    # Bias compensated IMU measurements
    f_ins = f_imu - b_acc_ins
    w_ins = w_imu - b_ars_ins

    # Normalized specific force
    v01 = np.array([0,0,-1]).T
    v1 = f_ins/np.linalg.norm(f_ins)

    A = np.block([[O3, I3, O3, O3, O3],
         [O3, O3, -R, -R @ sk.skew(f_ins), O3],
         [O3, O3, -(1/T_acc)*I3, O3, O3],
         [O3, O3, O3, -sk.skew(w_ins), -I3],
         [O3, O3, O3, O3, -(1/T_ars)*I3]])
    
    Ad = expm(A * h)

    if all(y_pos) != None:
        Cd = np.block([[I3, O3, O3, O3, O3],
              [O3, O3, O3, sk.skew(R.T@v01), O3]]) 
        Cd = np.vstack((Cd, [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0]))
        print("CD-"*30)
        print(Cd)
        print("CD-"*30)
    else:
        Cd = np.block([[I3, O3, O3, O3, O3],
              [O3, I3, O3, O3, O3],
              [O3, O3, O3, sk.skew(R.T@v01), O3]]) 
        Cd = np.vstack((Cd, [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0]))
    
    Ed = h * np.block([[O3, O3, O3, O3],
              [-R, O3, O3, O3],
              [O3, I3, O3, O3],
              [O3, O3, -I3, O3],
              [O3, O3, O3, I3]])
    
    # Kalman filter algorithm
    if all(y_pos) == None:
        P_hat = P_prd
    else:
        # EKSF gain: K[k]
        K = P_prd @ Cd.T @ np.linalg.inv(Cd @ P_prd @ Cd.T + Rd)
        print("K-"*30)
        print(K)
        print("K-"*30)
        IKC = np.eye(15) - K @ Cd

        # Estimate error: eps[k]
        eps_pos = y_pos - p_ins
        eps_g = v1 - R.T @ v01
        eps_psi = np.arctan2(v_ins[1],v_ins[0]) # ssa = arctan2

        if all(y_pos) != None:
            eps = np.hstack([eps_pos, eps_g, eps_psi])
            print(eps)
        else:
            eps_vel = y_vel - v_ins
            eps = np.hstack([eps_pos, eps_vel, eps_g, eps_psi])
            print(eps)

        # Corrector: delta_x_hat[k] and P_hat[k]
        delta_x_hat = K @ eps
        P_hat = IKC @ P_prd @ IKC + K @ Rd @ K.T

        # INS reset: x_ins[k]
        
        p_ins = p_ins + delta_x_hat(0);	         # Reset INS position
        v_ins = v_ins + delta_x_hat(1);			 # Reset INS velocity
        b_acc_ins = b_acc_ins + delta_x_hat(2);  # Reset ACC bias
        theta_ins = theta_ins + delta_x_hat(3);  # Reset INS attitude
        b_ars_ins = b_ars_ins + delta_x_hat(4);  # Reset ARS bias

    # Predictor: P_prd[k+1]
    P_prd = Ad * P_hat * Ad.T + Ed * Qd * Ed.T

    # INS propagation: x_ins[k+1]
    a_ins = R * f_ins + g_n
    p_ins = p_ins + h * v_ins + h**2/2 * a_ins
    v_ins = v_ins + h * a_ins
    theta_ins = theta_ins + h * (Rot.from_euler('zyx',[theta_ins[0],theta_ins[1]]) @ w_ins)

    x_ins = [p_ins, v_ins, b_acc_ins, theta_ins, b_ars_ins]

# How to initialize ins
# p_ins = np.array([0, 0, 0]).T
# v_ins = np.array([0, 0, 0]).T
# b_acc_ins = np.array([0, 0, 0]).T
# theta_ins = np.array([0, 0, 0]).T
# b_ars_ins = np.array([0, 0, 0]).T
# x_ins = [p_ins, v_ins, b_acc_ins, theta_ins, b_ars_ins]

