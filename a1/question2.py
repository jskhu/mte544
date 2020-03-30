import numpy as np
import math
import matplotlib.pyplot as plt

mu0 = np.array([0, 0.1, 2 * np.pi / 180, 30 * np.pi / 180])
l = 1
dt = 0.1
t_max = 10
ts = np.arange(0, t_max + dt, dt)
process_noise = np.diag([0.0005, 0.0005, 0.0001, 0.0001])
meas_noise = np.diag([0.1, 0.3])

def get_range(x, y):
    return np.sqrt(x**2 + y**2)

def get_angle(x, y):
    return math.atan2(y, x)

def three_wheel_robot(mu0, S0, ts, Q, R, process_noise, meas_noise):
    np.random.seed(23)
    # [x, y, delta, theta]^T
    mu = []
    S = []
    truth = []
    truth.append(mu0)
    mu.append(mu0)
    S.append(S0)

    for step, t in enumerate(ts):
        if step == 0:
            continue

        # Get control input
        v = 10 * np.sin(t)
        wd = 0.01

        update = dt * np.array([v * np.cos(truth[step-1][3]),
                                v * np.sin(truth[step-1][3]),
                                wd,
                                (v / l) * np.tan(truth[step-1][2] + dt * wd)])

        truth.append(truth[step-1] + update + np.sqrt(np.diag(process_noise)) * np.random.randn(4))
        

        # Construct the G matrix
        G = np.eye(4)
        G[0][3] = -dt * v * np.sin(mu[step-1][3])
        G[1][3] = dt * v * np.cos(mu[step-1][3])
        G[3][2] = (dt * v / l) / (np.cos(mu[step-1][2] + dt*wd))**2

        # Setup initial prediction
        mup = mu[step-1] + dt * np.array([v * np.cos(mu[step-1][3]),
                                          v * np.sin(mu[step-1][3]),
                                          wd,
                                          (v / l) * np.tan(mu[step-1][2] + dt * wd)])
        Sp = G @ S[step-1] @ G.transpose() + R

        # Get measurement
        z = np.zeros(2)
        z[0] = get_range(truth[step][0], truth[step][1]) + np.sqrt(meas_noise[0][0]) * np.random.randn()
        z[1] = get_angle(truth[step][0], truth[step][1]) + np.sqrt(meas_noise[1][1]) * np.random.randn()

        # Construct the H matrix
        sqrxy = mup[0]**2 + mup[1]**2

        H = np.array([[mup[0] / np.sqrt(sqrxy), mup[1] / np.sqrt(sqrxy), 0, 0],
                    [-mup[1] / sqrxy, mup[0] / sqrxy, 0, 0]])

        # Get Kalman gain
        K = Sp @ H.transpose() @ np.linalg.inv(H @ Sp @ H.transpose() + Q)

        mu.append(mup + K @ (z - np.array([get_range(mup[0], mup[1]),
                                           get_angle(mup[0], mup[1])])))
        S.append((np.eye(4) - K @ H) @ Sp)

    return truth, mu, S


if __name__ == "__main__":
    Q1 = np.diag([0.1, 0.3])
    Q2 = np.diag([0.2, 0.3])
    Q3 = np.diag([0.3, 0.3])
    Q4 = np.diag([0.1, 0.4])
    Q5 = np.diag([0.1, 0.5])
    S0 = 0.5 * np.eye(4)
    process_noise = np.diag([0.003, 0.003, 0.003, 0.003])
    meas_noise = np.diag([0.1, 0.3])

    R1 = np.diag([0.0001, 0.0001, 0.0001, 0.0001])
    R2 = np.diag([0.001, 0.001, 0.001, 0.001])
    R3 = np.diag([0.003, 0.003, 0.003, 0.003])
    R4 = np.diag([0.005, 0.005, 0.005, 0.005])
    R5 = np.diag([0.007, 0.007, 0.007, 0.007])

    truth_Q1_R1, mu_Q1_R1, S_Q1_R1 = three_wheel_robot(mu0, S0, ts, meas_noise, R1, process_noise, meas_noise)
    truth_Q1_R2, mu_Q1_R2, S_Q1_R2 = three_wheel_robot(mu0, S0, ts, meas_noise, R2, process_noise, meas_noise)
    truth_Q1_R3, mu_Q1_R3, S_Q1_R3 = three_wheel_robot(mu0, S0, ts, meas_noise, R3, process_noise, meas_noise)
    truth_Q1_R4, mu_Q1_R4, S_Q1_R4 = three_wheel_robot(mu0, S0, ts, meas_noise, R4, process_noise, meas_noise)
    truth_Q1_R5, mu_Q1_R5, S_Q1_R5 = three_wheel_robot(mu0, S0, ts, meas_noise, R5, process_noise, meas_noise)

    truth_Q2_R2, mu_Q2_R2, S_Q2_R2 = three_wheel_robot(mu0, S0, ts, Q2, process_noise, process_noise, meas_noise)
    truth_Q3_R2, mu_Q3_R2, S_Q3_R2 = three_wheel_robot(mu0, S0, ts, Q3, process_noise, process_noise, meas_noise)
    truth_Q4_R2, mu_Q4_R2, S_Q4_R2 = three_wheel_robot(mu0, S0, ts, Q4, process_noise, process_noise, meas_noise)
    truth_Q5_R2, mu_Q5_R2, S_Q5_R2 = three_wheel_robot(mu0, S0, ts, Q5, process_noise, process_noise, meas_noise)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_Q1_R1)), list(map(lambda x: x[1], truth_Q1_R1)))
    plt.legend(['Ground truth'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_Q1_R3)), list(map(lambda x: x[1], truth_Q1_R3)))
    plt.plot(list(map(lambda x : x[0], mu_Q1_R1)), list(map(lambda x: x[1], mu_Q1_R1)))
    plt.plot(list(map(lambda x : x[0], mu_Q1_R2)), list(map(lambda x: x[1], mu_Q1_R2)))
    plt.plot(list(map(lambda x : x[0], mu_Q1_R3)), list(map(lambda x: x[1], mu_Q1_R3)))
    plt.plot(list(map(lambda x : x[0], mu_Q1_R4)), list(map(lambda x: x[1], mu_Q1_R4)))
    plt.plot(list(map(lambda x : x[0], mu_Q1_R5)), list(map(lambda x: x[1], mu_Q1_R5)))

    plt.legend(['Ground truth',
                'R = diag(0.0001, 0.0001, 0.0001, 0.0001)',
                'R = diag(0.001, 0.001, 0.001, 0.001)',
                'R = diag(0.003, 0.003, 0.003, 0.003)',
                'R = diag(0.005, 0.005, 0.005, 0.005)',
                'R = diag(0.007, 0.007, 0.007, 0.007)'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_Q1_R3)), list(map(lambda x: x[1], truth_Q1_R1)))
    plt.plot(list(map(lambda x : x[0], mu_Q1_R2)), list(map(lambda x: x[1], mu_Q1_R2)))
    plt.plot(list(map(lambda x : x[0], mu_Q2_R2)), list(map(lambda x: x[1], mu_Q2_R2)))
    plt.plot(list(map(lambda x : x[0], mu_Q3_R2)), list(map(lambda x: x[1], mu_Q3_R2)))
    plt.plot(list(map(lambda x : x[0], mu_Q4_R2)), list(map(lambda x: x[1], mu_Q4_R2)))
    plt.plot(list(map(lambda x : x[0], mu_Q5_R2)), list(map(lambda x: x[1], mu_Q5_R2)))

    plt.legend(['Ground truth',
                'Q = diag(0.1, 0.3)',
                'Q = diag(0.2, 0.3)',
                'Q = diag(0.3, 0.3)',
                'Q = diag(0.1, 0.4)',
                'Q = diag(0.1, 0.5)'])

    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')
    plt.show()
