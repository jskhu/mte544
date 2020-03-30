import numpy as np
import math
import matplotlib.pyplot as plt

def get_range(x, y):
    return np.sqrt(x**2 + y**2)

def get_angle(x, y):
    return math.atan2(y, x)

def two_wheel_robot(mu0, S0, ts, Q, R, v_func, w_func, process_noise, meas_noise):
    np.random.seed(1234)
    # [x, y, theta]^T
    mu = []
    S = []
    truth = []
    mu.append(mu0)
    S.append(S0)
    truth.append(mu0)

    for step, t in enumerate(ts):
        if step == 0:
            continue
        # Get control input
        v = v_func(t)
        w = w_func(t)

        # Update truth
        update = dt * np.array([v * np.cos(truth[step-1][2]),
                                v * np.sin(truth[step-1][2]),
                                w])
        truth.append(truth[step-1] + update + np.sqrt(np.diag(process_noise)) * np.random.randn(3))

        # Construct the G matrix
        G = np.eye(3)
        G[0][2] = -dt * v * np.sin(mu[step-1][2])
        G[1][2] = dt * v * np.cos(mu[step-1][2])

        # Setup initial prediction
        mup = mu[step-1] + dt * np.array([v * np.cos(mu[step-1][2]),
                                          v * np.sin(mu[step-1][2]),
                                          w])
        Sp = G @ S[step-1] @ G.transpose() + R

        # Get measurement
        z = np.copy(truth[step])
        noise = np.random.randn(3)
        z += np.sqrt(np.diag(meas_noise)) * noise

        H = np.eye(3)

        # Get Kalman gain
        K = Sp @ H.transpose() @ np.linalg.inv(H @ Sp @ H.transpose() + Q)

        mu.append(mup + K @ (z - mup))
        S.append((np.eye(3) - K @ H) @ Sp)

    return truth, mu, S

def two_wheel_robot_meas(mu0, S0, ts, Q, R, v_func, w_func, process_noise, meas_noise):
    np.random.seed(1234)
    # [x, y, theta]^T
    mu = []
    S = []
    truth = []
    mu.append(mu0)
    S.append(S0)
    truth.append(mu0)

    for step, t in enumerate(ts):
        if step == 0:
            continue
        # Get control input
        v = v_func(t)
        w = w_func(t)

        # Update truth
        update = dt * np.array([v * np.cos(truth[step-1][2]),
                                v * np.sin(truth[step-1][2]),
                                w])
        truth.append(truth[step-1] + update + np.sqrt(np.diag(process_noise)) * np.random.randn(3))

        # Construct the G matrix
        G = np.eye(3)
        G[0][2] = -dt * v * np.sin(mu[step-1][2])
        G[1][2] = dt * v * np.cos(mu[step-1][2])

        # Setup initial prediction
        mup = mu[step-1] + dt * np.array([v * np.cos(mu[step-1][2]),
                                          v * np.sin(mu[step-1][2]),
                                          w])
        Sp = G @ S[step-1] @ G.transpose() + R

        # Get measurement
        z = np.zeros(2)
        np.random.randn()
        z[0] = get_range(truth[step][0], truth[step][1]) + np.sqrt(process_noise[0][0]) * np.random.randn()
        z[1] = get_angle(truth[step][0], truth[step][1]) + np.sqrt(process_noise[1][1]) * np.random.randn()

        sqrtxy = np.sqrt(mup[0]**2 + mup[1]**2)
        H = np.array([[mup[0] / sqrtxy, mup[1] / sqrtxy, 0],
                      [0, 0, 1+(np.tan(mup[2]))**2]])

        # Get Kalman gain
        K = Sp @ H.transpose() @ np.linalg.inv(H @ Sp @ H.transpose() + Q)

        mu.append(mup + K @ (z - np.array([get_range(mup[0], mup[1]),
                                        get_angle(mup[0], mup[1])])))
        S.append((np.eye(3) - K @ H) @ Sp)

    return truth, mu, S

if __name__ == "__main__":
    Q = np.diag([0.5, 0.5, 1])
    R = np.diag([0.01, 0.01, 0.01])
    mu0 = np.array([2, -3, np.pi / 3])
    S0 = 0.01 * np.eye(3)
    dt = 0.1
    t_max = 20
    ts = np.arange(0, t_max + dt, dt)
    process_noise = np.diag([0.01, 0.01, 0.01])
    meas_noise = np.diag([0.5, 0.5, 1])
    v_func1 = lambda t : 2
    w_func1 = lambda t : np.cos(t/4)
    v_func2 = lambda t : 10 * np.random.uniform(low=-1, high=1)
    w_func2 = lambda t : -np.cos(t / 4)

    truth_p1, mu_p1, S_p1 = two_wheel_robot(mu0, S0, ts, Q, R, v_func1, w_func1, process_noise, meas_noise)
    truth_p2, mu_p2, S_p2 = two_wheel_robot(mu0, S0, ts, Q, R, v_func2, w_func2, process_noise, meas_noise)
    
    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_p1)), list(map(lambda x: x[1], truth_p1)))
    plt.plot(list(map(lambda x : x[0], truth_p2)), list(map(lambda x: x[1], truth_p2)))
    plt.legend(['Ground truth sinusoidal input','Ground truth random input',])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    S1 = 0.01 * np.eye(3)
    S2 = 0.1 * np.eye(3)
    S3 = 1 * np.eye(3)
    S4 = 10 * np.eye(3)

    truth_S1, mu_S1, S_S1 = two_wheel_robot(mu0, S1, ts, Q, R, v_func1, w_func1, process_noise, meas_noise)
    truth_S2, mu_S2, S_S2 = two_wheel_robot(mu0, S2, ts, Q, R, v_func1, w_func1, process_noise, meas_noise)
    truth_S3, mu_S3, S_S3 = two_wheel_robot(mu0, S3, ts, Q, R, v_func1, w_func1, process_noise, meas_noise)
    truth_S4, mu_S4, S_S4 = two_wheel_robot(mu0, S4, ts, Q, R, v_func1, w_func1, process_noise, meas_noise)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_S1)), list(map(lambda x: x[1], truth_S2)))
    plt.plot(list(map(lambda x : x[0], mu_S1)), list(map(lambda x: x[1], mu_S1)))
    plt.plot(list(map(lambda x : x[0], mu_S2)), list(map(lambda x: x[1], mu_S2)))
    plt.plot(list(map(lambda x : x[0], mu_S3)), list(map(lambda x: x[1], mu_S3)))
    plt.plot(list(map(lambda x : x[0], mu_S4)), list(map(lambda x: x[1], mu_S4)))
    plt.legend(['Ground truth',
                'S = diag(0.01, 0.01, 0.01)',
                'S = diag(0.1, 0.1, 0.1)',
                'S = diag(1.0, 1.0, 1.0)',
                'S = diag(10.0, 10.0, 10.0)'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    Q1 = np.diag([0.01, 0.01, 0.02])
    Q2 = np.diag([0.5, 0.5, 1])
    Q3 = np.diag([2.0, 2.0, 4.0])
    Q4 = np.diag([10.0, 10.0, 20.0])
    truth_Q1, mu_Q1, S_Q1 = two_wheel_robot(mu0, S1, ts, Q1, R, v_func1, w_func1, process_noise, meas_noise)
    truth_Q2, mu_Q2, S_Q2 = two_wheel_robot(mu0, S1, ts, Q2, R, v_func1, w_func1, process_noise, meas_noise)
    truth_Q3, mu_Q3, S_Q3 = two_wheel_robot(mu0, S1, ts, Q3, R, v_func1, w_func1, process_noise, meas_noise)
    truth_Q4, mu_Q4, S_Q4 = two_wheel_robot(mu0, S1, ts, Q4, R, v_func1, w_func1, process_noise, meas_noise)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_Q1)), list(map(lambda x: x[1], truth_Q1)))
    plt.plot(list(map(lambda x : x[0], mu_Q1)), list(map(lambda x: x[1], mu_Q1)))
    plt.plot(list(map(lambda x : x[0], mu_Q2)), list(map(lambda x: x[1], mu_Q2)))
    plt.plot(list(map(lambda x : x[0], mu_Q3)), list(map(lambda x: x[1], mu_Q3)))
    plt.plot(list(map(lambda x : x[0], mu_Q4)), list(map(lambda x: x[1], mu_Q4)))
    plt.legend(['Ground truth',
                'Q = diag(0.01, 0.01, 0.02)',
                'Q = diag(0.5, 0.5, 1.0)',
                'Q = diag(2.0, 2.0, 4.0)',
                'Q = diag(10.0, 10.0, 20.0)'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    R1 = np.diag([0.001, 0.001, 0.001])
    R2 = np.diag([0.01, 0.01, 0.01])
    R3 = np.diag([0.05, 0.05, 0.05])
    R4 = np.diag([1.0, 1.0, 1.0])
    truth_R1, mu_R1, S_Q1 = two_wheel_robot(mu0, S1, ts, meas_noise, R1, v_func1, w_func1, process_noise, meas_noise)
    truth_R2, mu_R2, S_Q2 = two_wheel_robot(mu0, S1, ts, meas_noise, R2, v_func1, w_func1, process_noise, meas_noise)
    truth_R3, mu_R3, S_Q3 = two_wheel_robot(mu0, S1, ts, meas_noise, R3, v_func1, w_func1, process_noise, meas_noise)
    truth_R4, mu_R4, S_Q4 = two_wheel_robot(mu0, S1, ts, meas_noise, R4, v_func1, w_func1, process_noise, meas_noise)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_R1)), list(map(lambda x: x[1], truth_R1)))
    plt.plot(list(map(lambda x : x[0], mu_R1)), list(map(lambda x: x[1], mu_R1)))
    plt.plot(list(map(lambda x : x[0], mu_R2)), list(map(lambda x: x[1], mu_R2)))
    plt.plot(list(map(lambda x : x[0], mu_R3)), list(map(lambda x: x[1], mu_R3)))
    plt.plot(list(map(lambda x : x[0], mu_R4)), list(map(lambda x: x[1], mu_R4)))
    plt.legend(['Ground truth',
                'Q = diag(0.01, 0.01, 0.02)',
                'Q = diag(0.5, 0.5, 1.0)',
                'Q = diag(2.0, 2.0, 4.0)',
                'Q = diag(10.0, 10.0, 20.0)'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    p1 = np.diag([0.001, 0.001, 0.001])
    p2 = np.diag([0.01, 0.01, 0.01])
    p3 = np.diag([0.1, 0.1, 0.1])

    truth_p1, mu_p1, S_p1 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, p1, meas_noise)
    truth_p2, mu_p2, S_p2 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, p2, meas_noise)
    truth_p3, mu_p3, S_p3 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, p3, meas_noise)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_p1)), list(map(lambda x: x[1], truth_p1)))
    plt.plot(list(map(lambda x : x[0], mu_p1)), list(map(lambda x: x[1], mu_p1)))
    plt.plot(list(map(lambda x : x[0], truth_p2)), list(map(lambda x: x[1], truth_p2)))
    plt.plot(list(map(lambda x : x[0], mu_p2)), list(map(lambda x: x[1], mu_p2)))
    plt.plot(list(map(lambda x : x[0], truth_p3)), list(map(lambda x: x[1], truth_p3)))
    plt.plot(list(map(lambda x : x[0], mu_p3)), list(map(lambda x: x[1], mu_p3)))

    plt.legend(['Process Noise = diag(0.001, 0.001, 0.001)',
                'EKF for diag(0.001, 0.001, 0.001)',
                'Process Noise = diag(0.01, 0.01, 0.01)',
                'EKF for diag(0.01, 0.01, 0.01)',
                'Process Noise = diag(0.1, 0.1, 0.1)',
                'EKF for diag(0.1, 0.1, 0.1)'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    m1 = np.diag([0.01, 0.01, 0.02])
    m2 = np.diag([0.1, 0.1, 0.2])
    m3 = np.diag([0.5, 0.5, 1.0])
    m4 = np.diag([1.0, 1.0, 2.0])
    m5 = np.diag([5.0, 5.0, 10.0])

    truth_m1, mu_m1, S_m1 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, process_noise, m1)
    truth_m2, mu_m2, S_m2 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, process_noise, m2)
    truth_m3, mu_m3, S_m3 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, process_noise, m3)
    truth_m4, mu_m4, S_m4 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, process_noise, m4)
    truth_m5, mu_m5, S_m5 = two_wheel_robot(mu0, S1, ts, Q2, R2, v_func1, w_func1, process_noise, m5)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_m1)), list(map(lambda x: x[1], truth_m1)))
    plt.plot(list(map(lambda x : x[0], mu_m1)), list(map(lambda x: x[1], mu_m1)))
    plt.plot(list(map(lambda x : x[0], mu_m2)), list(map(lambda x: x[1], mu_m2)))
    plt.plot(list(map(lambda x : x[0], mu_m3)), list(map(lambda x: x[1], mu_m3)))
    plt.plot(list(map(lambda x : x[0], mu_m4)), list(map(lambda x: x[1], mu_m4)))
    plt.plot(list(map(lambda x : x[0], mu_m5)), list(map(lambda x: x[1], mu_m5)))

    plt.legend(['Ground truth',
                'Measurement Noise = diag(0.01, 0.01, 0.02)',
                'Measurement Noise = diag(0.1, 0.1, 0.2)',
                'Measurement Noise = diag(0.5, 0.5, 1.0)',
                'Measurement Noise = diag(1.0, 1.0, 2.0)',
                'Measurement Noise = diag(5.0, 5.0, 10.0)',])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    Q = np.diag([0.5, 1])
    R = R2
    mu0 = np.array([2, -3, np.pi / 3])
    S0 = 0.01 * np.eye(3)
    dt = 0.1
    t_max = 20
    ts = np.arange(0, t_max + dt, dt)

    truth_meas, mu_meas, S_meas = two_wheel_robot_meas(mu0, S0, ts, Q, R, v_func1, w_func1, R, Q)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], truth_meas)), list(map(lambda x: x[1], truth_meas)))
    plt.plot(list(map(lambda x : x[0], mu_meas)), list(map(lambda x: x[1], mu_meas)))
    plt.legend(['Ground truth',
                'Estimate'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')
    plt.show()