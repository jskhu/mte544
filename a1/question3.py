import numpy as np
import math
import matplotlib.pyplot as plt

# Initial conditions
mu0 = [0.1, 0.2, 45 / 180 * np.pi]
b = 0.45
dt = 0.1
t_max = 10
ts = np.arange(0, t_max + 2*dt, dt)

def encoder_robot(mu0, ts, encoders, error):
    mu_true = []
    mu = []
    mu_true.append(mu0)
    mu_true.append(mu0)
    mu.append(mu0)
    mu.append(mu0)
    rmse = []

    for step, t in enumerate(ts):
        if step <= 1:
            continue

        dr_true = encoders['right'][step-1] - encoders['right'][step-2]
        dl_true = encoders['left'][step-1] - encoders['left'][step-2]

        dr = (encoders['right'][step-1] - encoders['right'][step-2]) * error['right'][step-1]
        dl = (encoders['left'][step-1] - encoders['left'][step-2]) * error['left'][step-1]

        dc_true = (dr_true + dl_true) / 2
        dtheta_true = (dr_true - dl_true) / b

        dc = (dr + dl) / 2
        dtheta = (dr - dl) / b
        mu_true.append(mu_true[step-1] + np.array([dc_true * np.cos(mu_true[step-1][2] + dtheta_true / 2),
                                                   dc_true * np.sin(mu_true[step-1][2] + dtheta_true / 2),
                                                   dtheta_true]))
        mu.append(mu[step-1] + np.array([dc * np.cos(mu[step-1][2] + dtheta / 2),
                                        dc * np.sin(mu[step-1][2] + dtheta / 2),
                                        dtheta]))
        rmse.append(np.sqrt(np.sum((mu_true[step] - mu[step])**2)))
    print('rmse: {}'.format(np.sum(rmse)))
    return mu_true, mu, rmse

if __name__ == "__main__":
    encoders = {
        'left': [np.cos(x) + np.sin(x) for x in ts],
        'right': [np.sin(x) - np.cos(x) for x in ts]
    }
    error = {
        'left': np.ones(len(ts)),
        'right': np.ones(len(ts))
    }
    mu_true_no_error, mu_no_error, rmse_no_error = encoder_robot(mu0, ts, encoders, error)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], mu_no_error)), list(map(lambda x: x[1], mu_no_error)))
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    encoders = {
        'left': [np.cos(x) + np.sin(x) for x in ts],
        'right': [np.sin(x) - np.cos(x) for x in ts]
    }
    error = {
        'left': np.ones(len(ts)) * 1.02,
        'right': np.ones(len(ts)) * 1.02
    }
    mu_true_2p, mu_2p, rmse_2p = encoder_robot(mu0, ts, encoders, error)
    encoders = {
        'left': [np.cos(x) + np.sin(x) for x in ts],
        'right': [np.sin(x) - np.cos(x) for x in ts]
    }
    error = {
        'left': np.ones(len(ts)) * 1.03,
        'right': np.ones(len(ts)) * 1.02
    }
    mu_true_diff, mu_diff, rmse_diff = encoder_robot(mu0, ts, encoders, error)

    plt.figure(figsize=(15,10))
    plt.plot(list(map(lambda x : x[0], mu_true_2p)), list(map(lambda x: x[1], mu_true_2p)))
    plt.plot(list(map(lambda x : x[0], mu_2p)), list(map(lambda x: x[1], mu_2p)))
    plt.plot(list(map(lambda x : x[0], mu_diff)), list(map(lambda x: x[1], mu_diff)))
    plt.legend(['Ground Truth', '2% error', '3% left and 2% right errror'])
    plt.xlabel('Position (m)')
    plt.ylabel('Position (m)')

    plt.figure(figsize=(15,10))
    plt.plot(np.arange(0, t_max, dt), rmse_2p, 'C1')
    plt.plot(np.arange(0, t_max, dt), rmse_diff, 'C2')
    plt.legend(['2% error', '3% left and 2% right errror'])
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.show()