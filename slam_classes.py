import numpy as np
import matplotlib.pyplot as plt
import math
import time
from numpy import linalg

from slam_functions import *


class RobotPos:
    """
    These are the robot poses at each timestamp.
    """
    # Start from our initial guess then will be updated
    def __init__(self, x, y, theta):
        # Robot poses
        self.x = x            # x position [m]
        self.y = y            # y position [m]
        self.theta = theta    # orientation [rad]


class LandmarkPos:
    """
    These are the positions of 20 landmarks(fixed)
    """
    # Start from our initial guess then will be updated
    def __init__(self, x, y):
        self.x = x           # x position [m]
        self.y = y           # y position [m]


class Measurements:
    """
    The odometry measurements and laser measurements at each time stamp
    """
    # Can be read from data_preprocessed.txt
    # Covariance matrix seems to be missing.
    def __init__(self, fra, time, landmark_id, las_range, las_bear, vel_for, vel_ang):
        """
        Measurements constructor, containing observations and odometric information
        """
        self.fra = fra              # Frame / timestamp
        self.time = time            # Time [s]
        # Observation measurements
        self.landmark_id = landmark_id    # The observed landmarks
        self.las_range = las_range  # Actual laser range
        self.las_bear = las_bear    # Actual laser bearing
        # Odometry measurements
        self.vel_for = vel_for      # Actual forward velocity
        self.vel_ang = vel_ang      # Actual angular velocity


class SLAMGraph:
    def __init__(self, meas=[], robot_pos=[], landmark_pos=[], verbose=True):
        self.meas = meas
        self.robot_pos = robot_pos                      # 1986
        self.landmark_pos = landmark_pos                # 17
        self.E = np.zeros((3*1986+2*10492, 3*1986+34))  # 1986 timestamps (thus 1986 robot poses), but 10492 observations
        self.E_nonzero = np.zeros((3*1986+2*10492, 3*1986+14))
        self.e_bar = np.zeros((3*1986+2*10492, 1))
        # odom_var = 1986 * [4.4e-05, 4.4e-05, 8.2e-05]
        # meas_var = 10492 * [9.0036e-04, 6.7143e-04]
        # self.T = np.diag(odom_var + meas_var)  # Diagonal matrix with odometry / observation noise variances
        # Solve A*dz_star = b
        self.A = []                                     # dim(A) = (3x1986+34) x (3x1986+34)
        self.b = []                                     # dim(b) = (3x1986+34) x 1
        self.verbose = verbose                          # show optimization steps

    def read_data(self, meas_file, node_file):
        """
        Read the measurement data and initial guess from files
        """
        meas_data = np.loadtxt(meas_file)
        num_timestamp = 1986                 # the number of timestamps
        for ele in meas_data:                # 10492 measurements, state 0 included
            self.meas.append(Measurements(fra=ele[0],
                                          time=ele[1],
                                          landmark_id=ele[2],
                                          las_range=ele[3],
                                          las_bear=ele[4],
                                          vel_for=ele[5],
                                          vel_ang=ele[6]))
        nodes = np.loadtxt(node_file)
        nodes_len = nodes.shape[0]
        for i in range(num_timestamp + 17):  # 1986 + 17 obj
            if i < num_timestamp:
                rb_pos = nodes[(3 * i):(3 * i + 3)]  # three values
                self.robot_pos.append(RobotPos(x=rb_pos[0], y=rb_pos[1], theta=rb_pos[2]))

            else:
                lm_pos = nodes[(3 * num_timestamp + 2 * (i - num_timestamp)):(3 * num_timestamp + 2 * (i - num_timestamp) + 2)]  # landmark position
                self.landmark_pos.append(LandmarkPos(x=lm_pos[0], y=lm_pos[1]))

    def linearize(self):
        """
        Linearization of motion model and observation model
        """
        prev_time = 0
        idx = 0                             # in range [0, 15130]
        old_idx = 0
        for ele in self.meas:               # 10492 measurements, state 0 included
            # Get the time stamp, altogether 1986 time stamps, state 0 excluded
            k = int(ele.fra)                # in range [0, 1986], same timestamp -> same frame
            # print("k = ", k)
            curr_time = float(ele.time)
            if k == 0:
                X_k = np.array([[3.0197561], [7.0899048e-02], [-2.9101574]])
            if curr_time != prev_time:      # starts from k = 1
                # 1. Get the robot pose
                X_k = np.array([[self.robot_pos[k - 1].x],
                                [self.robot_pos[k - 1].y],
                                [self.robot_pos[k - 1].theta]])
                if k == 1:
                    X_k_minus_1 = np.array([[3.0197561], [7.0899048e-02], [-2.9101574]])
                else:
                    X_k_minus_1 = np.array([[self.robot_pos[k - 2].x],
                                            [self.robot_pos[k - 2].y],
                                            [self.robot_pos[k - 2].theta]])
                u_k = np.array([[self.meas[idx].vel_for],
                                [self.meas[idx].vel_ang]])
                # f_k = f(x_k-1, uk, 0)
                trans_mat = np.array([[np.cos(X_k_minus_1[2]), 0],
                                      [np.sin(X_k_minus_1[2]), 0],
                                      [0,                      1]], dtype=float)
                delta_t = curr_time - prev_time
                # We use the linear motion model to simplify the algorithm
                # X_k_minus_1: 3 x 1, trans_mat: 3 x 2, u_k: 2 x 1
                f_k = X_k_minus_1 + delta_t * trans_mat.dot(u_k)
                e_k = X_k - f_k   # 3 x 1 vector: motion error = actual - predicted by motion model
                e_k[2, 0] = determine_angle(e_k[2, 0])
                # Is the above e_k_bar?
                # Insert numbers into vector e_bar
                # self.e_bar: 3*1986+2*10492 x 1
                self.e_bar[(3 * k - 3):(3 * k), :] = e_k
                # Compute the Jacobian
                F_k_minus_1 = np.array([[1, 0, float(-delta_t*u_k[0]*math.sin(X_k_minus_1[2]))],
                                        [0, 1, float(delta_t*u_k[0]*math.cos(X_k_minus_1[2]))],
                                        [0, 0,                                             1]])
                # Insert numbers into matrix E
                sub_mat_motion = np.hstack((-1 * F_k_minus_1, np.eye(3)))
                # Shape of sub_mat_motion is 3x6
                if k == 1:
                    self.E[:3, :3] = np.eye(3)
                else:
                    self.E[(3 * k - 3):(3 * k), (3 * k - 6):(3 * k)] = sub_mat_motion
                prev_time = curr_time
                old_idx = idx

            # 2. Get the landmark position that can be observed at this time step
            # At the same timestamp, different obs -> different landmark ids
            lm_id = int(self.meas[idx].landmark_id)  # lm_id starts from 1
            lm_pos = np.array([self.landmark_pos[lm_id - 1].x,
                               self.landmark_pos[lm_id - 1].y])
            y_kl = np.array([[self.meas[idx].las_range],
                             [self.meas[idx].las_bear]])

            # g_kl = g(x_k, m_l, 0)
            g_kl = np.array([[math.sqrt((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2)],
                             [wraptopi(np.arctan2(float(lm_pos[1]) - float(X_k[1]), float(lm_pos[0]) - float(X_k[0])) - float(X_k[2]))]])
            e_kl = y_kl - g_kl  # 2 x 1 vector: observation error = actual - predicted by obs model
            e_kl[1, 0] = determine_angle(e_kl[1, 0])

            # Is the above e_kl_bar?
            # Insert numbers into vector e_bar
            self.e_bar[(3 * 1986 + 2 * idx):(3 * 1986 + 2 * (idx + 1)), :] = e_kl
            # Compute the Jacobians
            G_x_kl = np.array([[float((X_k[0] - lm_pos[0]) / math.sqrt((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2)),
                                float((X_k[1] - lm_pos[1]) / math.sqrt((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2)),
                                0],
                               [float((lm_pos[1] - X_k[1]) / ((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2)),
                                float((X_k[0] - lm_pos[0]) / ((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2)),
                                -1]]).reshape(2, 3)

            G_m_kl = np.array([[(lm_pos[0] - X_k[0]) / math.sqrt((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2),
                                (lm_pos[1] - X_k[1]) / math.sqrt((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2)],
                               [(X_k[1] - lm_pos[1]) / ((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2),
                                (lm_pos[0] - X_k[0]) / ((X_k[0] - lm_pos[0])**2 + (X_k[1] - lm_pos[1])**2)]]).reshape(2, 2)
            # Insert numbers into matrix E
            if k != 0:
                self.E[(3 * 1986 + 2 * idx):(3 * 1986 + 2 * (idx + 1)), (3 * k - 3):(3 * k)] = -1 * G_x_kl
            self.E[(3 * 1986 + 2 * idx):(3 * 1986 + 2 * (idx + 1)), (3 * 1986 + 2 * (lm_id - 1)):(3 * 1986 + 2 * lm_id)] = -1 * G_m_kl
            idx += 1

        # If not all 17 landmarks are observed
        # zero_col_E = np.where(~self.E.any(axis=0))[0]
        # self.E_nonzero = np.delete(self.E, zero_col_E, axis=1)

        # self.A = np.dot(np.dot(self.E_nonzero.T, np.linalg.inv(self.T)), self.E_nonzero)
        quad = np.dot(self.E.T, self.E)
        self.A = quad                                  # simple Gauss-Newton
        # self.A = quad + 0.2 * np.diag(np.diag(quad))     # Levenberg-Marquardt approach
        # self.b = -np.dot(np.dot(self.E_nonzero.T, np.linalg.inv(self.T)), self.e_bar)
        self.b = -np.dot(self.E.T, self.e_bar)

    def optimize(self, path_txt, path_fig, max_iterations, tolerance):
        """
        Graph optimization - least squared error minimization
        """
        norm_dz_output = []
        err_output = []
        tol_cnt = 0              # tolerance counter
        opt_iterations = None    # number of iterations in optimization process
        t_opt_start = time.time()

        for i in range(max_iterations):
            t_step_start = time.time()

            if self.verbose:
                print('\r\nPose Graph Optimization, iteration %d.' % (i + 1))

            # Create empty matrix A and vector b
            # dim(A) = (3x1986+34) x (3x1986+34)
            self.A = np.zeros((3*1986+34, 3*1986+34), dtype=np.float64)
            # dim(b) = (3x1986+34) x 1
            self.b = np.zeros((3*1986+34, 1), dtype=np.float64)

            if self.verbose:
                print("Linearizing...")

            # Linearization of motion model and observation model
            self.linearize()

            if self.verbose:
                print('Solving...')

            dz = solve(A=self.A, b=self.b, sparse_solve=True)
            norm_dz = np.linalg.norm(dz)
            norm_dz_output.append(norm_dz)

            norm_err = linalg.norm(self.e_bar)
            err_output.append(norm_err)

            if self.verbose:
                print("Updating robot poses and landmark positions...")

            # dz includes robot pose increments and landmark position increments
            self.update_z(dz, i, path_txt, path_fig)

            if self.verbose:
                print("step duration: %f [s]" % (time.time() - t_step_start))
                print("error = %f" % norm_err)
                print("|dz| = %f" % norm_dz)

            # Convergence check
            if i >= 1 and np.abs(norm_dz_output[i] - norm_dz_output[i-1]) < tolerance:
            # if i >= 1 and np.abs(norm_dz) < tolerance or norm_dz_output[i] - norm_dz_output[i - 1] > 0:
            # if i >= 1 and np.abs(norm_err) < tolerance:
            # if i >= 1 and np.abs(norm_dz) < tolerance:
                tol_cnt += 1
            else:
                tol_cnt = 0

            if tol_cnt >= 2:
                opt_iterations = i+1
                break

            if self.verbose:
                if opt_iterations == None:
                    print('\r\nOptimization process finished - maximum number of iterations reached!')
            else:
                print('\r\nOptimization process converged after %d iterations!' % opt_iterations)
            print('Optimization process duration: %.2f [s]' % (time.time() - t_opt_start))
        return norm_dz_output, err_output

    def update_z(self, dz, iter, path_txt, path_fig):
        """
        Update the estimates of robot poses and landmark positions given increments
        """
        damping = 1.0
        # Update robot poses
        dz_robot_pos = damping * dz[:1986 * 3].reshape(1986, 3)   # At each timestamp, a robot pose has 3 values
        for i in range(1986):
            self.robot_pos[i].x += dz_robot_pos[i, 0]
            self.robot_pos[i].y += dz_robot_pos[i, 1]
            self.robot_pos[i].theta += dz_robot_pos[i, 2]
        # Update landmark positions
        dz_landmark_pos = damping * dz[1986 * 3:].reshape(17, 2)  # Each landmark position has 2 values
        for j in range(17):
            self.landmark_pos[j].x += dz_landmark_pos[j, 0]
            self.landmark_pos[j].y += dz_landmark_pos[j, 1]
        self.output_res(str(iter + 1), path_txt, path_fig)

    def output_res(self, title, path_txt, path_fig):
        """
        Write the estimates to files and plot the trajectories
        """
        f = open(path_txt + title + '.txt', 'w')
        plt.figure()

        rb_x = []
        rb_y = []
        for rb_posi in self.robot_pos:
            rb_x.append(rb_posi.x)
            rb_y.append(rb_posi.y)
            f.write(str(rb_posi.x) + '\n')
            f.write(str(rb_posi.y) + '\n')
            f.write(str(rb_posi.theta) + '\n')

        plt.scatter(rb_x, rb_y, c='peru', marker='o', s=2, label='robot positions')
        plt.plot(rb_x, rb_y, c='peru')

        lm_x = []
        lm_y = []
        for lm_posi in self.landmark_pos:
            lm_x.append(lm_posi.x)
            lm_y.append(lm_posi.y)
            f.write(str(lm_posi.x) + '\n')
            f.write(str(lm_posi.y) + '\n')

        plt.scatter(lm_x, lm_y, c='darkcyan', marker='x', s=4, label='landmark positions')
        plt.title(title)
        plt.xlabel('x[m]')
        plt.ylabel('y[m]')
        plt.legend(loc='upper right')
        # plt.show()
        plt.savefig(path_fig + title + ".png")
        plt.close()
        plt.clf()

        f.close()



