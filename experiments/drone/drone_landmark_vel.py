import time
import os
import numpy as np
from drone_vel import DroneVel
from scipy.io import loadmat
from alpha_filter import AlphaFilter
from existed_filter import ExistedFilter

URI = 'radio://0/80/2M/E7E7E7E7E7' # replace with your drone address

class LQRController:
    def __init__(self, K, max_vel=1.0, landmarks=None, save_traj_data=False, log_path='.'):
        self.target_state = np.zeros(4)
        self.K = K
        self.max_vel = max_vel

        if landmarks is None:
            landmarks = [[0, 0]]
        self.landmarks = np.array([self.gen_ref_state(landmark) for landmark in landmarks])
        self.cur_idx = 0

        if save_traj_data:
            log_time = time.strftime('%Y%m%d_%H%M%S')
            os.makedirs(log_path, exist_ok=True)
            self.log_file = open(os.path.join(log_path, f"v_traj_{log_time}.csv"), "w")
        else:
            self.log_file = None

        self.pre_u = [0, 0]

        self.time = time.time()

    def __del__(self):
        if self.log_file is not None:
            self.log_file.close()

    def gen_ref_state(self, state):
        state = state + [0, 0] + [0, 0]
        return state

    def eval(self, state):
        err = self.landmarks[self.cur_idx] - np.hstack((state, self.pre_u))
        u = self.K @ err
        vel = np.clip(u, -self.max_vel, self.max_vel)

        self.pre_u = u

        print('current target runtime:', time.time() - self.time)
        if np.linalg.norm(err) < 0.025 or time.time() - self.time > 5:
            print('Arrived, move to the next landmark.')
            self.cur_idx = self.cur_idx + 1
            self.time = time.time()

        return vel.tolist()

if __name__ == '__main__':
    lms = [[0, 0.25],
           [0.5, 1],
           [1.2, 1],
           [1.2, 0.5],
           [1, 0.5],
           [1, -0.5],
           [1.2, -0.5],
           [1.2, -1],
           [0.3, -1],
           [0.3, -0.5],
           [0.5, -0.5],
           [0.5, 0.25],
           [0, -0.5],
           [-0.5, 0.25],
           [-0.5, -0.5],
           [-0.3, -0.5],
           [-0.3, -1],
           [-1.2, -1],
           [-1.2, -0.5],
           [-1, -0.5],
           [-1, 0.5],
           [-1.2, 0.5],
           [-1.2, 1],
           [-0.5, 1],
           [0, 0.25]]
    # initialize the reference tracking controller
    K = loadmat('data/K.mat')
    ctl = LQRController(K['K'], landmarks=lms, save_traj_data=False, log_path='data')
    
    data = loadmat('data/alpha_filter_py.mat')
    
    # initialize the opportunistic safety supervisory (aka alpha filter)
    alpha_filter = AlphaFilter(data['H_max'], data['h_max'], data['H_xu'], data['h_xu'])
    # initialize the robust safety supervisory
    robust_filter = ExistedFilter(data['H_max'], data['h_max'], data['H_xu'], data['h_xu'])
    # We use the opportunistic safety supervisor. You can swith to the robust safety
    # supervisor by setting "supervisor=robust_filter"
    drone = DroneVel(URI, controller=ctl, supervisor=alpha_filter, save_flight_data=True, log_path='data')

    try:
        drone.run()
    except KeyboardInterrupt:
        drone.stop()
        if drone.log_file is not None:
            drone.log_file.close()
        if ctl.log_file is not None:
            ctl.log_file.close()
    except Exception as e:
        drone.stop()
        if drone.log_file is not None:
            drone.log_file.close()
        if ctl.log_file is not None:
            ctl.log_file.close()
        print(e)
