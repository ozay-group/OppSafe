import logging
import os
import sys
import warnings
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import numpy as np
np.random.seed(42)


logging.basicConfig(level=logging.ERROR)


class DroneVel:
    def __init__(self, uri, controller=None, supervisor=None, save_flight_data=False, log_path='.'):
        print('Initializing Crazyflie')
        self.state_estimate = [0, 0, 0, 0]
        self.timestamp = None
        self.uri = uri_helper.uri_from_env(default=uri)
        self.deck_attached_event = Event()
        cflib.crtp.init_drivers()

        self.logconf = LogConfig(name='Position', period_in_ms=10)
        self.init_logconf()
        self.scf = None

        self.controller = controller
        self.supervisor = supervisor

        self.z = 0

        if save_flight_data:
            log_time = time.strftime('%Y%m%d_%H%M%S')
            os.makedirs(log_path, exist_ok=True)
            self.log_file = open(os.path.join(log_path, f"v_flight_data_{log_time}.csv"), 'w')
            self.log_file.write(
                'timestamp,vel_x,vel_y,state_estimate_x,state_estimate_y,state_estimate_vx,state_estimate_vy,raw_x,raw_y,z,alpha_x,alpha_y,lm_x,lm_y\n')
        else:
            self.log_file = None

        self.state_estimate_ = [0, 0, 0, 0]

    def __del__(self):
        if self.log_file is not None:
            self.log_file.close()

    def init_logconf(self):
        self.logconf.add_variable('stateEstimate.x', 'float')
        self.logconf.add_variable('stateEstimate.y', 'float')
        self.logconf.add_variable('stateEstimate.vx', 'float')
        self.logconf.add_variable('stateEstimate.vy', 'float')
        self.logconf.data_received_cb.add_callback(self.log_pos_callback)

        self.logconf.add_variable('stateEstimate.z', 'float')

    def log_pos_callback(self, timestamp, data, logconf):
        # print(data)
        self.timestamp = timestamp
        # kp, kv, = 0.05, 0.05
        self.state_estimate[0] = data['stateEstimate.x'] + np.random.normal(0, 0.05)
        self.state_estimate[1] = data['stateEstimate.y'] + np.random.normal(0, 0.05)
        self.state_estimate[2] = data['stateEstimate.vx']+ np.random.normal(0, 0.05)
        self.state_estimate[3] = data['stateEstimate.vy']+ np.random.normal(0, 0.05)
        self.state_estimate_[0] = data['stateEstimate.x']
        self.state_estimate_[1] = data['stateEstimate.y']
        self.state_estimate_[2] = data['stateEstimate.vx']
        self.state_estimate_[3] = data['stateEstimate.vy']

        self.z = data['stateEstimate.z']

    def param_deck_flow(self, _, value_str):
        value = int(value_str)
        if value:
            self.deck_attached_event.set()
            print('Deck is attached')
        else:
            print('Deck is NOT attached')

    def flight_routine(self):
        with MotionCommander(self.scf, default_height=0.5) as mc:
            self.controller.time = time.time()
            self.controller.cur_idx = 0
            while True:
                t_s = time.time()

                vel = self.controller.eval(self.state_estimate)
                if self.supervisor is not None:
                    print("#########################")
                    vel_ = vel
                    vel, alpha = self.supervisor.eval(self.state_estimate, vel)
                print("desired velocity: v_x = %f, v_y = %f" % (vel[0], vel[1]))
                mc.start_linear_motion(vel[0], vel[1], 0)
                if self.z < 0.4:
                    print("################################ Catch you! #########################################")

                if self.log_file is not None:
                    log_data = [
                        self.timestamp,
                        *vel,
                        *self.state_estimate_,
                        *vel_,
                        self.z,
                        *alpha,
                        self.controller.landmarks[self.controller.cur_idx][0],
                        self.controller.landmarks[self.controller.cur_idx][1]
                    ]
                    # print(log_data)
                    print("x = %f, y = %f, vx = %f, vy = %f" % (self.state_estimate_[0], self.state_estimate_[1], self.state_estimate_[2], self.state_estimate_[3]))
                    self.log_file.write(','.join(map(str, log_data)) + '\n')

                t_e = time.time()
                print("time: ", (t_e - t_s))
                t = 0.1 - (t_e - t_s)
                if t > 0:
                    time.sleep(t)
                else:
                    print("Iteration took longer than expected")

    def run(self):
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as self.scf:
            self.scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=self.param_deck_flow)
            time.sleep(1)
            self.scf.cf.log.add_config(self.logconf)

            if not self.deck_attached_event.wait(timeout=5):
                print('No flow deck detected')
                sys.exit(1)

            self.logconf.start()
            self.flight_routine()
            self.logconf.stop()
    
    def stop(self):
        self.scf.cf.commander.send_stop_setpoint()
        print("Emergency stop command sent!")
