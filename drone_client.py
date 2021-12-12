import json
import logging
import socket
import struct
import threading
import time

import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

import matplotlib.pyplot as plt

# Specify the uri of the drone to which we want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
from drone_data import DroneData

uri = 'radio://0/35/2M/E7E7E7E7E7'
BRAIN_IP = '10.183.5.60'  # TODO enter ip of brain
BRAIN_PORT = '8100'
CLIENT_PORT = '8080'
DRONE_ID = 0

drone_data = DroneData()

# Specify the variables we want to log (all at 100 Hz)
variables = [
    # State estimates (custom observer)
    'ae483log.o_x',
    'ae483log.o_y',
    'ae483log.o_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # State estimates (default observer)
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
    'kalman.statePX',
    'kalman.statePY',
    'kalman.statePZ',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Setpoint
    'ae483log.o_x_des',
    'ae483log.o_y_des',
    'ae483log.o_z_des',
    'ctrltarget.x',
    'ctrltarget.y',
    'ctrltarget.z',
    # Motor power commands
    # 'ae483log.m_1',
    # 'ae483log.m_2',
    # 'ae483log.m_3',
    # 'ae483log.m_4',
]


class SimpleClient:
    def __init__(self, uri, use_controller=False, use_observer=False):
        self.init_time = time.time()
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self.connected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)
        self.cf.disconnected.add_callback(self.disconnected)
        print(f'Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_connected = False
        self.data = {}
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.packets = 0

    def connected(self, uri):
        print(f'Connected to {uri}')
        self.is_connected = True

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 3:  # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self.log_data)
                logconf.error_cb.add_callback(self.log_error)
                logconf.start()
            except KeyError as e:
                print(f'Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')

        # Reset the stock EKF
        self.cf.param.set_value('kalman.resetEstimation', 1)

        # Enable the controller (1 for stock controller, 4 for ae483 controller)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 4)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
            self.cf.param.set_value('ae483par.reset_observer', 1)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)

    def connection_failed(self, uri, msg):
        print(f'Connection to {uri} failed: {msg}')

    def connection_lost(self, uri, msg):
        print(f'Connection to {uri} lost: {msg}')

    def disconnected(self, uri):
        print(f'Disconnected from {uri}')
        self.is_connected = False

    def log_data(self, timestamp, data, logconf):
        logging.info('Logging data, sending to Brain if modified')
        modified = False  # will be set to true if x, y or z is modified
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp)
            self.data[v.name]['data'].append(data[v.name])
            if v.name == 'ae483log.o_x':
                drone_data.x = data[v.name]
                modified = True
            if v.name == 'ae483log.o_y':
                drone_data.y = data[v.name]
                modified = True
            if v.name == 'ae483log.o_z':
                drone_data.z = data[v.name]
                modified = True
        if modified and self.packets % 30:
            # send to drone only if any of the drone x, y, z data has changed
            position = struct.pack('ifff', DRONE_ID, drone_data.x, drone_data.y, drone_data.z)
            b = self.socket.sendto(position, (BRAIN_IP, int(BRAIN_PORT)))
            logging.info(
                f'Successfully sent {b} bytes drone id+position: {(DRONE_ID, drone_data.x, drone_data.y, drone_data.z)}')
        else:
            # if no relevant data was modified
            logging.debug('Log data did not contain data to send to Brain')

    def log_error(self, logconf, msg):
        logging.error(f'Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    def move_smooth(self, p1, p2, yaw, dt):
        print(f'Move smoothly from {p1} to {p2} with yaw {yaw} degrees in {dt} seconds')
        p1 = np.array(p1)
        p2 = np.array(p2)
        start_time = time.time()
        while True:
            current_time = time.time()
            s = (current_time - start_time) / dt
            p = (1 - s) * p1 + (s * p2)
            self.cf.commander.send_position_setpoint(p[0], p[1], p[2], yaw)
            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def stop(self, dt):
        print(f'Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def zPos(self, z):
        print('sending')
        return z

    def disconnect(self):
        self.cf.close_link()

    def write_data(self, filename='logged_data.json'):
        with open(filename, 'w') as outfile:
            json.dump(self.data, outfile, indent=4, sort_keys=False)


class MockClient(SimpleClient):
    def __init__(self, uri, use_controller=False, use_observer=False):
        super().__init__(uri, use_controller, use_observer)
        self.is_connected = True
        self.logconfs = []
        my_data = {'ae483log.o_x': 1.2, 'ae483log.o_y': 3.7, 'ae483log.o_z': 0.72}
        my_logconf = LogConfig(name='test', period_in_ms=500)
        for v in my_data.keys():
            my_logconf.add_variable(v, fetch_as='float')
            self.data[v] = {'time': [], 'data': []}
        self.logconfs.append(my_logconf)
        thread = threading.Thread(target=simulate_log_update, args=(self,))
        thread.start()

    def stop(self, dt):
        logging.info('Mock stop')

    def move(self, x, y, z, yaw, dt):
        r = 0.1
        drone_data.target_x = x
        drone_data.target_y = y
        drone_data.target_z = z
        drone_data.x = (drone_data.target_x - drone_data.x) * r + drone_data.x
        drone_data.y = (drone_data.target_y - drone_data.y) * r + drone_data.y
        drone_data.z = (drone_data.target_z - drone_data.z) * r + drone_data.z
        logging.info(f'Mock move target {drone_data.target_x, drone_data.target_y, drone_data.target_z}')
        logging.info(f'Mock move to {drone_data.x, drone_data.y, drone_data.z}')
        time.sleep(0.1)

    def disconnect(self):
        logging.info('Mock disconnect')
        self.is_connected = False  # stops mock logging
        x = self.data['ae483log.o_x']['data']
        y = self.data['ae483log.o_y']['data']
        # plt.scatter(x, y)
        # plt.show()



def simulate_log_update(client: MockClient):


    print('Starting simulate log update')
    while client.is_connected:
        time.sleep(0.5)
        my_data = {'ae483log.o_x': drone_data.x,
                   'ae483log.o_y': drone_data.y,
                   'ae483log.o_z': drone_data.z}
        client.log_data(timestamp='time', data=my_data, logconf=client.logconfs[0])