import logging
import threading
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import requests
from flask import Flask, request, Response
from drone_data import DroneData

logging.basicConfig(level=logging.INFO)

# Specify the uri of the drone to which we want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/35/2M/E7E7E7E7E7'
IP_OF_BRAIN = ''  # TODO enter ip of brain

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

drone_data = DroneData()
send_to_drone = True


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

    def connected(self, uri):
        print(f'Connected to {uri}')
        self.is_connected = True

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5:  # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
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
        logging.info('logging and sending data')
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp)
            self.data[v.name]['data'].append(data[v.name])
            if v.name == 'ae483log.o_x':
                drone_data.x = data[v.name]
            if v.name == 'ae483log.o_y':
                drone_data.y = data[v.name]
            if v.name == 'ae483log.o_z':
                drone_data.z = data[v.name]
        payload = drone_data.string_dict()
        payload['drone_id'] = '0'  # TODO change drone id
        try:
            response = requests.get(f'http://{IP_OF_BRAIN}:8080/drone_data', params=payload)
            if response.status_code != 200:
                print(f'Error code sending request {response.status_code}')
            else:
                print(f'url: {response.url}')
                print(f'response: {response.content}')
        except requests.exceptions.RequestException:
            logging.info('catch error while receiving')

    def log_error(self, logconf, msg):
        print(f'Error when logging {logconf}: {msg}')

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


# Web server listening to brain
app = Flask(__name__)
logging.info('flask server initiated')


@app.route("/drone_target")
def drone_target():
    """
    Request coming from brain containing target x, y, and z coordinates
    :return:
    """
    logging.info('drone target')
    x_str = request.args.get("target_x")  # gets the 'x’ argument as str
    drone_data.target_x = float(x_str)  # convert string to float
    y_str = request.args.get("target_y")  # gets the ‘y’ argument as str
    drone_data.target_y = float(y_str)  # convert string to float
    z_str = request.args.get("target_z")  # gets the ‘z’ argument as str
    drone_data.target_z = float(z_str)  # convert string to float
    logging.info(f'drone target: {x_str}, {y_str}, {z_str}')
    return Response('ok')


@app.route('/end')
def end():
    """
    Ends program
    :return:
    """
    logging.info('ending...')
    global send_to_drone
    send_to_drone = False


def send_target_to_drone(client):
    logging.info('sending target to drone')
    while send_to_drone:
        print('moving drone')
        # client.cf.commander.send_position_setpoint(drone_data.target_x, drone_data.target_y, drone_data.target_z, 0)
        client.move(drone_data.target_x, drone_data.target_y, drone_data.target_z, 0, 0.01)
    client.move(0, 0, 0.5, 0, 5)
    client.stop(5)
    client.disconnect()


if __name__ == '__main__':
    logging.info('main')
    # Initialize everything
    # logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    #  Create and start the Client that will connect to the drone
    client = SimpleClient(uri, use_controller=True, use_observer=True)
    while not client.is_connected:
        print(f' ... connecting ...')
        time.sleep(1.0)
    thread = threading.Thread(target=send_target_to_drone, args=(client,))
    thread.start()
    logging.info('threading')

    logging.info('starting web server on port 8080')
    app.run(host='0.0.0.0', port=8080, debug=True)

