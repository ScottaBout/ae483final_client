import logging
import threading
import time

import cflib.crtp
from flask import Flask, request, Response

from drone_client import drone_data, BRAIN_IP, DRONE_ID, BRAIN_PORT, SimpleClient, CLIENT_PORT, uri, MockClient

logging.basicConfig(level=logging.DEBUG)

TESTING = True  # change to FALSE if working with drones

send_to_drone = True  # as long as this is true, the client will send position set point updates to the drone

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
        logging.debug(f'moving drone to {(drone_data.target_x, drone_data.target_y, drone_data.target_z)}')
        # client.cf.commander.send_position_setpoint(drone_data.target_x, drone_data.target_y, drone_data.target_z, 0)
        client.move(drone_data.target_x, drone_data.target_y, drone_data.target_z, 0, 0.01)
    client.move(0, 0, 0.5, 0, 5)
    client.stop(5)
    client.disconnect()


def app_run():
    logging.info('Starting Client web server')
    app.run(host='0.0.0.0', port=int(CLIENT_PORT), debug=True)


if __name__ == '__main__':
    if BRAIN_IP == '':
        logging.critical('IP_OF_BRAIN not set')
    else:
        logging.info(f'Starting Client for drone with ID={DRONE_ID}')
        logging.info(f'Using BRAIN_IP={BRAIN_IP} port {BRAIN_PORT}')

        # Initialize everything
        # logging.basicConfig(level=logging.ERROR)
        if not TESTING:
            cflib.crtp.init_drivers()

        #  Create and start the Client that will connect to the drone
        client = MockClient(uri, use_controller=True, use_observer=False) if TESTING else SimpleClient(uri,
                                                                                                       use_controller=True,
                                                                                                       use_observer=True)
        while not client.is_connected:
            logging.debug(f' ... connecting ...')
            time.sleep(1.0)

        logging.info('Starting send_target_to_drone thread')
        thread = threading.Thread(target=send_target_to_drone, args=(client,))
        thread.start()

        logging.info('Starting app_run thread')
        thread2 = threading.Thread(target=app_run)
        thread2.start()

        while True:
            time.sleep(1)
