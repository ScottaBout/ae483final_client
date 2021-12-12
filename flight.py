import logging
import socket
import struct
import threading
import time
from multiprocessing import Process, Queue
from queue import Empty

import cflib.crtp

from drone_client import BRAIN_IP, DRONE_ID, BRAIN_PORT, SimpleClient, CLIENT_PORT, uri, MockClient

LOGLEVEL = logging.DEBUG
TESTING = True  # change to FALSE if working with drones




def socket_listener(queue: Queue):
    """
    Process target that listens to socket and sets drone_data based on
    incoming data from Brain
    :param queue shared queue to pass data from listener process to main process
    """
    logging.basicConfig(level=LOGLEVEL)
    logging.info('Starting socket listener')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('0.0.0.0', int(CLIENT_PORT)))
        logging.info(f'Socket connected and listening')
        while True:
            data = s.recvfrom(1024)[0]
            if not data:
                logging.info('No data received')
                break
            else:
                if len(data) == 3 * 4:
                    # struct contains 3 floats, representing target x, y and z
                    targets = struct.unpack('fff', data)  # convert the received data from bytes to float
                    logging.debug(f'Received targets {targets} from Brain')
                    queue.put(targets)  # push into queue, read by main process
                    if targets[2] < 0:
                        # end if target_z < 0
                        break
                else:
                    logging.warning(f'Received only {len(data)} bytes: {data}')
    logging.info('Socket listener finished')


def send_target_to_drone(queue: Queue):
    """
    Thread target that pulls target x, y and z from the queue and
    sends it to the drone via the CrazyFlie SimpleClient
    :param queue shared queue that receives data from the listener process
    """
    logging.basicConfig(level=LOGLEVEL)
    logging.info('Thread starting: send_target_to_drone')
    #  Create and start the Client that will connect to the drone
    client = MockClient(uri, use_controller=True, use_observer=False) if TESTING else SimpleClient(uri,
                                                                                                   use_controller=True,
                                                                                                   use_observer=True)
    while not client.is_connected:
        logging.debug(f' ... connecting to CrazyFlie drone client ...')
        time.sleep(1.0)
    logging.info('CrazyFlie drone client connected')
    while True:
        try:
            targets = queue.get(block=False)
            logging.debug(f'moving drone to {targets}')
            client.move(targets[0], targets[1], targets[2], 0, 0.1)
            if targets[2] < 0:
                # land drone if z target is < 0
                break
        except Empty:
            time.sleep(0.1)
    logging.info('Landing drone')
    client.move(0, 0, 0.5, 0, 5)
    client.stop(5)
    client.disconnect()
    logging.info('Thread ending: send_target_to_drone')


def main():
    logging.basicConfig(level=LOGLEVEL)
    if BRAIN_IP == '':
        logging.critical('IP_OF_BRAIN not set')
    else:
        logging.info(f'Starting Client for drone with ID={DRONE_ID}')
        logging.info(f'Using BRAIN_IP={BRAIN_IP} port {BRAIN_PORT}')

        # Initialize everything
        # logging.basicConfig(level=logging.ERROR)
        if not TESTING:
            cflib.crtp.init_drivers()

        q = Queue()  # used to pass information from the listener process to the main process
        logging.info('Starting send_target_to_drone thread')
        thread = threading.Thread(target=send_target_to_drone, args=(q,))
        thread.start()

        logging.info('Starting separate brain communication process')
        process = Process(target=socket_listener, args=(q,))
        process.start()

        thread.join()  # waits until send_target_to_drone ends
        process.join()
        logging.info('Client is finished')


if __name__ == '__main__':
    main()