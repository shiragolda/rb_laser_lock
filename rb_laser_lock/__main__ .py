"""Logs the error signal of the rb laser servo."""

import time
import random
import zmq
import datetime
import sys
import numpy as np
from scipy.interpolate  import interp1d
import os
import serial

import argparse



parser = argparse.ArgumentParser(description='Reads and publishes the cavity'
                                 'temperature on a zeromq socket.\n')
parser.add_argument("-s", "--serialport", type=str,
                    help='Serial port to use to communicate with the'
                         'wavemeter. e.g /dev/ttyUSB0 for linux, '
                         'COM10 for windows',
                    default='COM10')
parser.add_argument("-p", "--publishport", type=int,
                    help='zeromq port to use to broadcast the wavemeter'
                         'reading.',
                    default=5550)
parser.add_argument("-t", "--topic", type=str,
                    help='topic to use when broadcasting. e.g wa1500-lab1',
                    default='rb_laser')
args = parser.parse_args()

class zmq_pub_dict:
    """Publishes a python dictionary on a port with a given topic."""

    def __init__(self, port, topic):
        zmq_context = zmq.Context()
        self.topic = topic
        self.pub_socket = zmq_context.socket(zmq.PUB)

        self.pub_socket.bind("tcp://*:%s" % port)
        print('Broadcasting on port {0} with topic {1}'.format(port,
                                                               topic))

    def send(self, data_dict):
        timestamp = time.time()
        send_string = "%s %f %s" % (self.topic, timestamp, repr(data_dict))
        print(send_string)
        self.pub_socket.send(send_string)

    def close(self):
        self.pub_socket.close()


publisher = zmq_pub_dict(args.publishport, "rb_laser_error_signal")

ser = serial.Serial()
ser.port = 'COM10'
ser.baudrate = 115200
ser.setDTR(False)
ser.open()

def get_error_signal():
    s = ser.readline()
    try:
        if 'lock' in s:
            error = -9999.0
            err_msg = s
        else:
            error,correction = s.split(',')
            error = float(error)
            correction = float(correction)
            err_msg = ''
    except:
        error = -9999.0
        err_msg = 'unknown error'
    
    return error,correction,err_msg


done = False
while not done:
    try:
        error,correction,err_msg = get_error_signal();

        data_dict = {'error_signal': error, 'correction_signal': correction, 'error_message': err_msg}

        dt = str(time.time())
        publisher.send(data_dict)

    except KeyboardInterrupt:
        ser.close()
        done = True
        publisher.close()
publisher.close()

