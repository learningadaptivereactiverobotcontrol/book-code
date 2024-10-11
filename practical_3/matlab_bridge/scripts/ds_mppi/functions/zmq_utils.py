#!/usr/bin/env python3

import zmq
import time
import logging

# def zmq_init_recv(socket):
#     val = None
#     while val is None:
#         try:
#             val = socket.recv_pyobj(flags=zmq.DONTWAIT)
#             print(val)
#             status = 1
#         except:
#             print('No input data! (yet) waiting...')
#             time.sleep(0.05)
#             pass
#     return val

def zmq_init_recv(socket, timeout=5000, poll_interval=100):
    """
    Waits to receive a Python object from the socket. 

    Parameters:
    - socket: ZMQ socket to receive data from.
    - timeout: The maximum time (in milliseconds) to wait for data. Default is 10000ms (10 seconds).
    - poll_interval: Interval (in milliseconds) to check for data availability. Default is 100ms.

    Returns:
    - val: The received object or None if timeout occurs.
    """
    val = None
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    start_time = time.time()

    while True:
        # Check for incoming messages with a timeout
        socks = dict(poller.poll(timeout=poll_interval))
        
        if socks.get(socket) == zmq.POLLIN:
            try:
                val = socket.recv_pyobj(flags=zmq.DONTWAIT)
                if val is not None:
                    print("Received data:", val)
                    return val
            except zmq.ZMQError as e:
                print(f"ZMQ Error while receiving data: {e}")
            except Exception as e:
                print(f"Unexpected error while receiving data: {e}")

        if (time.time() - start_time) * 1000 > timeout:
            print(f"No input data received in the last {timeout/1000} seconds.", flush=True)
            print(f"Trying again...", flush=True)
            start_time = time.time()

        time.sleep(poll_interval / 1000)  # Convert milliseconds to seconds

    return val


def zmq_try_recv(val, socket):
    status = 0
    try:
        val = socket.recv_pyobj(flags=zmq.DONTWAIT)
        status = 1
    except:
        pass
    return val, status

def zmq_try_recv_raw(val, socket):
    status = 0
    try:
        val = socket.recv(flags=zmq.DONTWAIT)
        status = 1
    except:
        pass
    return val, status

def init_subscriber(context, address, port, topic_filter=b""):
    """
    Initialize a ZeroMQ subscriber socket.

    Parameters:
    - context: ZeroMQ context.
    - address: Address to connect to (e.g., 'localhost').
    - port: Port to connect to.
    - topic_filter: Topic filter for subscription. Default is an empty string, which subscribes to all topics.

    Returns:
    - socket: Initialized ZeroMQ subscriber socket.
    """
    try:
        # Create a ZeroMQ SUB socket
        socket = context.socket(zmq.SUB)
        
        # Enable conflate option to keep only the latest message
        socket.setsockopt(zmq.CONFLATE, 1)
        
        # Construct the connection string and connect the socket
        connection_str = f"tcp://{address}:{port}"
        socket.connect(connection_str)
        
        # Set the subscription topic filter (default is to subscribe to everything)
        socket.setsockopt(zmq.SUBSCRIBE, topic_filter)
        
        logging.info(f"Subscriber connected to {connection_str} with topic filter: {topic_filter.decode('utf-8')}")
        
    except zmq.ZMQError as e:
        logging.error(f"Failed to initialize subscriber socket: {e}")
        socket = None
    
    return socket


def init_subscriber_bind(context, address, port):
    # socket to receive stuff
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.bind("tcp://%s:%s" % (address, str(port)))
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    return socket


def init_publisher(context, address, port):
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://%s:%s" % (address, str(port)))
    return socket

