#!/usr/bin/env python3

import bluetooth
import time

# Define the MAC address of the paired Bluetooth device
device_address = '00:0C:BF:18:44:14'  # Replace with the actual MAC address of your paired device
port = 1  # Bluetooth serial port (usually 1 for SPP devices)
socket = None  # Define the global variable explicitly


# Function to create and connect the Bluetooth RFCOMM socket
"""
def create_socket():
    socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    try:
        socket.connect((device_address, port))
        print("Connected successfully!")
        return socket
    except bluetooth.BluetoothError as e:
        print(f"Error connecting to the device: {e}")
        return None
"""
def create_socket():
    """ Function to create a new Bluetooth socket and connect it. """
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    try:
        sock.connect((device_address, port))
        sock.setblocking(False)  # Set to non-blocking mode
        print("Connected successfully!")
        return sock
    except bluetooth.BluetoothError as e:
        print(f"Error connecting to the device: {e}")
        return None

"""
def receive_data():
    try:
        # Receive data from the Bluetooth device
        data = socket.recv(5)
        if data:
            data_list = list(data)
            print(f"Received data (list): {data_list}")
            return data_list
        else:
            print("No data received.")
            return [0,0,0,0,0]
    except bluetooth.BluetoothError as e:
        print(f"Error receiving data: {e}")
        close_and_reopen_socket()
        return None
"""
def receive_data():
    """ Function to handle receiving data from the Bluetooth device. """
    global socket
    try:
        data = socket.recv(5)
        if data:
            data_list = list(data)
            print(f"Received data (list): {data_list}")
            return data_list
        else:
            print("No data received.")
            return [0,0,0,0,0]
    except bluetooth.BluetoothError:
        # No data available or other non-blocking error
        return [0,0,0,0,0]


def send_data(message):
    """ Function to handle sending data to the Bluetooth device. """
    try:
        # Send data to the Bluetooth device
        socket.send(message)
        print(f"Sent message: {message}")
    except bluetooth.BluetoothError as e:
        print(f"Error sending data: {e}")
        close_and_reopen_socket()

def close_and_reopen_socket():
    """ Function to close the existing socket and reopen it. """
    global socket
    try:
        if socket:
            print("Closing the existing socket...")
            socket.close()
        socket = create_socket()  # Recreate and reconnect the socket
    except bluetooth.BluetoothError as e:
        print(f"Error reopening socket: {e}")

def connect_remote():
    global socket
    try:
        print("Connecting to the device...")
        if socket is None:
            socket = create_socket()  # Assuming this may raise an exception
            print("Connection successful.")
        else:
            print("Already connected.")
        return True
    except bluetooth.BluetoothError as e:
        print(f"Bluetooth error: {e}")
        socket = None  # Ensure socket is not left in a bad state
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        socket = None
        return False

def close_connection():
    # Close the Bluetooth socket when done
    if socket:
        socket.close()
    print("Connection closed.")

def main():
    connect_remote()
    send_data("Hello from Raspberry Pi!")

    try:
        while True:
            remote_data = receive_data()
            if remote_data:
                # Process the data as needed
                print(f"Processed received data: {remote_data}")

            send_data("Keep Alive Message")
            time.sleep(0.5)  # Short sleep to prevent CPU hogging
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        close_connection()

        

if __name__ == "__main__":
    main()