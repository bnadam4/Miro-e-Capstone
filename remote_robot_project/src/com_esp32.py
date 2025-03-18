#!/usr/bin/env python3

import requests
import time
import threading
import random

# ESP32 IP address (replace with your ESP32 IP)
esp32_ip = "http://192.168.1.103"

def send_data(data):
    try:
        # Send the 5 bytes to the ESP32
        response = requests.post(f"{esp32_ip}/senddata", json=data, timeout=5)
        response.raise_for_status()  # Raise an exception for HTTP errors (4xx or 5xx)
        
        # Check if the response is successful
        if response.status_code == 200:
            print("Data successfully sent to ESP32!")
        else:
            print(f"Error sending data: {response.text}")
    except requests.exceptions.RequestException as e:
        # Handle any errors that occur during the request
        print(f"Failed to send data: {e}")


def get_data():
    try:
        # Fetch the 5 bytes from the ESP32
        response = requests.get(f"{esp32_ip}/getdata", timeout=5)  # Add a timeout for safety
        response.raise_for_status()  # Raise an exception for HTTP errors (4xx or 5xx)
        
        # Parse the JSON response
        data = response.json()
        
        # Convert all the data values to integers
        data1 = int(data['data1'])
        data2 = int(data['data2'])
        data3 = int(data['data3'])
        data4 = int(data['data4'])
        data5 = int(data['data5'])


        # Return the data as a dictionary of integers
        return {'data1': data1, 'data2': data2, 'data3': data3, 'data4': data4, 'data5': data5}
    
    except requests.exceptions.RequestException as e:
        # Handle any errors that occur during the request
        print(f"Failed to get data: {e}")
        return None
"""
def get_data():
    try:
        # Fetch the 5 bytes from the ESP32
        response = requests.get(f"{esp32_ip}/getdata", timeout=5)  # Add a timeout for safety
        response.raise_for_status()  # Raise an exception for HTTP errors (4xx or 5xx)
        
        # Parse the JSON response
        data = response.json()
        print("Data from ESP32:")
        print(f"data1: {data['data1']}")
        print(f"data2: {data['data2']}")
        print(f"data3: {data['data3']}")
        print(f"data4: {data['data4']}")
        print(f"data5: {data['data5']}")
        return data
    except requests.exceptions.RequestException as e:
        # Handle any errors that occur during the request
        print(f"Failed to get data: {e}")
        return None
"""
# Function to send data every 5 seconds
def send_data_periodically():
    while True:
        # Example data to send to the ESP32 (5 bytes)
        data_to_send = [1, 2, 0, 0, 1]
        send_data(data_to_send)
        
        # Sleep for 5 seconds before sending again
        time.sleep(5)

# Function to get data continuously
def get_data_continuously():
    while True:
        get_data()
        time.sleep(1)  # Optionally adjust this sleep time if needed

# Example of usage
if __name__ == "__main__":
    # Create threads for sending and receiving data
    send_thread = threading.Thread(target=send_data_periodically)
    receive_thread = threading.Thread(target=get_data_continuously)
    
    # Start the threads
    send_thread.start()
    receive_thread.start()

    # Wait for threads to complete (they run indefinitely)
    send_thread.join()
    receive_thread.join()

