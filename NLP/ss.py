import socket
import time

# Define host and port
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 65432        # The port used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect to the server
s.connect((HOST, PORT))

# Send data to the server
s.sendall(b'HELLO;5;')

# Receive data from the server
#data = s.recv(1024)

#print('Received:', data.decode())
time.sleep(1)  # Sleep for a second before sending the next message
