import socket

s = socket.socket(
    socket.AF_INET, socket.SOCK_STREAM
)  # Socket will create with TCP and, IP protocols
s.connect(("localhost", 9999))  # Will connect with the server
# Wait and receive messages until stop
try:
    while True:
        try:
            message = s.recv(1024).decode()
            print(f"Server: {message}")
        except socket.error as e:
            connected = False
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            while not connected:
                try:
                    s.connect(("localhost", 9999))
                    connected = True
                except Exception as e:
                    print(f"Error: {e}")
                    pass

except KeyboardInterrupt:
    s.close()  # Close the connection
    print("Connection closed")