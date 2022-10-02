import socket
import sys
 
# Create a connection to the server application on port 81
tcp_socket = socket.create_connection(('10.1.1.2', 65000))
print("Connection established")
 
try:
    data = 'spectra_req'
    tcp_socket.sendall(str.encode(data))

    while True:
        spectra = tcp_socket.recv(21000)
        print("Spectra received!")
        print(spectra)
 
finally:
    print("Closing socket")
    tcp_socket.close()