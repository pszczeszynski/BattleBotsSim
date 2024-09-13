import socket

# Server configuration
HOST = '127.0.0.1'
PORT = 11117 

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Server is listening...")

# Accept a connection
client_socket, addr = server_socket.accept()
print(f"Connection from {addr} established.")

try:
    counter = 0
    while True:
        # Open the image file
        with open(f'../../../MachineLearning/TrainingData/TrainingInputs/image_{counter}.jpg', 'rb') as file:
        # with open(f'image_1.jpg', 'rb') as file:
            image_data = file.read()

        # Send the image data size
        client_socket.send(len(image_data).to_bytes(4, byteorder='big'))

        # Send the image data to the client
        client_socket.sendall(image_data)

        data_received = client_socket.recv(12)  # 8 bytes for two 4-byte integers
        current_mode = int.from_bytes(data_received[:4], byteorder='big')
        x = int.from_bytes(data_received[:4], byteorder='big')
        y = int.from_bytes(data_received[4:], byteorder='big')
        print(f"Current mode: {current_mode}")
        if x > 0 and y > 0:
            print(f'Position x: {x}, Position y: {y}')

        counter += 1
        counter %= 40

except KeyboardInterrupt:
    # Close the server socket
    client_socket.close()
    server_socket.close()
