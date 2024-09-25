import socket

# Server configuration
HOST = '127.0.0.1'
PORT = 11117 

# Create a socket object (UDP socket)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))
print("UDP Server is ready to receive messages...")

try:
    counter = 0
    while True:
        # Open the image file
        with open(f'../../../MachineLearning/TrainingData/TrainingInputs/image_{counter}.jpg', 'rb') as file:
            image_data = file.read()

        # Receive the client's address and initial message
        data, client_addr = server_socket.recvfrom(1024)  # Receiving initial request from client

        # Send the image data size to the client
        server_socket.sendto(len(image_data).to_bytes(4, byteorder='big'), client_addr)

        # Send the image data to the client in chunks
        for i in range(0, len(image_data), 1024):
            server_socket.sendto(image_data[i:i+1024], client_addr)

        # Receive the client's response
        data_received, client_addr = server_socket.recvfrom(12)  # 12 bytes for two 4-byte integers
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
    server_socket.close()
