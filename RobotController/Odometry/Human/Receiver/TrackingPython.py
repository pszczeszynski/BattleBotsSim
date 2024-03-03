import socket
import cv2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('localhost', 11117)
sock.bind(server_address)

def get_image():
    # read image from socket
    # Create a UDP socket
    sock.listen()
    conn, addr = sock.accept()
    with conn:
        print(f"Connected by {addr}")
        data = conn.recv(1024)
        if not data:
            return False, None
        print(f"Received: {data.decode()}")
        return True, data.decode()

def return_position(position):
    sock.sendall(position.encode())

def main():
    while True:
        print("Getting image")
        # 1. get the imge
        success, img = get_image()
        if not success:
            print("Error getting image")
            continue
        print("Got image")

        cv2.imshow('image', img)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()
