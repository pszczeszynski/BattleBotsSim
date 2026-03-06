#pragma once

#define NOMINMAX
#include <windows.h>

#include <string>
#pragma comment(lib, "ws2_32.lib")

#define DEFAULT_BUFLEN 512

class ServerSocket {
 private:
  // The port to listen on
  std::string port;
  // The socket to listen on
  SOCKET listenSocket;
  // Buffer for receiving data  
  char recvbuf[DEFAULT_BUFLEN];
  // Number of bytes actually received
  int recvbuflen = DEFAULT_BUFLEN;
  // The address of the last sender

  int setup_receiving_socket();

 public:
  // The address of the last sender
  // Made public for now so human interface can prevent it from changing if a
  // bad message is received
  SOCKADDR last_sender_addr;
  // The length of the last sender's address
  int last_sender_addr_len;

  std::string receive(int* outError = nullptr);

  // Receive exactly one datagram. Blocks up to timeout_ms (0 = infinite).
  // Updates last_sender_addr for reply_to_last_sender. Returns "" on timeout.
  std::string receive_one(int timeout_ms, int* outError = nullptr);

  // Non-blocking receive of one datagram. Returns "" if none available.
  std::string receive_one_nonblock(int* outError = nullptr);

  void reply_to_last_sender(std::string data);
  ServerSocket(std::string port);
  ~ServerSocket();
};
