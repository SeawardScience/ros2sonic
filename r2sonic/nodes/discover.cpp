#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <thread>

#define SYSTEMS 4
#define DAQS 8
#define UDP_PORT_CONFIG (0x8000 + 'R2')  // Port 53810
#define GUI_PORT 53810

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#pragma pack(push, 1)
struct R2DD {
  u32 PacketName;     // 'R2DD'
  u32 gui_ip;
  u16 gui_baseport;
  u16 gui_spare;
  u32 spare1[8];
};

struct R2DI {
  u32 PacketName;     // 'R2DI'
  u32 SerialNumber[3];
  u32 PartNumber[3];
  u32 ModelNumber[3];
  u32 spare1[8];
};
#pragma pack(pop)

u32 ip_to_u32(const std::string& ip) {
  in_addr addr;
  inet_pton(AF_INET, ip.c_str(), &addr);
  return ntohl(addr.s_addr);
}

void print_ascii_field(const u32 field[3]) {
  char buffer[13] = {};
  memcpy(buffer, field, 12);
  std::cout << buffer;
}

int main() {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("socket");
    return 1;
  }

  // Allow broadcast
  int broadcast = 1;
  setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

  // Bind to GUI port
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = INADDR_ANY;
  local.sin_port = htons(GUI_PORT);
  if (bind(sock, (sockaddr*)&local, sizeof(local)) < 0) {
    perror("bind");
    return 1;
  }

  // Prepare R2DD packet
  R2DD pkt{};
  pkt.PacketName = htonl('R2DD');
  pkt.gui_ip = htonl(ip_to_u32("10.0.1.102"));  // your local IP
  pkt.gui_baseport = htons(GUI_PORT);

  // Broadcast address
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_port = htons(UDP_PORT_CONFIG);
  inet_pton(AF_INET, "10.255.255.255", &remote.sin_addr);  // broadcast address for subnet

  if (sendto(sock, &pkt, sizeof(pkt), 0, (sockaddr*)&remote, sizeof(remote)) < 0) {
    perror("sendto");
    return 1;
  }

  std::cout << "Sent R2DD packet, waiting for responses...\n";

  fd_set readfds;
  timeval timeout{1, 0};  // 1 second timeout

  while (true) {
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);
    int sel = select(sock + 1, &readfds, nullptr, nullptr, &timeout);
    if (sel <= 0) break;

    if (FD_ISSET(sock, &readfds)) {
      R2DI response{};
      sockaddr_in src{};
      socklen_t slen = sizeof(src);
      ssize_t len = recvfrom(sock, &response, sizeof(response), 0, (sockaddr*)&src, &slen);
      if (len == sizeof(R2DI) && ntohl(response.PacketName) == 'R2DI') {
        std::cout << "Response from " << inet_ntoa(src.sin_addr) << ": ";
        std::cout << "Serial: "; print_ascii_field(response.SerialNumber);
        std::cout << "  Part: "; print_ascii_field(response.PartNumber);
        std::cout << "  Model: "; print_ascii_field(response.ModelNumber);
        std::cout << '\n';
      }
    }
  }

  close(sock);
  return 0;
}
