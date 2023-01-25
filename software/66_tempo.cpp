// #include <iostream>
// #include <ctime>

// int main() {
//   // Get the current time
//   time_t now = time(0);

//   // Convert the current time to a tm struct
//   tm *ltm = localtime(&now);

//   // Print the date in the format "Month Day, Year"
//   std::cout << ltm->tm_mon + 1 << " " << ltm->tm_mday << ", " << ltm->tm_year + 1900 << std::endl;

//   std::string date = std::to_string(ltm->tm_mday) + "/" + std::to_string(ltm->tm_mon + 1) + "/" + std::to_string(ltm->tm_year + 1900);
//   std::cout << date << std::endl;
//   return 0;
// }

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {
    // Create a socket

    while(true)
    {
      usleep(1000000);
      int sock = socket(AF_INET, SOCK_STREAM, 0);
      if (sock < 0) {
          std::cout << "Error creating socket" << std::endl;
          return 1;
      }

      // Connect to a server
      sockaddr_in server;
      server.sin_family = AF_INET;
      server.sin_addr.s_addr = inet_addr("8.8.8.8"); // Google DNS server
      server.sin_port = htons(53); // DNS port
      int result = connect(sock, (sockaddr*)&server, sizeof(server));
      if (result < 0) {
          std::cout << "Not connected to the internet." << std::endl;
      } else {
          std::cout << "Connected to the internet." << std::endl;
      }

      // Cleanup
      close(sock);
    }
    return 0;
}