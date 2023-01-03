#include <iostream>
#include <ctime>

int main() {
  // Get the current time
  time_t now = time(0);

  // Convert the current time to a tm struct
  tm *ltm = localtime(&now);

  // Print the date in the format "Month Day, Year"
  std::cout << ltm->tm_mon + 1 << " " << ltm->tm_mday << ", " << ltm->tm_year + 1900 << std::endl;

  std::string date = std::to_string(ltm->tm_mday) + "/" + std::to_string(ltm->tm_mon + 1) + "/" + std::to_string(ltm->tm_year + 1900);
  std::cout << date << std::endl;
  return 0;
}