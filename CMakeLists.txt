CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(HIVE_ROBOT_SYSTEM VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(pixhawk/mavlink/include/mavlink/v2.0)

# set good gcc version.
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -L/usr/local/lib -L/usr/bin/openssl")

# found package.
FIND_PACKAGE(OpenCV 4.2 REQUIRED)
FIND_PACKAGE(websocketpp REQUIRED)

# FIND_PACKAGE(OpenSSL REQUIRED)
# [!] Comment for HTTPS.
FIND_PACKAGE(sioclient REQUIRED)

# # PROGRAMME 01 : GESTION DU SYSTEME YAML ET REDIS + DEBUG
ADD_EXECUTABLE(01_SYSTEM_MANAGER 
software/01_system.cpp
src/00_function.cpp
include/00_function.h)

TARGET_LINK_LIBRARIES(01_SYSTEM_MANAGER
    pthread
    hiredis
    redis++
    ${OpenCV_LIBS})

# PROGRAMME 02 : GESTION DES COMMUNICATIONS AVEC LES MCU's (HARDWARE)
ADD_EXECUTABLE(02_HARDWARE 
software/02_hardware.cpp
src/00_function.cpp
include/00_function.h
src/00_communication.cpp
include/00_communication.h
pixhawk/mavlink/include/mavlink/v2.0
src/autopilot_interface.cpp
include/autopilot_interface.h
src/serial_port.cpp
include/serial_port.h)

TARGET_LINK_LIBRARIES(02_HARDWARE
    pthread
    hiredis
    redis++
    serial
    sioclient
    ${OpenCV_LIBS})

# PROGRAMME 03 : GESTION DES COMMUNICATIONS AVEC LE SERVER.
ADD_EXECUTABLE(03_SERVER 
software/03_server.cpp
src/00_function.cpp
include/00_function.h
src/00_communication.cpp
include/00_communication.h
src/00_navigation.cpp 
include/00_navigation.h)

TARGET_LINK_LIBRARIES(03_SERVER
    pthread
    hiredis
    redis++
    serial
    # [!] Decomment for HTTPS.
    # ssl
    # crypto 
    # /usr/local/lib/libsioclient_tls.so
    # [!] Comment for HTTPS.
    sioclient
    ${OpenCV_LIBS})

# PROGRAMME 04 : ANALYSE DE LA SITUATION EST CHOIX COMMANDE MOTOR ROUES.
ADD_EXECUTABLE(04_NAVIGATION
software/04_navigation.cpp
src/00_function.cpp
include/00_function.h
src/00_communication.cpp
include/00_communication.h
src/00_navigation.cpp 
include/00_navigation.h)

TARGET_LINK_LIBRARIES(04_NAVIGATION
    pthread
    hiredis
    redis++
    sioclient
    serial
    ${OpenCV_LIBS})

# # PROGRAMME 66 : WTF.
# ADD_EXECUTABLE(66_EXEMPLE
# software/66_tempo.cpp)

# TARGET_LINK_LIBRARIES(66_EXEMPLE
#     pthread
#     hiredis
#     redis++
#     sioclient
#     # /usr/local/lib/libsioclient_tls.so
#     crypto
#     ssl
#     serial)
