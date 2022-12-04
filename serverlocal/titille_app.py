import socketio
import time

sio = socketio.Client()

def check_map(cmd):
    # sio.emit('SEND_CMD_TO_REAL_ROBOT', data = {"map_id" : map_number, "montab": cmd})
    sio.emit('SEND_CMD_TO_REAL_ROBOT', data = cmd)

def check_map2():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "STANDARD", "LONGITUDE": "2.00253", "LATITUDE": "49.04644"})
    # sio.emit('SEND_CMD_TO_REAL_ROBOT', data = cmd)

def check_map2p():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "PARKING", "LONGITUDE": "2.00254", "LATITUDE": "49.04779"})

def check_map3():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "STANDARD", "LONGITUDE": "2.00203", "LATITUDE": "49.04758"})
    # sio.emit('SEND_CMD_TO_REAL_ROBOT', data = cmd)

def check_map4():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "STANDARD", "LONGITUDE": "2.00315", "LATITUDE": "49.04821"})
    # sio.emit('SEND_CMD_TO_REAL_ROBOT', data = cmd)    

def check_map5():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "STANDARD", "LONGITUDE": "2.00310", "LATITUDE": "49.04850"})

def check_map6():
    sio.emit('ORDER_HARDWARE', data = {"ID_BOX" : 1})

def check_map7():
    sio.emit('ORDER_HARDWARE', data = {"ID_BOX" : 2})

def check_map8():
    sio.emit('ORDER_HARDWARE', data = {"ID_BOX" : 3})

def check_map9():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "STANDARD", "LONGITUDE": "2.00097", "LATITUDE": "49.04698"})

def check_map10():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "STANDARD", "LONGITUDE": "2.00355", "LATITUDE": "49.04731"})

def check_map14():
    sio.emit('SEND_AUTONAV', data = {"OPT_DEST" : "STANDARD", "LONGITUDE": "2.00106", "LATITUDE": "49.04774"})

def check_map11():
    sio.emit('SEND_WAITING', data = {"IMPORTANT" : False, "END_TIMESTAMP": 0})

def check_map12():
    t = time.time()
    t_ms = int(t * 1000) + 10000
    sio.emit('SEND_WAITING', data = {"IMPORTANT" : True, "END_TIMESTAMP": t_ms})

def check_map13():
    sio.emit('SEND_WAITING_END', data = {"RIEN" : False})

if __name__ == '__main__':
    connected = False
    map_check = False
    while not connected:
        try:
            sio.connect('http://0.0.0.0:5000')
        except socketio.exceptions.ConnectionError as err:
            print("ConnectionError: ", err)
        else:
            print("Connected!")
            connected = True
            while True:
                x = input()
                if x == "0":
                    cmd = [0, 0, 0]
                    print("mode1")
                    check_map(cmd)
                if x == "1":
                    cmd = [0, 100, 90]
                    print("mode1")
                    check_map(cmd)
                if x == "2":
                    cmd = [100, 0, 90]
                    print("mode1")
                    check_map(cmd)
                if x == "3":
                    cmd = [0, 100, 181]
                    print("mode1")
                    check_map(cmd)
                if x == "4":
                    cmd = [0, 100, 271]
                    print("mode1")
                    check_map(cmd)
                if x == "a":
                    check_map2()
                if x == "b":
                    check_map3()
                if x == "c":
                    check_map4()
                if x == "d":
                    check_map5()
                if x == "A":
                    check_map6()
                    print("BOX A OPEN")
                if x == "B":
                    check_map7()
                if x == "C":
                    check_map8()
                if x == "e":
                    check_map9()
                if x == "f":
                    check_map10()
                if x == "g":
                    check_map14()
                if x == "w":
                    check_map11()
                if x == "w10":
                    check_map12()
                if x == "wend":
                    check_map13()
                if x == "ap":
                    check_map2p()
