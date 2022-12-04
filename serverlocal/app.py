from flask import Flask, Blueprint, render_template, redirect, url_for, request, flash, jsonify, abort
from flask_socketio import SocketIO, emit, join_room, leave_room, close_room
from flask_cors import CORS

socketio = SocketIO(cors_allowed_origins='*')

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})
app.config['SECRET_KEY'] = 'secret!'

socketio.init_app(app)



robot = {}


@socketio.on('connect')
def test_connect():
    print('Client Connected')

@socketio.on('ping')
def ping():
    username = request.sid
    socketio.emit('pong', to=username)

name = ""

@socketio.on('ROBOT_ID')
def handle_message(auth):    
    global robot, name

    name = auth

    print(auth, 'Connected')
    username = request.sid
    room = request.sid
    join_room(room)

    robot[auth] = {'username': username}
    # print(robot)
    # socketio.emit('received', "ok", to=username)


@socketio.on('disconnect')
def disconnect():
    global robot, name

    room = request.sid
    leave_room(room)
    print("Client leave room:" + request.sid)
    close_room(room)
    print("Room: ", room, " is closed.")
    print('Client disconnected')
    # del robot['1-FLASH-MK4_LIGHT-LA_DEFENSE']


@socketio.on('SEND_CMD_TO_REAL_ROBOT')
def SEND_CMD_TO_REAL_ROBOT(data):
    global robot, name
    print(robot)
    socketio.emit('ORDER_MANUALNAV', data, to=robot[name]['username'])

@socketio.on('SEND_AUTONAV')
def SEND_CMD_TO_REAL_ROBOT(data):
    global robot, name
    print(data)
    socketio.emit('ORDER_AUTONAV', data, to=robot[name]['username'])

@socketio.on('SEND_WAITING')
def SEND_CMD_TO_REAL_ROBOT(data):
    global robot, name
    print(data)
    socketio.emit('ORDER_WAITING', data, to=robot[name]['username'])

@socketio.on('SEND_WAITING_END')
def SEND_CMD_TO_REAL_ROBOT(data):
    global robot, name
    print(data)
    socketio.emit('ORDER_WAITING_END', data, to=robot[name]['username'])

@socketio.on('ORDER_HARDWARE')
def SEND_CMD_TO_REAL_ROBOT(data):
    global robot, name
    print(data)
    socketio.emit('ORDER_HARDWARE', data, to=robot[name]['username'])

@socketio.on('EVENT')
def SEND_CMD_TO_REAL_ROBOT(data):
    global robot, name
    print(data)
                                                     
if __name__ == '__main__':
    socketio.run(app, host="0.0.0.0")
