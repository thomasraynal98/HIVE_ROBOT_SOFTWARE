from flask import Flask, Response
from mjpeg.server import MJPEGResponse
from mjpeg.client import MJPEGClient

app = Flask(__name__)

client = MJPEGClient(url)

def relay():
    while True:
        buf = client.dequeue_buffer()
        yield memoryview(buf.data)[:buf.used]
        client.enqueue_buffer(buf)

@app.route('/')
def stream():
    return MJPEGResponse(relay())

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)