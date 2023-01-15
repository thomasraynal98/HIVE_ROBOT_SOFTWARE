import asyncio
import logging
import os
import time
import numpy as np
import imageio as iio
import io
from PIL import Image
from server import MjpegServer

logging.basicConfig()
log_level = os.environ.get("LOG_LEVEL", "INFO").upper()
logger = logging.getLogger("mjpeg")
log_level = getattr(logging, log_level)
logger.setLevel(log_level)

class Camera:

    def __init__(self, idx):
        self._idx = idx
        self.reader = iio.get_reader(f'<video{idx}>')

    @property
    def identifier(self):
        return self._idx

    # The camera class should contain a "get_frame" method
    async def get_frame(self):
        '''
        Method to get frames. It returns the encoded jpeg image
        The camera class should have this "get_frame" method
        '''
        
        frame = self.reader.get_next_data()

        # print(Image.ENCODERS)

        buf = io.BytesIO()
        frame = Image.fromarray(frame).resize(size = (352, 240)).save(buf, "jpeg")
        # iio.imwrite(buf, frame, format=".jpg")
        buf.seek(0)
        await asyncio.sleep(1 / 30)
        return buf.read()

    def stop(self):
        self.reader.close()

if __name__ == "__main__":

    # Instantiate Server
    server = MjpegServer()

    # create lookup of routes and different camera objects
    cams = {
            "cam0": Camera(0),
            "cam1": Camera(2)
           }

    for route, cam in cams.items():
        # add routes
        server.add_stream(route, cam)

    try:
        # start server
        server.start()

    except KeyboardInterrupt:
        logger.warning("Keyboard Interrupt, exiting...")
    finally:
        server.stop()
        for cam in cams.values():
            cam.stop()
