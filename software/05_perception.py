import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import redis


class AppState:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 3
        self.scale = False
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


class OccupancyGrid:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME           = 'Occupancy'

        self.precision          = 0.1 # 1 rectangle = 10cm
        self.LENGHT             = 4
        self.WIDTH              = 4
        self.resolution_per_case= 10

        self.occupancy_grid     = np.zeros([
            int(self.LENGHT / self.precision),
            int(self.WIDTH / self.precision),
            3], dtype = np.uint8)
        self.occupancy_grid.fill(255)

        self.counter_grid       = np.zeros([
            int(self.LENGHT / self.precision),
            int(self.WIDTH / self.precision),
            1], dtype = np.uint8)
        self.counter_grid.fill(0)

        self.camera_position    = np.array([
            0,
            int( (self.LENGHT / self.precision) /2 )], dtype=np.uint8)


class DepthCamera:
    def __init__(self, serial_id: str):
        self.serial_id = serial_id
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial_id)

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.device_product_line = str(device.get_info(rs.camera_info.product_line))
        self.device_serial_number = str(device.get_info(rs.camera_info.serial_number))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

        # Get stream profile and camera intrinsics
        self.profile = self.pipeline.get_active_profile()
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = self.depth_profile.get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height

        # Processing blocks
        self.pc = rs.pointcloud()
        self.decimate = rs.decimation_filter()
        self.decimate.set_option(rs.option.filter_magnitude, 2 ** 3)
        self.colorizer = rs.colorizer()

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()()


def objectHumanDetection(state, occupancy_grid, camera):
    # cv2.namedWindow(occupancy_grid.WIN_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    # cv2.resizeWindow(occupancy_grid.WIN_NAME, camera.w, camera.h)


    # model            = torch.hub.load('ultralytics/yolov5', 'yolov5s')
    # model.classes    = [0]  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
    # device           = torch.device(0)
    # model.to(device)  # i.e. device=torch.device(0)


    out = np.empty((camera.h, camera.w, 3), dtype=np.uint8)
    time_vertice = time.time()

    reds = redis.Redis(host='localhost', port=6379)

    while True:

        occupancy_grid.occupancy_grid.fill(255)
        occupancy_grid.counter_grid.fill(0)

        # Grab camera data
        if not state.paused:
            # Wait for a coherent pair of frames: depth and color
            frames = camera.pipeline.wait_for_frames()

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            depth_frame = camera.decimate.process(depth_frame)
            color_frame = camera.decimate.process(color_frame)

            # Grab new intrinsics (may be changed by decimation)
            depth_intrinsics = rs.video_stream_profile(
                depth_frame.profile).get_intrinsics()
            w, h = depth_intrinsics.width, depth_intrinsics.height

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())


            #region HUMAN DETECTION

            # results = model(color_image)

            # print(results.xyxy[0])

            # humans = results.xyxy[0].cpu().numpy()

            humans_to_redis = [str(int(time.time() * 1000))]
            # if humans.size != 0:

            #     for human in humans:

            #         x  = int(human[0])
            #         y  = int(human[1])
            #         x2 = int(human[2])
            #         y2 = int(human[3])

            #         # cx = (x + x2) // 2
            #         # cy = (y + y2) // 2

            #         # ratio_X = (color_image.shape[0] / h) + 1
            #         # ratio_Y = (color_image.shape[1] / w) + 1
                    
            #         # distance = depth_image[int(cy / ratio_Y), int(cx / ratio_X)]

            #         # human_angle = (30 - (cx * 60 / color_image.shape[1]))

            #         # humans_to_redis.extend([str(str(camera.serial_id)[:3]), 'p', str(distance), str(human_angle)])
                    
            # #endregion


            depth_colormap = np.asanyarray(
                camera.colorizer.colorize(depth_frame).get_data())

            if state.color:
                mapped_frame, color_source = color_frame, color_image
            else:
                mapped_frame, color_source = depth_frame, depth_colormap

            points = camera.pc.calculate(depth_frame)
            camera.pc.map_to(mapped_frame)

            # Pointcloud data to arrays
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
            # print('VERTS -> ', verts.shape)

            print('FPS -> ', 1/ (time.time() - time_vertice))
            time_vertice = time.time()

            for v in verts:
                if(v[2] > 1.0 and v[2] < 3.0):
                    x = int( (occupancy_grid.LENGHT / occupancy_grid.precision) - (v[0] / occupancy_grid.precision + ((occupancy_grid.LENGHT / occupancy_grid.precision)/2)))
                    z = int(v[2] / occupancy_grid.precision)

                    # print(int(x), int(z))
                    if x < int(occupancy_grid.LENGHT / occupancy_grid.precision) and z < int(occupancy_grid.WIDTH / occupancy_grid.precision):
                        occupancy_grid.counter_grid[int(z)][int(x)] += 1


            occupancy_grid.occupancy_grid[occupancy_grid.camera_position[0]][occupancy_grid.camera_position[1]] = [255, 0, 0]


            rows, cols = int(occupancy_grid.WIDTH / occupancy_grid.precision), int(occupancy_grid.LENGHT / occupancy_grid.precision)

            for r in range(rows):
                for c in range(cols):
                    if occupancy_grid.counter_grid[r][c] > occupancy_grid.resolution_per_case:
                        occupancy_grid.occupancy_grid[r][c] = [0 ,0, 0]

                        distance_obstacle = math.sqrt(
                            math.pow((r - occupancy_grid.camera_position[0]), 2) 
                            + 
                            math.pow((c - occupancy_grid.camera_position[1]), 2)
                        ) * occupancy_grid.precision

                        if r != 0 and c != 0:
                            x1 = occupancy_grid.camera_position[0]
                            y1 = occupancy_grid.camera_position[1]
                            x2 = r
                            y2 = c

                            angle_formula = -1*(math.atan2( y2 - y1, x2 - x1 ) * ( 180 / math.pi ))
                            
                            humans_to_redis.extend([str(str(camera.serial_id)[:3]), 'o', str(round(angle_formula,2)), str(round(distance_obstacle,2))])



        
        # cv2.imshow(occupancy_grid.WIN_NAME, occupancy_grid.occupancy_grid)

        msg_to_redis = "|".join(humans_to_redis)
        # print(msg_to_redis)
        reds.set('ENV_CAM1_OBSTACLE', msg_to_redis)
                
        # key = cv2.waitKey(1)

        # if key in (27, ord("q")):
        #     break

    # Stop streaming
    camera.pipeline.stop()

if __name__ == "__main__":

    state = AppState()
    occupancy_grid = OccupancyGrid()
    camera = DepthCamera('105322250429')

    objectHumanDetection(state, occupancy_grid, camera)