import json
import cv2
import mediapipe as mp
import numpy as np
# from unity_connect import fetchFrameUnity,sendToUnity_Direction, sendToUnity_MoveCommand

RIGHT_EYE_LANDMARKS = [160, 161, 163, 144, 145, 153, 154, 157, 158, 159]
RIGHT_IRIS_LANDMARKS = [472, 469, 470, 471]

LEFT_EYE_LANDMARKS = [384, 385, 386, 387, 388, 390, 373, 374, 380, 381]
LEFT_IRIS_LANDMARKS = [474, 475, 476, 477]

# From -1 to 1
DEFAULT_LEFT_RATIO_THRESH = -0.18
DEFAULT_RIGHT_RATIO_THRESH = 0.18

DIR_BUFFER_WINDOW = 2
OVERLAY_TRANSPARENCY = 0.1



class GazeCamera:

    def __init__(self):
        self.ema_left = 0
        self.ema_right = 0
        self.ema_alpha = 0.3  # smoothing factor

        self.cam = cv2.VideoCapture(0)
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks=True)

        self.config = {}
        with open('config.json', 'r') as f:
            self.config = json.load(f)

        cv2.namedWindow("Steering_Control")

    @staticmethod
    def determine_direction(pupil_x, eye_x_low, eye_x_high):
        direction_ratio = (pupil_x - eye_x_low) / (eye_x_high - eye_x_low)

        # Convert from [0, 1] range into [-1, 1] range so it's
        # easier to use in Unity.
        direction_ratio = (2 * direction_ratio) - 1

        return direction_ratio
    
    @staticmethod
    def get_eye_direction_ratio(self, frame, landmarks, frame_w, frame_h, right=False, debug=False):
        if (right):
            eye_landmarks = RIGHT_EYE_LANDMARKS
            iris_landmarks = RIGHT_IRIS_LANDMARKS
        else:
            eye_landmarks = LEFT_EYE_LANDMARKS
            iris_landmarks = LEFT_IRIS_LANDMARKS

        # Eye
        pts = []
        for i in eye_landmarks:
            x = int(landmarks[i].x * frame_w)
            y = int(landmarks[i].y * frame_h)
            pts.append((x, y))
        pts = np.array(pts, np.int32)
        x, y, w, h = cv2.boundingRect(pts)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)  # Draw rectangle around eye

        if debug:
            # Draw all eye landmarks
            hull_indices = cv2.convexHull(pts, returnPoints=False)
            for point in pts:
                cv2.circle(frame, tuple(point), 2, (255, 255, 255), -1)
            hull_points = [pts[index] for index in hull_indices]
            cv2.drawContours(frame, [np.array(hull_points)], 0, (255, 255, 255), 1)

        eye_x_low = x
        eye_x_high = x+w

        # Iris
        pts = []
        for i in iris_landmarks:
            x = int(landmarks[i].x * frame_w)
            y = int(landmarks[i].y * frame_h)
            pts.append((x, y))
        pts = np.array(pts, np.int32)
        center, radius = cv2.minEnclosingCircle(pts)
        center = tuple(map(int, center))
        radius = int(radius/2)
        cv2.circle(frame, center, radius, (0, 0, 255), 1)  # Draw circle around iris

        direction_ratio = self.determine_direction(center[0], eye_x_low, eye_x_high)

        return frame, direction_ratio

    # @staticmethod
    # def get_eye_detailed_variables(self, frame, landmarks, frame_w, frame_h, right=False, debug=False):
    #     if (right):
    #         eye_landmarks = RIGHT_EYE_LANDMARKS
    #         iris_landmarks = RIGHT_IRIS_LANDMARKS
    #     else:
    #         eye_landmarks = LEFT_EYE_LANDMARKS
    #         iris_landmarks = LEFT_IRIS_LANDMARKS

    #     # Eye
    #     pts = []
    #     for i in eye_landmarks:
    #         x = int(landmarks[i].x * frame_w)
    #         y = int(landmarks[i].y * frame_h)
    #         pts.append((x, y))
    #     pts = np.array(pts, np.int32)
    #     x, y, w, h = cv2.boundingRect(pts)
    #     cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)  # Draw rectangle around eye

    #     if debug:
    #         # Draw all eye landmarks
    #         hull_indices = cv2.convexHull(pts, returnPoints=False)
    #         for point in pts:
    #             cv2.circle(frame, tuple(point), 2, (255, 255, 255), -1)
    #         hull_points = [pts[index] for index in hull_indices]
    #         cv2.drawContours(frame, [np.array(hull_points)], 0, (255, 255, 255), 1)

    #     eye_x_low = x
    #     eye_x_high = x+w

    #     # Iris
    #     pts = []
    #     for i in iris_landmarks:
    #         x = int(landmarks[i].x * frame_w)
    #         y = int(landmarks[i].y * frame_h)
    #         pts.append((x, y))
    #     pts = np.array(pts, np.int32)
    #     center, radius = cv2.minEnclosingCircle(pts)
    #     center = tuple(map(int, center))
    #     radius = int(radius/2)
    #     cv2.circle(frame, center, radius, (0, 0, 255), 1)  # Draw circle around iris

    #     direction_ratio = self.determine_direction(center[0], eye_x_low, eye_x_high)

    #     return frame, direction_ratio, eye_x_low, eye_x_high, center[0]
    
    @staticmethod
    def write_direction_on_frame(direction, frame):
        if direction < 0:
            cv2.putText(frame, "left", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA,)
        elif direction > 0:
            cv2.putText(frame, "right", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

        return frame

    @staticmethod
    def draw_control_panel_overlay(input_image, gaze_direction=None):
        # Create control panel the same size as input image to overlay on top
        control_panel = np.zeros_like(input_image, np.uint8) 

        input_h, input_w, input_c = input_image.shape

        # Draw lines to divide the panel into 3
        cv2.line(control_panel, # Image
                (input_w//3, 0), # Start point
                (input_w//3, input_h), # End point
                (255,255,255), # color = white
                3 # Thickness
                )
        cv2.line(control_panel, # Image
                (input_w*2//3, 0), # Start point
                (input_w*2//3, input_h), # End point
                (255,255,255), # color = white
                3 # Thickness
                )
        
        # Arrows and text for UX (turn directions)
        cv2.arrowedLine(control_panel,
                        (input_w//3-100, input_h//2),
                        (input_w//3-300, input_h//2),
                        (255,255,255),
                        10,
                        tipLength=0.8
                        )
        cv2.arrowedLine(control_panel,
                        (input_w*2//3+100, input_h//2),
                        (input_w*2//3+300, input_h//2),
                        (255,255,255),
                        10,
                        tipLength=0.8
                        )
        
        # Add a rectangle to highlight left/right column
        if gaze_direction<0:
            cv2.rectangle(control_panel, # Image
                        (0, 0), # Start point
                        (input_w//3, input_h), # End point
                        (255,255,255), # color = white
                        -1 # Thickness -1 to fill
                        )
        elif gaze_direction>0:
            cv2.rectangle(control_panel, # Image
                        (input_w*2//3, 0), # Start point
                        (input_w, input_h), # End point
                        (255,255,255), # color = white
                        -1 # Thickness -1 to fill
                        )
        # Else, do nothing (TODO: Discuss if we want a stop panel)

        # Return overlay image
        return control_panel

    @staticmethod
    def draw_steering_control_frame(self, input_image, direction, transparency=OVERLAY_TRANSPARENCY):
        # Draw control panel overlay
        control_panel = self.draw_control_panel_overlay(input_image, direction)

        # Piece together the steering control image
        steering_frame = cv2.addWeighted(control_panel, transparency, input_image, 1-transparency, 0)

        return steering_frame

    ### Provide a 0<screen_ratio_x<1 to display a point on the screen, where 0 is left edge and 1 is right edge
    @staticmethod
    def draw_pointer(frame, screen_ratio_x):
        frame_h, frame_w, _ = frame.shape
        center = (round(frame_w * screen_ratio_x), round(frame_h * 0.5))
        # Radius get smaller by time
        radius = 5
        return cv2.circle(frame, center, radius, (0, 255, 0), -1)
    
    def loop(self):
        _, frame = self.cam.read()
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output = self.face_mesh.process(rgb_frame)
        landmark_points = output.multi_face_landmarks
        frame_h, frame_w, _ = frame.shape
        direction = 0
        if landmark_points:
            landmarks = landmark_points[0].landmark
            frame, right_direction_ratio = self.get_eye_direction_ratio(
                self, frame, landmarks, frame_w, frame_h, True, self.config['debug'])
            self.ema_right = (self.ema_alpha * right_direction_ratio) + ((1 - self.ema_alpha) * self.ema_right)
            right_direction_ratio = self.ema_right

            frame, left_direction_ratio = self.get_eye_direction_ratio(
                self, frame, landmarks, frame_w, frame_h, False, self.config['debug'])
            self.ema_left = (self.ema_alpha * left_direction_ratio) + ((1 - self.ema_alpha) * self.ema_left)
            left_direction_ratio = self.ema_left

            # Personal offset from calibration
            #left_direction_ratio += config['left_offset']
            #right_direction_ratio += config['right_offset']

            cv2.putText(frame, f"{round(right_direction_ratio, 4)}", (200, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA,)
            cv2.putText(frame, f"{round(left_direction_ratio, 4)}", (100, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA,)

            direction = (left_direction_ratio+ right_direction_ratio) / 2
            raw_direction = direction + self.config["center_offset"]
            direction = direction + self.config["center_offset"]
            # Prevent jittering / drifting
            if (direction > self.config['left_threshold'] and direction < self.config['right_threshold']):
                direction = 0
            if(direction > self.config['right_threshold']):
                direction = 0.15
            if(direction < self.config['left_threshold']):
                direction = -0.15

            frame = self.write_direction_on_frame(direction, frame)
            control_frame = self.draw_steering_control_frame(self, frame, direction)

            # sendToUnity_Direction(direction * 100)
            control_frame = self.draw_pointer(control_frame, raw_direction+0.5)

            cv2.imshow("Steering_Control", control_frame)
            cv2.waitKey(1)

        # if self.config['unity_camera']:
        #     try:
        #         unity_frame_dat = fetchFrameUnity()
        #         unity_frame_array = np.frombuffer(unity_frame_dat, dtype=np.uint8)
        #         u_frame = cv2.imdecode(unity_frame_array, cv2.IMREAD_UNCHANGED)
        #         u_frame_h, u_frame_w, _ = u_frame.shape
        #         midpoint = u_frame_w/2
        #         start_point = (int(midpoint), int(u_frame_h))
        #         # End coordinate
        #         end_point = (int(midpoint+(direction*midpoint)),int( u_frame_h/2) )
        #         # Red color in BGR
        #         color = (0, 0, 255)
        #         # Line thickness of 9 px
        #         thickness = 9
        #         # Using cv2.arrowedLine() method
        #         # Draw a red arrow line
        #         # with thickness of 9 px and tipLength = 0.5
                
        #         #u_frame = cv2.arrowedLine(u_frame, start_point, end_point, color, thickness, tipLength = 0.5)
        #         u_frame  = self.draw_steering_control_frame(u_frame, direction)
        #         interpolated = (raw_direction - self.config['left_threshold'])/(self.config['right_threshold'] - self.config['left_threshold'])
        #         interpolated =  interpolated*(0.7-0.3) + 0.3
        #         u_frame= self.draw_pointer(u_frame, interpolated)
        #         u_frame= self.draw_pointer(u_frame, 0.3)
        #         u_frame= self.draw_pointer(u_frame, 0.7)
        #         cv2.imshow("Unityfetchertester",u_frame)
        #     except Exception:
        #         print("Unity not up")


# g = GazeCamera()
# while True:
#     g.loop()