import cv2
import numpy as np
import pyautogui
###"""A UI that tracks the horizontal position of mouse """


DIR_BUFFER_WINDOW = 2
DEFAULT_SPEED = 5
SPEEDUP_MULTIPLIER = 1.0
OVERLAY_TRANSPARENCY = 0.1
TURN_RATE_MULTIPLIER = 100
TURN_THRESHOLD_LEFT = 0.3
TURN_THRESHOLD_RIGHT = 0.7


class GazeCursor:

    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        self.speed = DEFAULT_SPEED
        self.monitor_shape_w, self.monitor_shape_h = pyautogui.size()
        self.config = {}
        _, frame = self.cam.read()
        self.frame = cv2.flip(frame, 1)
        cv2.namedWindow("Steering_Control")

        
    def get_cursor_position(self):
        return pyautogui.position()

    def write_direction_on_frame(self, direction, frame):
        if direction < 0:
            cv2.putText(frame, "left", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA,)
        elif direction > 0:
            cv2.putText(frame, "right", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

        return frame

    def draw_control_panel_overlay(self, input_image, gaze_direction=None):
        # Create control panel the same size as input image to overlay on top
        control_panel = np.zeros_like(input_image, np.uint8) 

        input_h, input_w, input_c = input_image.shape

        # Draw lines to divide the panel into 3
        cv2.line(control_panel, # Image
                 (input_w//5, 0), # Start point
                 (input_w//5, input_h), # End point
                 (255,255,255), # color = white
                 3 # Thickness
                 )
        cv2.line(control_panel, # Image
                 (input_w*4//5, 0), # Start point
                 (input_w*4//5, input_h), # End point
                 (255,255,255), # color = white
                 3 # Thickness
                 )
    
        # Arrows and text for UX (turn directions)
        cv2.arrowedLine(control_panel,
                        (input_w//5-100, input_h//2),
                        (input_w//5-270, input_h//2),
                        (255,255,255),
                        10,
                        tipLength=0.8
                        )
        cv2.arrowedLine(control_panel,
                        (input_w*4//5+100, input_h//2),
                        (input_w*4//5+270, input_h//2),
                        (255,255,255),
                        10,
                        tipLength=0.8
                        )
     
        # Add a rectangle to highlight left/right column
        if gaze_direction<TURN_THRESHOLD_LEFT:
            cv2.rectangle(control_panel, # Image
                         (0, 0), # Start point
                         (input_w//5, input_h), # End point
                         (255,255,255), # color = white
                         -1 # Thickness -1 to fill
                         )
        elif gaze_direction> TURN_THRESHOLD_RIGHT:
            cv2.rectangle(control_panel, # Image
                         (input_w*4//5, 0), # Start point
                         (input_w, input_h), # End point
                         (255,255,255), # color = white
                         -1 # Thickness -1 to fill
                         )
        # Else, do nothing (TODO: Discuss if we want a stop panel)

        # Return overlay image
        return control_panel

    def draw_steering_control_frame(self, input_image, direction, transparency=OVERLAY_TRANSPARENCY):
        # Draw control panel overlay
        control_panel = self.draw_control_panel_overlay(input_image, direction)

        # Piece together the steering control image
        steering_frame = cv2.addWeighted(control_panel, transparency, input_image, 1-transparency, 0)

        return steering_frame

    ### Provide a 0<screen_ratio_x<1 to display a point on the screen, where 0 is left edge and 1 is right edge
    def draw_pointer(self, frame, screen_ratio_x):
        frame_h, frame_w, _ = frame.shape
        center = (round(frame_w * screen_ratio_x), round(frame_h * 0.5))
        # Radius get smaller by time
        radius = 5
        return cv2.circle(frame, center, radius, (0, 255, 0), -1)

    
    def loop(self):
        cursor = self.get_cursor_position()
        cursor_x = cursor[0]
        cursor_y = cursor[1]
        cursor_x_normalized = (cursor_x)/self.monitor_shape_w 
        cursor_y_normalized= (cursor_y)/self.monitor_shape_h 
        cursor_x_normalized_raw = cursor_x_normalized
        print("cursor y {}".format(cursor_y_normalized))
        
        if(cursor_x_normalized > TURN_THRESHOLD_LEFT and cursor_x_normalized < TURN_THRESHOLD_RIGHT):
            cursor_x_normalized = 0.5

        if(cursor_y_normalized<0.2):
            self.speed = DEFAULT_SPEED * SPEEDUP_MULTIPLIER
        else:
            self.speed = DEFAULT_SPEED
        
        turn_rate = (cursor_x_normalized - 0.5) * TURN_RATE_MULTIPLIER
        #sendToUnity_targetSpeedAndDirection(speed, turn_rate)
        print("Rotating {}".format(turn_rate))
        

        _, frame = self.cam.read()
        u_frame = self.draw_steering_control_frame(frame, cursor_x_normalized)
        u_frame= self.draw_pointer(u_frame, cursor_x_normalized_raw)
        
        DEBUG = False
        if DEBUG:
            u_frame_h, u_frame_w, _ = u_frame.shape
            midpoint = u_frame_w/2
            start_point = (int(midpoint), int(u_frame_h/2))
            end_point = (int(cursor_x_normalized_raw*u_frame_w),int( u_frame_h/2) )
            color = (0, 0, 255)
            thickness = 9
            u_frame = cv2.arrowedLine(u_frame, start_point, end_point, color, thickness, tipLength = 0.5)

        cv2.imshow("main_gazetrack_frame",u_frame)
        cv2.waitKey(1)
        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #    break
        
        return turn_rate, self.speed
        


# g = GazeCursor()
# while True:
#     g.loop()