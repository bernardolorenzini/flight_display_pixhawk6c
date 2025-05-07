import cv2
import sys

import numpy as np
import skimage.measure
from math import atan2, cos, sin, pi, degrees, radians
from numpy.linalg import norm

from crop_and_scale import get_cropping_and_scaling_parameters, crop_and_scale
from draw_display import draw_horizon

import time

# for pwm in [1000, 1500, 2000, 1500]:
#     print(f"Setting servo to {pwm}")
#     move_servo(1, pwm)
#     time.sleep(1.5)

def normalize_roll(angle):
    if angle is None:
        return None
    return angle - 360 if angle > 180 else angle




POOLING_KERNEL_SIZE = 5
FULL_ROTATION = 360
crop_and_scale_parameters = get_cropping_and_scaling_parameters((1280,720), (100,100))

CROP_AND_SCALE_PARAM = crop_and_scale_parameters

from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPainter, QColor, QFont
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from pymavlink import mavutil



#### here is mavlink side of code
# Lista de possíveis portas de telemetria
telemetry_ports = ["/dev/tty.usbmodem146401", "/dev/tty.usbmodem146403"]

# Tenta conectar à Pixhawk via MAVLink
master = None
for port in telemetry_ports:
    try:
        print(f"Tentando conectar na porta {port}...")
        master = mavutil.mavlink_connection(port, baud=57600)
        print("Aguardando heartbeat...")
        master.wait_heartbeat()
        print(f"Conectado! System: {master.target_system}, Component: {master.target_component}")
        break  # Sai do loop se a conexão for bem-sucedida
    except Exception as e:
        print(f"Falha ao conectar em {port}: {e}")

if master is None:
    print("Erro: Nenhuma porta de telemetria disponível.")
    sys.exit(1)

# Attitude Data
attitude_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

# Placeholder for battery data (voltage)
battery_data = {"voltage": 12.0}  # Placeholder for battery voltage, in volts

# Update battery data from MAVLink
def update_battery_data():
    global battery_data
    msg = master.recv_match(type='BATTERY_STATUS', blocking=False)
    if msg:
        battery_data["voltage"] = msg.voltages[0] / 1000.0  # Convert millivolts to volts
    QTimer.singleShot(50, update_battery_data)

class AttitudeIndicator(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(800, 800)
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateAttitude)
        self.timer.start(50)  # Update every 50ms

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluOrtho2D(-1, 1, -1, 1)
        glMatrixMode(GL_MODELVIEW)

 
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        roll_angle = -np.degrees(attitude_data["roll"])  # Get roll angle in degrees
        pitch_offset = -np.degrees(attitude_data["pitch"]) / 90

        # Draw sky and ground
        glPushMatrix()
        glRotatef(roll_angle, 0, 0, 1)
        glTranslatef(0, pitch_offset, 0)

        # Sky
        glColor3f(0.0, 0.5, 1.0)
        glBegin(GL_QUADS)
        glVertex2f(-1, 0)
        glVertex2f(1, 0)
        glVertex2f(1, 1)
        glVertex2f(-1, 1)
        glEnd()

        # Ground
        glColor3f(0.5, 0.25, 0.0)
        glBegin(GL_QUADS)
        glVertex2f(-1, 0)
        glVertex2f(1, 0)
        glVertex2f(1, -1)
        glVertex2f(-1, -1)
        glEnd()

        glPopMatrix()

        # Disable depth test for overlay elements
        glDisable(GL_DEPTH_TEST)

        # Draw UI elements
        self.drawRollMarkers()
        self.drawHeadingScale()
        self.drawAltitudeIndicator(battery_data["voltage"])
        self.drawSpeedIndicator(0)  # Placeholder speed
        self.drawWindIndicator(10, 45)
        self.drawFlightMode("TEST")

        # self.drawServoPosition(roll_angle)


        # 🔴 🔴 🔴 🔴 🔴 🔴 🔴 🔴 🔴 🔴 🔴 🔴 Check roll angle and display warning if needed
        if abs(roll_angle) > 70:
            print(f"ROLL WARNING: {roll_angle:.2f}°")  # Log the warning in the console
            self.drawWarningSign()  # Draw the warning sign

        # Re-enable depth test for next frame
        glEnable(GL_DEPTH_TEST)

        glFlush()

    def drawWarningSign(self):
        """Draw a red warning triangle and warning text when roll exceeds ±70°"""
        glPushMatrix()
        glLoadIdentity()
        
        # Draw a red warning triangle
        glColor3f(1.0, 0.0, 0.0)  # Red color 🔴 
        glBegin(GL_TRIANGLES)
        glVertex2f(-0.2, 0.3)
        glVertex2f(0.2, 0.3)
        glVertex2f(0, 0.6)
        glEnd()
        
        # Display "ROLL WARNING!" text ao atingir 70  de angulo 🔴 🔴 🔴 🔴 🔴 
        painter = QPainter(self)
        painter.setPen(QColor(255, 0, 0))  # Red text na janela grafica 🔴 🔴 🔴 🔴 🔴 
        painter.setFont(QFont("Arial", 30, QFont.Bold))
        painter.drawText(300, 400, "ROLL WARNING!")  # Adjust position as needed
        painter.end()
        
        glPopMatrix()

    def drawRollMarkers(self):
        """Draw roll angle markers at ±10°, ±20°, ±30°, and ±45°"""
        glPushMatrix()
        glLoadIdentity()
        glColor3f(1.0, 1.0, 1.0)
        for angle in [-45, -30, -20, -10, 10, 20, 30, 45]:
            rad = np.radians(angle)
            x = np.sin(rad) * 0.8
            y = np.cos(rad) * 0.8
            glBegin(GL_LINES)
            glVertex2f(x, y)
            glVertex2f(x * 1.1, y * 1.1)
            glEnd()
        glPopMatrix()

    def drawHeadingScale(self):
        """Draw heading scale at the top"""
        yaw_angle = np.degrees(attitude_data["yaw"])
        glPushMatrix()
        glLoadIdentity()
        glTranslatef(0, 0.9, 0)
        glColor3f(1.0, 1.0, 1.0)

        glBegin(GL_LINES)
        glVertex2f(-0.5, 0)
        glVertex2f(0.5, 0)
        glEnd()

        for i in range(-90, 91, 30):
            x = i / 180.0
            glBegin(GL_LINES)
            glVertex2f(x, 0.02)
            glVertex2f(x, -0.02)
            glEnd()
        glPopMatrix()

    def drawAltitudeIndicator(self, voltage):
        """Draw altitude indicator (battery voltage as example)"""
        glPushMatrix()
        glLoadIdentity()
        glTranslatef(0.8, 0.9, 0)
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glVertex2f(0, 0)
        glVertex2f(0.1, 0)
        glVertex2f(0.1, 0.1)
        glVertex2f(0, 0.1)
        glEnd()
        glPopMatrix()

    def drawSpeedIndicator(self, speed):
        """Draw speed indicator"""
        glPushMatrix()
        glLoadIdentity()
        glTranslatef(0.8, 0.8, 0)
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glVertex2f(0, 0)
        glVertex2f(0.1, 0)
        glVertex2f(0.1, 0.1)
        glVertex2f(0, 0.1)
        glEnd()
        glPopMatrix()

    def drawWindIndicator(self, speed, direction):
        """Draw wind indicator (example)"""
        glPushMatrix()
        glLoadIdentity()
        glTranslatef(0.8, 0.7, 0)
        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_LINES)
        glVertex2f(0, 0)
        glVertex2f(np.cos(np.radians(direction)) * 0.1, np.sin(np.radians(direction)) * 0.1)
        glEnd()
        glPopMatrix()

    def drawFlightMode(self, flight_mode):
        """Display current flight mode using QPainter for text rendering"""
        glPushMatrix()
        glLoadIdentity()
        glTranslatef(-0.9, 0.9, 0)  # Position at top-left corner
        
        # Use QPainter for text rendering
        painter = QPainter(self)
        painter.setPen(QColor(255, 255, 255))  # White text color
        painter.setFont(QFont("Arial", 20))  # Set font size and family
        painter.drawText(10, 10, flight_mode)  # Draw text at position (10, 10)
        painter.end()
        
        glPopMatrix()

    def updateAttitude(self):
        self.update()

def move_servo(channel, pwm_value):
    """
    Envia um comando para mover o servo na porta PWM especificada.
    O pwm_value geralmente varia de 1000 (posição mínima) a 2000 (posição máxima).
    """
    if master:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # Confirmation
            channel,  # Canal PWM (no meu caso, 1)
            pwm_value,  # Valor PWM (ex: 1500 é posição central)
            0, 0, 0, 0, 0  # Parâmetros adicionais não usados no meu codigo no momento
        )

def drawServoPosition(roll_angle):
    glPushMatrix()
    glLoadIdentity()
    glTranslatef(-0.8, 0.8, 0)
    glColor3f(1.0, 0.0, 0.0)

    servo_pos = 0.5 + (roll_angle / 140.0)  # normalizado entre 0 e 1
    glBegin(GL_LINES)
    glVertex2f(0, 0)
    glVertex2f(0, servo_pos * 0.2)
    glEnd()
    glPopMatrix()

def update_attitude_data():
    global attitude_data
    msg = master.recv_match(type='ATTITUDE', blocking=False)
    if msg:
        attitude_data["roll"] = msg.roll
        attitude_data["pitch"] = msg.pitch
        attitude_data["yaw"] = msg.yaw

        roll_angle = np.degrees(attitude_data["roll"])
        pitch_angle = np.degrees(attitude_data["pitch"])
        yaw_angle = np.degrees(attitude_data["yaw"])

        print(f"MAVLINK - Roll: {roll_angle:.2f}°")

        # print(f"MAVLINK - pitch: {pitch_angle:.2f}°")

        # print(f"MAVLINK - yaw: {yaw_angle:.2f}°")

        # nao esta ainda funcionando o codigo abaixo para mover os servo 
        # Se o roll ultrapassar ±70°, mover o servo para um ângulo diferente 
        if roll_angle > 70:
            move_servo(1, 2000)  # Move o servo para a posição máxima
        elif roll_angle < -70:
            move_servo(1, 1000)  # Move o servo para a posição mínima
        else:
            move_servo(1, 1500)  # Retorna para a posição neutra

    QTimer.singleShot(50, update_attitude_data)




def set_flight_mode(mode):
    """Altera o modo de voo do drone."""
    if master is None:
        print("Erro: MAVLink não conectado.")
        return

    # Envia comando de mudança de modo
    master.set_mode(mode)
    print(f"Modo alterado para {mode}")


class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Attitude Indicator")
        self.setCentralWidget(AttitudeIndicator(self))


### here end mavlink

class HorizonDetector:

    def __init__(self, exclusion_thresh = 4, fov = 48.8 , acceptable_variance = 1.3, frame_shape = (100,100)):

        self.exclusion_thresh = exclusion_thresh # in degrees of pitch
        self.exclusion_thresh_pixels = exclusion_thresh * frame_shape[0] // fov
        self.fov = fov
        self.acceptable_variance = acceptable_variance
        self.predicted_roll = None
        self.predicted_pitch = None
        self.recent_horizons = [None, None]

    def find_horizon(self, frame:np.ndarray, diagnostic_mode:bool=True):
        """
        frame: the image in which you want to find the horizon
        diagnostic_mode: if True, draws a diagnostic visualization. Should only be used for
        testing, as it slows down performance.
        """
        # default values to return if no horizon can be found
        roll, pitch, variance, is_good_horizon = None, None, None, None

        # get greyscale
        bgr2gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # filter our blue from the sky
        lower = np.array([109, 0, 116]) 
        upper = np.array([153, 255, 255]) 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, lower, upper)
        blue_filtered_greyscale = cv2.add(bgr2gray, hsv_mask)

        # generate mask
        blur = cv2.bilateralFilter(blue_filtered_greyscale,9,50,50)
        _, mask = cv2.threshold(blur,250,255,cv2.THRESH_OTSU)
        edges = cv2.Canny(image=bgr2gray, threshold1=200, threshold2=250) 
        edges = skimage.measure.block_reduce(edges, (POOLING_KERNEL_SIZE , POOLING_KERNEL_SIZE), np.max)

        # find contours
        # chain = cv2.CHAIN_APPROX_SIMPLE
        chain = cv2.CHAIN_APPROX_NONE 
        # Compatibility across OpenCV versions
        opencv_major_version = int(cv2.__version__.split('.')[0])

        if opencv_major_version >= 4:
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, chain)
        else:
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, chain)


        # If there weren't any contours found (i.e. the image was all black),
        # end early, returning None values and the mask.
        if len(contours) == 0:
            # predict the next horizon
            self._predict_next_horizon()

            # convert the diagnostic image to color
            if diagnostic_mode:
                mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            return roll, pitch, variance, is_good_horizon, mask

        # find the contour with the largest area
        largest_contour = sorted(contours, key=cv2.contourArea, reverse=True)[0] 

        # extract x and y values from contour
        x_original = np.array([i[0][0] for i in largest_contour])
        y_original = np.array([i[0][1] for i in largest_contour])

        # Separate the points that lie on the edge of the frame from all other points.
        # Edge points will be used to find sky_is_up.
        # All other points will be used to find the horizon.
        x_abbr = []
        y_abbr = []
        x_edge_points = []
        y_edge_points = []
        for n, x_point in enumerate(x_original):
            y_point = y_original[n]
            if x_point == 0 or x_point == frame.shape[1] - 1 or \
                y_point == 0 or y_point == frame.shape[0]- 1:
                x_edge_points.append(x_point)
                y_edge_points.append(y_point)
            else:
                x_abbr.append(x_point)
                y_abbr.append(y_point)

        # Find the average position of the edge points.
        # This will help determine the direction of the sky.
        # Reduce the number of edge points to improve performance.
        maximum_number_of_points = 20
        step_size = len(x_edge_points)//maximum_number_of_points
        if step_size > 1:
            x_edge_points = x_edge_points[::step_size]
            y_edge_points = y_edge_points[::step_size]
        
        # Check if there are any edge points. If there are, take the average of them to determine
        # sky_is_up. If there aren't any edge points, take the average of the abbreviated point list
        # instead, since it is not possible to take the average of an empty list.
        if x_edge_points:
            avg_x = np.average(x_edge_points)
            avg_y = np.average(y_edge_points)
        else:
            avg_x = np.average(x_abbr)
            avg_y = np.average(y_abbr)

        # Reduce the number of horizon points to improve performance.
        maximum_number_of_points = 100
        step_size = len(x_original)//maximum_number_of_points
        if step_size > 1:
            x_abbr = x_abbr[::step_size]
            y_abbr = y_abbr[::step_size]  

        # define some values for checking distance to previous horizon
        if self.predicted_roll is not None:
            # convert predicted_roll to radians
            predicted_roll_radians = radians(self.predicted_roll)

            # find the distance 
            distance = self.predicted_pitch / self.fov * frame.shape[0]

            # define the line perpendicular to horizon
            angle_perp = predicted_roll_radians + pi / 2
            x_perp = distance * cos(angle_perp) + frame.shape[1]/2
            y_perp = distance * sin(angle_perp) + frame.shape[0]/2

            # convert from roll and pitch of predicted horizon to m and b
            run = cos(predicted_roll_radians)
            rise = sin(predicted_roll_radians) 
            if run != 0:
                predicted_m = rise / run
                predicted_b = y_perp - predicted_m * x_perp            

            # define two points on the line from the previous horizon
            p1 = np.array([0, predicted_b])
            p2 = np.array([frame.shape[1], predicted_m * frame.shape[1] + predicted_b])
            p2_minus_p1 = p2 - p1 

        # Initialize some lists to contain the new (filtered) x and y values.
        x_filtered = []
        y_filtered = []
        # Filter out points that don't lie on an edge.
        for idx, x_point in enumerate(x_abbr):
            y_point = y_abbr[idx]

            # evaluate if the point exists on an edge
            if edges[y_point//POOLING_KERNEL_SIZE][x_point//POOLING_KERNEL_SIZE] == 0:
                continue # do not append the point

            # If there is no predicted horizon, perform no further
            # filtering on this point and accept it as valid.
            if self.predicted_roll is None:
                x_filtered.append(x_point)
                y_filtered.append(y_point)
                continue 

            # If there is a predicted horizon, check if the current point
            # is reasonably close to it.
            p3 = np.array([x_point, y_point])
            distance = norm(np.cross(p2_minus_p1, p1-p3))/norm(p2_minus_p1)
            if distance < self.exclusion_thresh_pixels:
                x_filtered.append(x_point)
                y_filtered.append(y_point)

        # convert to numpy array
        x_filtered = np.array(x_filtered)
        y_filtered = np.array(y_filtered)

        # Draw the diagnostic information.
        # Only use for diagnostics, as this slows down inferences. 
        if diagnostic_mode:
            # scale up the diagnostic image to make it easier to see
            desired_height = 500
            scale_factor = desired_height / frame.shape[0]
            desired_width = int(np.round(frame.shape[1] * scale_factor))
            desired_dimensions = (desired_width, desired_height)
            mask = cv2.resize(mask, desired_dimensions)
            # convert the diagnostic image to color
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # draw the abbreviated points
            for n, i in enumerate(x_abbr):
                circle_x = int(np.round(i * scale_factor))
                circle_y = int(np.round(y_abbr[n] * scale_factor))
                cv2.circle(mask, (circle_x, circle_y), 5, (0,0,255), -1)
            # draw the filtered points
            for n, i in enumerate(x_filtered):
                circle_x = int(np.round(i * scale_factor))
                circle_y = int(np.round(y_filtered[n] * scale_factor))
                cv2.circle(mask, (circle_x, circle_y), 5, (0,255,0), -1)
            # draw the predicted horizon, if there is one
            if self.predicted_roll:
                lower_pitch = self.predicted_pitch + self.exclusion_thresh
                draw_horizon(mask, self.predicted_roll, lower_pitch, self.fov, (0,150,255),  False)
                upper_pitch = self.predicted_pitch - self.exclusion_thresh
                draw_horizon(mask, self.predicted_roll, upper_pitch, self.fov, (0,150,255),  False)
                cv2.putText(mask, 'Horizon Lock',(20,40),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,150,255),1,cv2.LINE_AA)

            # for testing
            _, edges_binary = cv2.threshold(edges,10,255,cv2.THRESH_BINARY)
            edges_binary = cv2.resize(edges_binary, desired_dimensions)
            cv2.imshow('canny', edges_binary)
            blue_filtered_greyscale = cv2.resize(blue_filtered_greyscale, desired_dimensions)
            cv2.imshow('blue_filtered_greyscale', blue_filtered_greyscale)
            mask = cv2.resize(mask, desired_dimensions)
            cv2.imshow('mask', mask)
                
        # Return None values for horizon, since too few points were found.
        if x_filtered.shape[0] < 12:
            self._predict_next_horizon()
            return roll, pitch, variance, is_good_horizon, mask

        # polyfit
        m, b = np.polyfit(x_filtered, y_filtered, 1)
        roll = atan2(m,1)
        roll = degrees(roll)

        # determine the direction of the sky (above or below)
        if m * avg_x + b > avg_y:
            sky_is_up = 1 # above
        else:
            sky_is_up = 0 # below

        # Get pitch
        # Take the distance from center point of the image to the horizon and find the pitch in degrees
        # based on field of view of the camera and the height of the image.
        # Define two points along horizon.
        p1 = np.array([0, b])
        p2 = np.array([frame.shape[1], m * frame.shape[1] + b])
        # Center of the image
        p3 = np.array([frame.shape[1]//2, frame.shape[0]//2])
        # Find distance to horizon
        distance_to_horizon = norm(np.cross(p2-p1, p1-p3))/norm(p2-p1)
        # Find out if plane is pointing above or below horizon
        if p3[1] < m * frame.shape[1]//2 + b and sky_is_up:
            plane_pointing_up = 1
        elif p3[1] > m *frame.shape[1]//2 + b and sky_is_up == False:
            plane_pointing_up = 1
        else:
            plane_pointing_up = 0
        pitch = distance_to_horizon / frame.shape[0] * self.fov
        if not plane_pointing_up:
            pitch *= -1

        # FIND VARIANCE 
        # This will be treated as a confidence score.
        p1 = np.array([0, b])
        p2 = np.array([frame.shape[1], m * frame.shape[1] + b])
        p2_minus_p1 = p2 - p1
        distance_list = []
        for n, x_point in enumerate(x_filtered):
            y_point = y_filtered[n]
            p3 = np.array([x_point, y_point])
            distance = norm(np.cross(p2_minus_p1, p1-p3))/norm(p2_minus_p1)
            distance_list.append(distance)
            
        variance = np.average(distance_list) / frame.shape[0] * 100
        
        # adjust the roll within the range of 0 - 360 degrees
        roll = self._adjust_roll(roll, sky_is_up) 

        # determine if the horizon is acceptable
        if variance < self.acceptable_variance: 
            is_good_horizon = 1
        else:
            is_good_horizon = 0

        # predict the approximate position of the next horizon
        self._predict_next_horizon(roll, pitch, is_good_horizon)

        # return the calculated values for horizon
        return roll, pitch, variance, is_good_horizon, mask
    
    def _adjust_roll(self, roll: float, sky_is_up: bool) -> float:
        """
        Adjusts the roll to be within the range of 0-360 degrees.
        Removes negative values and values greater than 360 degrees.
        """
        roll = abs(roll % FULL_ROTATION)
        in_sky_is_up_sector = (roll >= FULL_ROTATION * .75  or (roll > 0 and roll <= FULL_ROTATION * .25))
        
        if sky_is_up == in_sky_is_up_sector:
            return roll
        if roll < FULL_ROTATION / 2:
            roll += FULL_ROTATION / 2
        else:
            roll -= FULL_ROTATION / 2
        return roll

    def _predict_next_horizon(self, current_roll=None, current_pitch=None, is_good_horizon=None):
        """
        Based on the positions of recent horizons, predict the approximate
        position of the next horizon.
        Used to filter out noise in the next iteration.
        """
        # if the current horizon is not good, mark it as None
        if not is_good_horizon:
            current_horizon = None
        else:
            current_horizon = (current_roll, current_pitch)

        # update the list of recent horizons
        self.recent_horizons.append(current_horizon)
        del self.recent_horizons[0]
        
        # calculate the positions of the next horizon
        if None in self.recent_horizons:
            self.predicted_roll = None
            self.predicted_pitch = None
        else:
            roll1 = self.recent_horizons[0][0]
            roll2 = self.recent_horizons[1][0]
            roll_delta = roll2 - roll1
            self.predicted_roll =  roll2 + roll_delta

            pitch1 = self.recent_horizons[0][1]
            pitch2 = self.recent_horizons[1][1]
            pitch_delta = pitch2 - pitch1
            self.predicted_pitch = pitch2 + pitch_delta

def main():
    # Open the default webcam (index 0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' or 'Esc' to exit.")

    horizon_detector = HorizonDetector()


###mavlink and opengl
    app = QApplication(sys.argv)
    mainWin = MainApp()
    mainWin.show()
    update_attitude_data()  # Start updating attitude data from MAVLink
    update_battery_data()  # Start updating battery data from MAVLink  ///// ver se esta funcionando com minha nova bateria
    
##end mavlink and opengl 
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        
        #frame = cv2.imread("/Users/bernardolorenzini/Downloads/image.jpeg")

        scaled_and_cropped_frame = crop_and_scale(frame, **crop_and_scale_parameters)

        output = horizon_detector.find_horizon(scaled_and_cropped_frame, True)
        roll, pitch, variance, is_good_horizon, mask = output
        
        color = (255,0,0)       
        draw_horizon(frame, roll, pitch, horizon_detector.fov, color, True)

        if roll is not None:
            normalized_roll = normalize_roll(roll)
            # continue processing

        corrected_pitch = pitch


        print(f'OPEN CV - Calculated roll: {normalized_roll:.2f}°')
        # print(f'OPEN CV - Calculated pitch: {pitch}°')
        #final_image = horizon_detector.find_horizon(frame)
        #cv2.imshow("Webcam Feed", final_image)
        # draw center circle
        center = (frame.shape[1]//2, frame.shape[0]//2)
        radius = frame.shape[0]//100
        cv2.circle(frame, center, radius, (255,0,0), 2)

        # show results
        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)


        

        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):  # 27 is the Esc key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    
    main()
