import sys
import numpy as np
import cv2
from math import sin, cos, radians, pi

from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPainter, QColor, QFont
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from pymavlink import mavutil

# Lista de possíveis portas de telemetria
telemetry_ports = ["/dev/tty.usbmodem146401", "/dev/tty.usbmodem146403"]


# Constants
FULL_ROTATION_RADIANS = 2 * pi

def _find_points(m, b, shape):
    """Find two points where the line intersects the image edges"""
    height, width = shape[:2]
    points = []

    # Left border (x = 0)
    y = int(m * 0 + b)
    if 0 <= y < height:
        points.append((0, y))

    # Right border (x = width - 1)
    y = int(m * (width - 1) + b)
    if 0 <= y < height:
        points.append((width - 1, y))

    # Top border (y = 0)
    if m != 0:
        x = int((0 - b) / m)
        if 0 <= x < width:
            points.append((x, 0))

    # Bottom border (y = height - 1)
    if m != 0:
        x = int((height - 1 - b) / m)
        if 0 <= x < width:
            points.append((x, height - 1))

    if len(points) >= 2:
        return points[:2]
    else:
        return None

def draw_horizon(frame: np.ndarray, roll: float, pitch: float, 
                 fov: float, color: tuple, draw_groundline: bool):
    if roll is None:
        return frame

    roll = radians(roll)
    sky_is_up = (roll >= FULL_ROTATION_RADIANS * .75 or (0 < roll <= FULL_ROTATION_RADIANS * .25))
    distance = pitch / fov * frame.shape[0]

    angle_perp = roll + pi / 2
    x_perp = distance * cos(angle_perp) + frame.shape[1] / 2
    y_perp = distance * sin(angle_perp) + frame.shape[0] / 2

    run = cos(roll)
    rise = sin(roll)
    if run != 0:
        m = rise / run
        b = y_perp - m * x_perp
        points = _find_points(m, b, frame.shape)
        if not points:
            return frame
        else:
            p1, p2 = points
    else:
        p1 = (int(np.round(x_perp)), 0)
        p2 = (int(np.round(x_perp)), frame.shape[0])

    cv2.line(frame, p1, p2, color, 2)

    if draw_groundline and run != 0:
        m_perp = -1 / m
        b_perp = y_perp - m_perp * x_perp
        points = _find_points(m_perp, b_perp, frame.shape)
        if not points:
            return frame
        above_line = m * points[0][0] + b < points[0][1]
        if (sky_is_up and above_line) or (not sky_is_up and not above_line):
            p2 = points[0]
        else:
            p2 = points[1]
        p1 = (int(np.round(x_perp)), int(np.round(y_perp)))
        cv2.line(frame, p1, p2, (0, 191, 255), 1)

    return frame

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


def update_attitude_data():
    global attitude_data
    msg = master.recv_match(type='ATTITUDE', blocking=False)
    if msg:
        attitude_data["roll"] = msg.roll
        attitude_data["pitch"] = msg.pitch
        attitude_data["yaw"] = msg.yaw

        roll_angle = np.degrees(attitude_data["roll"])

        print(f"Roll: {roll_angle:.2f}°")


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

def main():

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam")
        return

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Warning: Frame not captured properly")
            continue

        roll = -np.degrees(attitude_data["roll"])      # degrees
        pitch = -np.degrees(attitude_data["pitch"]) / 90     # degrees
        fov = 90      # degrees (adjust as needed)
        color = (0, 255, 0)

        frame = draw_horizon(frame, roll, pitch, fov, color, draw_groundline=True)

        cv2.imshow("Drone Horizon View", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = MainApp()
    mainWin.show()
    update_attitude_data()  # Start updating attitude data from MAVLink
    update_battery_data()  # Start updating battery data from MAVLink  ///// ver se esta funcionando com minha nova bateria
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Warning: Frame not captured properly")
            continue

        roll = -np.degrees(attitude_data["roll"])      # degrees
        pitch = -np.degrees(attitude_data["pitch"]) / 90     # degrees
        fov = 90      # degrees (adjust as needed)
        color = (0, 255, 0)

        frame = draw_horizon(frame, roll, pitch, fov, color, draw_groundline=True)

        cv2.imshow("Drone Horizon View", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    sys.exit(app.exec_())
