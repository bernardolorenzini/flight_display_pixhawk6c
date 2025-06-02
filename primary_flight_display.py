import sys
import glob
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QTimer, QPoint
from PyQt5.QtGui import QPainter, QColor, QFont, QTransform
from pymavlink import mavutil
from math import degrees

class PFD(QWidget):
    def __init__(self, master):
        super().__init__()
        self.master = master

        self.setWindowTitle("PFD - Artificial Horizon, Speed and Altitude Tape")
        self.setGeometry(100, 100, 800, 600)

        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed = None
        self.heading = None
        self.roll = 0
        self.pitch = 0

        # Timer to update PFD
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_pfd)
        self.timer.start(100)  # Update every 100 ms for smoothness

    def update_pfd(self):
        # GPS message
        msg_gps = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg_gps and msg_gps.fix_type >= 3:
            self.latitude = msg_gps.lat / 1e7
            self.longitude = msg_gps.lon / 1e7
            self.altitude = msg_gps.alt / 1000  # meters
            self.speed = msg_gps.vel / 100  # m/s (vel is in cm/s)
            self.heading = msg_gps.cog / 100  # degrees

        # Attitude message
        msg_att = self.master.recv_match(type='ATTITUDE', blocking=False)
        if msg_att:
            self.roll = degrees(msg_att.roll)    # Roll in degrees
            self.pitch = degrees(msg_att.pitch)  # Pitch in degrees

        self.update()  # Trigger repaint

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        center_x = self.width() // 2
        center_y = self.height() // 2

        # Background
        painter.fillRect(self.rect(), QColor(10, 10, 30))

                # Draw artificial horizon
        self.draw_horizon(painter, center_x, center_y)

        # Draw heading tape at top
        self.draw_heading_tape(painter, center_x, 150)

        # Heading and Lat/Lon display
        font = QFont('Arial', 18, QFont.Bold)
        painter.setFont(font)
        painter.setPen(QColor(255, 255, 255))

        if self.latitude is not None and self.longitude is not None:
            painter.drawText(center_x - 50, 50, f"HDG {self.heading:.1f}°")
            painter.drawText(20, 50, f"LAT {self.latitude:.5f}")
            painter.drawText(20, 90, f"LON {self.longitude:.5f}")

            self.draw_speed_tape(painter, center_x - 250, center_y)
            self.draw_altitude_tape(painter, center_x + 250, center_y)


    def draw_horizon(self, painter, center_x, center_y):
        painter.save()

        # Move origin to center
        painter.translate(center_x, center_y)

        # Rotate according to Roll (fixed airplane, moving world)
        painter.rotate(self.roll)

        # Move up/down according to Pitch
        pitch_offset = self.pitch * 4  # scale pitch for display
        painter.translate(0, pitch_offset)

        # Sky
        painter.setBrush(QColor(70, 130, 180))  # Sky blue
        painter.drawRect(-1000, -1000, 2000, 1000)

        # Ground
        painter.setBrush(QColor(139, 69, 19))  # Brown
        painter.drawRect(-1000, 0, 2000, 1000)

        # Horizon Line
        painter.setPen(QColor(255, 255, 255))
        painter.drawLine(-400, 0, 400, 0)

        # Pitch Ladder
        font = QFont('Arial', 10)
        painter.setFont(font)
        ladder_spacing = 40  # pixels between 10° pitch lines

        painter.setPen(QColor(255, 255, 255))

        for pitch in range(10, 91, 10):  # +10 to +90
            offset = pitch * 4
            # Positive Pitch
            painter.drawLine(-40, -offset, 40, -offset)
            painter.drawText(-60, -offset + 5, f"{pitch}")
            painter.drawText(45, -offset + 5, f"{pitch}")

            # Negative Pitch
            painter.drawLine(-40, offset, 40, offset)
            painter.drawText(-60, offset + 5, f"-{pitch}")
            painter.drawText(45, offset + 5, f"-{pitch}")

        painter.restore()

        # Draw fixed airplane symbol
        painter.setPen(QColor(255, 255, 0))
        painter.drawLine(center_x - 20, center_y, center_x + 20, center_y)  # Wings
        painter.drawLine(center_x, center_y - 10, center_x, center_y + 10)  # Fuselage


    def draw_speed_tape(self, painter, x, center_y):
        painter.setPen(QColor(0, 255, 0))
        painter.drawRect(x - 40, center_y - 100, 80, 200)

        if self.speed is not None:
            for i in range(-2, 3):
                speed_value = self.speed + (i * 5)
                y = center_y + (i * 40)
                painter.drawText(x - 30, y + 10, f"{int(speed_value)}")

            painter.setBrush(QColor(0, 255, 0))
            painter.drawRect(x - 40, center_y - 20, 80, 40)
            painter.setPen(QColor(0, 0, 0))
            painter.drawText(x - 20, center_y + 10, f"{int(self.speed)}")

    def draw_altitude_tape(self, painter, x, center_y):
        painter.setPen(QColor(0, 255, 0))
        painter.drawRect(x - 40, center_y - 100, 80, 200)

        if self.altitude is not None:
            for i in range(-2, 3):
                alt_value = self.altitude + (i * 10)
                y = center_y + (i * 40)
                painter.drawText(x - 30, y + 10, f"{int(alt_value)}")

            painter.setBrush(QColor(0, 255, 0))
            painter.drawRect(x - 40, center_y - 20, 80, 40)
            painter.setPen(QColor(0, 0, 0))
            painter.drawText(x - 20, center_y + 10, f"{int(self.altitude)}")

    def draw_heading_tape(self, painter, center_x, top_y):
        if self.heading is None:
            return

        painter.save()

        # Center the tape at the center_x
        tape_width = 400
        heading_spacing = 4  # pixels per degree
        pixels_per_deg = heading_spacing

        # Current heading in degrees
        current_heading = self.heading % 360

        painter.translate(center_x, top_y)

        # Draw base line
        painter.setPen(QColor(255, 255, 255))
        painter.drawLine(-tape_width // 2, 0, tape_width // 2, 0)

        font = QFont('Arial', 10, QFont.Bold)
        painter.setFont(font)

        # Draw heading ticks
        for deg in range(-60, 61, 10):  # show from -60° to +60°
            heading_deg = (current_heading + deg) % 360
            x = deg * pixels_per_deg

            if deg % 30 == 0:
                # Major tick with number or cardinal point
                painter.drawLine(x, -10, x, 10)

                if heading_deg in [0, 360]:
                    label = "N"
                elif heading_deg == 90:
                    label = "E"
                elif heading_deg == 180:
                    label = "S"
                elif heading_deg == 270:
                    label = "W"
                else:
                    label = f"{int(heading_deg):03d}"  # e.g., 030, 120

                painter.drawText(x - 10, -15, label)
            else:
                # Minor tick
                painter.drawLine(x, -5, x, 5)

        # Draw current heading pointer (fixed yellow triangle)
        painter.setBrush(QColor(255, 255, 0))
        painter.setPen(QColor(255, 255, 0))
        pointer = [
            (0, 15), (-10, 0), (10, 0)
        ]
        painter.drawPolygon(*[QPoint(p[0], p[1]) for p in pointer])

        painter.restore()



def main():
    import time

    # Dynamic USB port search
    telemetry_ports = glob.glob("/dev/cu.usbmodem*")

    if not telemetry_ports:
        print("Erro: Nenhuma porta /dev/cu.usbmodem encontrada.")
        sys.exit(1)

    master = None
    for port in telemetry_ports:
        try:
            print(f"Tentando conectar na porta {port}...")
            master = mavutil.mavlink_connection(port, baud=57600)
            print("Aguardando heartbeat...")
            master.wait_heartbeat()
            print(f"Conectado! System: {master.target_system}, Component: {master.target_component}")
            break
        except Exception as e:
            print(f"Falha ao conectar em {port}: {e}")

    if master is None:
        print("Erro: Não foi possível conectar a nenhuma porta.")
        sys.exit(1)

    print(f"Connected to system (ID {master.target_system})")

    app = QApplication(sys.argv)
    pfd = PFD(master)
    pfd.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
