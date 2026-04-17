"""
theseus_viz.py  —  Theseus Telemetry 3D Visualizer
====================================================
Reads THS, packets from the ground LoRa serial port and renders:
  - 3D rocket model rotating in real time based on gyro/accel fusion
  - Altitude bar
  - Bay temperature
  - Acceleration Z
  - Gyro rates
  - Link status

Usage:
    python theseus_viz.py --port COM8 --baud 9600

Packet format expected:
    THS,<alt_m>,<press_pa>,<temp_c>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\r\n
"""

import argparse
import math
import sys
import threading
import time
from collections import deque

import serial
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *


# ---------------------------------------------------------------------------
# Shared telemetry state
# ---------------------------------------------------------------------------
class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.alt_m      = 0.0
        self.press_pa   = 101325.0
        self.temp_c     = 0.0
        self.ax         = 0.0
        self.ay         = 0.0
        self.az         = 1.0
        self.gx         = 0.0
        self.gy         = 0.0
        self.gz         = 0.0
        self.roll       = 0.0
        self.pitch      = 0.0
        self.yaw        = 0.0
        self.last_packet_time = 0.0
        self.packet_count     = 0
        self.alt_history      = deque(maxlen=200)

    def update(self, fields):
        with self.lock:
            self.alt_m    = fields[0]
            self.press_pa = fields[1]
            self.temp_c   = fields[2]
            self.ax       = fields[3]
            self.ay       = fields[4]
            self.az       = fields[5]
            self.gx       = fields[6]
            self.gy       = fields[7]
            self.gz       = fields[8]
            self.last_packet_time = time.time()
            self.packet_count += 1
            self.alt_history.append(self.alt_m)

    def get(self):
        with self.lock:
            return {
                'alt_m':    self.alt_m,
                'press_pa': self.press_pa,
                'temp_c':   self.temp_c,
                'ax': self.ax, 'ay': self.ay, 'az': self.az,
                'gx': self.gx, 'gy': self.gy, 'gz': self.gz,
                'roll':  self.roll,
                'pitch': self.pitch,
                'yaw':   self.yaw,
                'last_packet_time': self.last_packet_time,
                'packet_count':     self.packet_count,
                'alt_history':      list(self.alt_history),
            }


# ---------------------------------------------------------------------------
# Complementary filter for attitude estimation
# ---------------------------------------------------------------------------
class AttitudeFilter:
    ALPHA = 0.98
    DT    = 0.1   # 10 Hz

    def __init__(self, state: TelemetryState):
        self.state = state
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

    def update(self, ax, ay, az, gx, gy, gz):
        # accel-based angles
        accel_roll  =  math.degrees(math.atan2(ay, az))
        accel_pitch = -math.degrees(math.atan2(ax, math.sqrt(ay*ay + az*az)))

        # gyro integration
        self.roll  = self.ALPHA * (self.roll  + gx * self.DT) + (1 - self.ALPHA) * accel_roll
        self.pitch = self.ALPHA * (self.pitch + gy * self.DT) + (1 - self.ALPHA) * accel_pitch
        self.yaw  += gz * self.DT

        with self.state.lock:
            self.state.roll  = self.roll
            self.state.pitch = self.pitch
            self.state.yaw   = self.yaw


# ---------------------------------------------------------------------------
# Serial receiver thread
# ---------------------------------------------------------------------------
def serial_thread(port: str, baud: int, state: TelemetryState, attitude: AttitudeFilter):
    print(f"[SERIAL] Opening {port} @ {baud}")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        return

    print(f"[SERIAL] Listening for THS packets...")
    buffer = ""
    while True:
        try:
            waiting = ser.in_waiting
            if waiting > 0:
                raw = ser.read(waiting)
                buffer += raw.decode('utf-8', errors='replace')

                while 'THS,' in buffer:
                    start = buffer.find('THS,')
                    rest  = buffer[start + 4:]
                    next_ths = rest.find('THS,')
                    if next_ths == -1:
                        # only one packet, need more data
                        if '\n' in rest or '\r' in rest:
                            for delim in ['\r\n', '\n', '\r']:
                                end = rest.find(delim)
                                if end != -1:
                                    chunk = rest[:end].strip()
                                    buffer = rest[end:]
                                    break
                            else:
                                break
                        else:
                            break
                    else:
                        chunk = rest[:next_ths].strip().rstrip(',').rstrip('\r\n')
                        buffer = rest[next_ths:]

                    parts = chunk.split(',')
                    if len(parts) == 9:
                        try:
                            fields = [float(p) for p in parts]
                            state.update(fields)
                            attitude.update(fields[3], fields[4], fields[5],
                                            fields[6], fields[7], fields[8])
                            print(f"[OK] alt={fields[0]:.1f} az={fields[5]:.2f} gx={fields[6]:.1f} gy={fields[7]:.1f} gz={fields[8]:.1f}")
                        except ValueError:
                            pass
            else:
                time.sleep(0.01)
        except Exception as e:
            continue
# ---------------------------------------------------------------------------
# OpenGL rocket geometry
# ---------------------------------------------------------------------------
def draw_rocket():
    """Simple rocket: cylinder body + nose cone + fins"""

    # body
    glColor3f(0.85, 0.85, 0.90)
    quad = gluNewQuadric()
    glPushMatrix()
    glRotatef(-90, 1, 0, 0)
    gluCylinder(quad, 0.15, 0.15, 1.2, 16, 4)
    glPopMatrix()

    # nose cone
    glColor3f(0.95, 0.30, 0.20)
    glPushMatrix()
    glTranslatef(0, 1.2, 0)
    glRotatef(-90, 1, 0, 0)
    gluCylinder(quad, 0.15, 0.0, 0.5, 16, 4)
    glPopMatrix()

    # engine nozzle
    glColor3f(0.4, 0.4, 0.45)
    glPushMatrix()
    glRotatef(90, 1, 0, 0)
    gluCylinder(quad, 0.10, 0.15, 0.2, 16, 4)
    glPopMatrix()

    # fins (4x)
    glColor3f(0.95, 0.30, 0.20)
    for angle in [0, 90, 180, 270]:
        glPushMatrix()
        glRotatef(angle, 0, 1, 0)
        glBegin(GL_TRIANGLES)
        glVertex3f(0.15,  0.0,  0.0)
        glVertex3f(0.15,  0.35, 0.0)
        glVertex3f(0.45, -0.15, 0.0)
        glEnd()
        glPopMatrix()

    gluDeleteQuadric(quad)


def draw_ground_grid():
    glColor3f(0.25, 0.35, 0.25)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    for i in range(-5, 6):
        glVertex3f(i * 0.5, -2.0, -2.5)
        glVertex3f(i * 0.5, -2.0,  2.5)
        glVertex3f(-2.5, -2.0, i * 0.5)
        glVertex3f( 2.5, -2.0, i * 0.5)
    glEnd()


def draw_axes():
    glLineWidth(2.0)
    glBegin(GL_LINES)
    glColor3f(1, 0, 0); glVertex3f(0,0,0); glVertex3f(0.5,0,0)
    glColor3f(0, 1, 0); glVertex3f(0,0,0); glVertex3f(0,0.5,0)
    glColor3f(0, 0, 1); glVertex3f(0,0,0); glVertex3f(0,0,0.5)
    glEnd()


# ---------------------------------------------------------------------------
# HUD rendering (pygame 2D overlay)
# ---------------------------------------------------------------------------
def render_hud(surface, font_large, font_small, font_mono, data, W, H):
    link_age = time.time() - data['last_packet_time']
    link_ok  = link_age < 1.0

    # background panel
    panel = pygame.Surface((280, H), pygame.SRCALPHA)
    panel.fill((10, 10, 20, 180))
    surface.blit(panel, (0, 0))

    y = 12
    pad = 10

    def label(txt, color=(180, 180, 200)):
        s = font_small.render(txt, True, color)
        surface.blit(s, (pad, y))

    def value(txt, color=(255, 255, 255)):
        s = font_large.render(txt, True, color)
        surface.blit(s, (pad, y))

    def mono(txt, color=(200, 220, 200)):
        s = font_mono.render(txt, True, color)
        surface.blit(s, (pad, y))

    # link status
    link_color = (0, 255, 100) if link_ok else (255, 60, 60)
    link_txt   = f"LINK  {'OK' if link_ok else 'LOST'}  #{data['packet_count']}"
    s = font_small.render(link_txt, True, link_color)
    surface.blit(s, (pad, y)); y += 22

    pygame.draw.line(surface, (60, 60, 80), (pad, y), (270, y)); y += 8

    # altitude
    label("altitude"); y += 16
    alt_color = (100, 220, 255)
    value(f"{data['alt_m']:.1f} m", alt_color); y += 32

    # altitude bar
    bar_h = 80
    bar_w = 12
    max_alt = max(200.0, data['alt_m'] * 1.2)
    fill = int((data['alt_m'] / max_alt) * bar_h)
    fill = max(0, min(fill, bar_h))
    pygame.draw.rect(surface, (40, 40, 60), (250, y - 32, bar_w, bar_h))
    pygame.draw.rect(surface, alt_color,    (250, y - 32 + bar_h - fill, bar_w, fill))
    y += 52

    pygame.draw.line(surface, (60, 60, 80), (pad, y), (270, y)); y += 8

    # temperature
    label("bay temperature"); y += 16
    temp_c = data['temp_c']
    if temp_c < 40:
        tc = (100, 220, 150)
    elif temp_c < 60:
        tc = (255, 200, 50)
    else:
        tc = (255, 80, 80)
    value(f"{temp_c:.1f} C", tc); y += 32

    pygame.draw.line(surface, (60, 60, 80), (pad, y), (270, y)); y += 8

    # pressure
    label("pressure"); y += 16
    value(f"{data['press_pa']:.0f} Pa", (200, 200, 255)); y += 32

    pygame.draw.line(surface, (60, 60, 80), (pad, y), (270, y)); y += 8

    # acceleration
    label("acceleration (g)"); y += 16
    mono(f"X {data['ax']:+.2f}  Y {data['ay']:+.2f}"); y += 18
    az_color = (255, 150, 50) if abs(data['az']) > 2.0 else (200, 220, 200)
    mono(f"Z {data['az']:+.2f}", az_color); y += 24

    pygame.draw.line(surface, (60, 60, 80), (pad, y), (270, y)); y += 8

    # gyro
    label("gyro (deg/s)"); y += 16
    mono(f"X {data['gx']:+6.1f}"); y += 18
    mono(f"Y {data['gy']:+6.1f}"); y += 18
    mono(f"Z {data['gz']:+6.1f}"); y += 24

    pygame.draw.line(surface, (60, 60, 80), (pad, y), (270, y)); y += 8

    # attitude
    label("attitude (deg)"); y += 16
    mono(f"roll  {data['roll']:+7.1f}"); y += 18
    mono(f"pitch {data['pitch']:+7.1f}"); y += 18
    mono(f"yaw   {data['yaw']:+7.1f}"); y += 24

    # altitude history sparkline
    pygame.draw.line(surface, (60, 60, 80), (pad, y), (270, y)); y += 8
    label("altitude history"); y += 16
    hist = data['alt_history']
    if len(hist) > 2:
        mx = max(hist) if max(hist) > 0 else 1
        mn = min(hist)
        rng = mx - mn if mx != mn else 1
        spark_w = 260
        spark_h = 40
        pts = []
        for i, v in enumerate(hist):
            sx = pad + int(i / len(hist) * spark_w)
            sy = y + spark_h - int((v - mn) / rng * spark_h)
            pts.append((sx, sy))
        if len(pts) > 1:
            pygame.draw.lines(surface, alt_color, False, pts, 1)
        y += spark_h + 8


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Theseus 3D telemetry visualizer")
    parser.add_argument('--port', default='COM8',  help='LoRa ground serial port')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate')
    parser.add_argument('--demo', action='store_true', help='Run with simulated data, no serial')
    args = parser.parse_args()

    W, H = 1200, 700

    state    = TelemetryState()
    attitude = AttitudeFilter(state)

    if not args.demo:
        t = threading.Thread(target=serial_thread,
                             args=(args.port, args.baud, state, attitude),
                             daemon=True)
        t.start()
    else:
        def demo_thread():
            t = 0.0
            while True:
                t += 0.1
                fields = [
                    30 + 10 * math.sin(t * 0.3),
                    101000 + 500 * math.sin(t * 0.1),
                    27 + math.sin(t * 0.05),
                    0.05 * math.sin(t),
                    0.05 * math.cos(t),
                    1.0 + 0.1 * math.sin(t * 2),
                    5 * math.sin(t * 0.5),
                    3 * math.cos(t * 0.3),
                    1 * math.sin(t * 0.2),
                ]
                state.update(fields)
                attitude.update(*fields[3:9])
                time.sleep(0.1)
        threading.Thread(target=demo_thread, daemon=True).start()

    pygame.init()
    screen = pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Theseus Telemetry — 3D Visualizer")

    font_large = pygame.font.SysFont('consolas', 22, bold=True)
    font_small = pygame.font.SysFont('consolas', 13)
    font_mono  = pygame.font.SysFont('consolas', 14)

    hud_surface = pygame.Surface((W, H), pygame.SRCALPHA)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    glLightfv(GL_LIGHT0, GL_POSITION, [2, 4, 3, 1])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1, 1, 1, 1])
    glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.3, 0.3, 0.3, 1])

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, W / H, 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)

    clock = pygame.time.Clock()
    cam_yaw   = 30.0
    cam_pitch = 15.0
    dragging  = False
    last_mouse = (0, 0)

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                pygame.quit()
                sys.exit()
            if event.type == MOUSEBUTTONDOWN and event.button == 1:
                dragging = True
                last_mouse = event.pos
            if event.type == MOUSEBUTTONUP and event.button == 1:
                dragging = False
            if event.type == MOUSEMOTION and dragging:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                cam_yaw   += dx * 0.4
                cam_pitch += dy * 0.4
                cam_pitch  = max(-89, min(89, cam_pitch))
                last_mouse = event.pos

        data = state.get()

        # 3D scene
        glClearColor(0.05, 0.07, 0.12, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glLoadIdentity()
        gluLookAt(
            4 * math.cos(math.radians(cam_yaw)) * math.cos(math.radians(cam_pitch)),
            4 * math.sin(math.radians(cam_pitch)),
            4 * math.sin(math.radians(cam_yaw)) * math.cos(math.radians(cam_pitch)),
            0, 0.3, 0,
            0, 1, 0
        )

        glDisable(GL_LIGHTING)
        draw_ground_grid()
        draw_axes()
        glEnable(GL_LIGHTING)

        glPushMatrix()
        glRotatef(data['roll'],  0, 0, 1)
        glRotatef(data['pitch'], 1, 0, 0)
        glRotatef(data['yaw'],   0, 1, 0)
        draw_rocket()
        glPopMatrix()

        # HUD overlay
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, W, H, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        hud_surface.fill((0, 0, 0, 0))
        render_hud(hud_surface, font_large, font_small, font_mono, data, W, H)

        hud_data = pygame.image.tostring(hud_surface, 'RGBA', True)
        glRasterPos2i(0, 0)
        glDrawPixels(W, H, GL_RGBA, GL_UNSIGNED_BYTE, hud_data)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        pygame.display.flip()
        clock.tick(30)


if __name__ == '__main__':
    main()
