"""
theseus_viz.py  —  Theseus Telemetry 3D Visualizer v2
Usage:
    python theseus_viz.py --port COM8 --baud 9600
    python theseus_viz.py --demo
"""

import argparse, math, sys, threading, time
from collections import deque
import serial, pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

class TelemetryState:
    def __init__(self):
        self.lock=threading.Lock(); self.alt_m=0.0; self.press_pa=101325.0
        self.temp_c=25.0; self.ax=self.ay=0.0; self.az=1.0
        self.gx=self.gy=self.gz=0.0; self.roll=self.pitch=self.yaw=0.0
        self.last_packet=0.0; self.packet_count=0
        self.alt_history=deque(maxlen=300); self.az_history=deque(maxlen=300)
        self.temp_history=deque(maxlen=300)
    def update(self,f):
        with self.lock:
            self.alt_m=f[0];self.press_pa=f[1];self.temp_c=f[2]
            self.ax=f[3];self.ay=f[4];self.az=f[5]
            self.gx=f[6];self.gy=f[7];self.gz=f[8]
            self.last_packet=time.time();self.packet_count+=1
            self.alt_history.append(f[0]);self.az_history.append(f[5]);self.temp_history.append(f[2])
    def get(self):
        with self.lock:
            return dict(alt_m=self.alt_m,press_pa=self.press_pa,temp_c=self.temp_c,
                ax=self.ax,ay=self.ay,az=self.az,gx=self.gx,gy=self.gy,gz=self.gz,
                roll=self.roll,pitch=self.pitch,yaw=self.yaw,
                last_packet=self.last_packet,packet_count=self.packet_count,
                alt_history=list(self.alt_history),az_history=list(self.az_history),
                temp_history=list(self.temp_history))

class AttitudeFilter:
    ALPHA=0.90
    def __init__(self,state):
        self.state=state;self.roll=self.pitch=self.yaw=0.0
        self.last_time=None;self.dr=self.dp=self.dy=0.0
    def update(self,ax,ay,az,gx,gy,gz):
        now=time.time()
        if self.last_time is None:self.last_time=now;return
        dt=min(now-self.last_time,0.5);self.last_time=now
        mag=math.sqrt(ax*ax+ay*ay+az*az)
        if mag>0.1:
            ax/=mag;ay/=mag;az/=mag
            ar=math.degrees(math.atan2(ay,az))
            ap=-math.degrees(math.atan2(ax,math.sqrt(ay*ay+az*az)))
        else:ar,ap=self.roll,self.pitch
        self.roll=self.ALPHA*(self.roll+gx*dt)+(1-self.ALPHA)*ar
        self.pitch=self.ALPHA*(self.pitch+gy*dt)+(1-self.ALPHA)*ap
        self.yaw+=gz*dt
        with self.state.lock:
            self.state.roll=self.roll;self.state.pitch=self.pitch;self.state.yaw=self.yaw
    def smooth(self,a=0.18):
        with self.state.lock:r,p,y=self.state.roll,self.state.pitch,self.state.yaw
        self.dr+=a*(r-self.dr);self.dp+=a*(p-self.dp);self.dy+=a*(y-self.dy)
        return self.dr,self.dp,self.dy

def serial_thread(port,baud,state,attitude):
    print(f"[SERIAL] Opening {port} @ {baud}")
    try:ser=serial.Serial(port,baud,timeout=0.1)
    except serial.SerialException as e:print(f"[ERROR] {e}");return
    print("[SERIAL] Listening for THS packets...")
    buf=""
    while True:
        try:
            w=ser.in_waiting
            if w>0:
                buf+=ser.read(w).decode('utf-8',errors='replace')
                while 'THS,' in buf:
                    s=buf.find('THS,');rest=buf[s+4:];nxt=rest.find('THS,')
                    if nxt==-1:
                        end=-1
                        for d in ['\r\n','\n','\r']:
                            i=rest.find(d)
                            if i!=-1 and(end==-1 or i<end):end=i
                        if end==-1:break
                        chunk=rest[:end].strip();buf=rest[end:]
                    else:
                        chunk=rest[:nxt].strip().rstrip(',\r\n');buf=rest[nxt:]
                    parts=chunk.split(',')
                    if len(parts)==9:
                        try:
                            f=[float(x) for x in parts]
                            state.update(f);attitude.update(f[3],f[4],f[5],f[6],f[7],f[8])
                        except ValueError:pass
            else:time.sleep(0.005)
        except Exception:continue

def draw_rocket():
    q=gluNewQuadric()
    glColor3f(0.85,0.85,0.92)
    glPushMatrix();glRotatef(-90,1,0,0);gluCylinder(q,0.15,0.15,1.2,20,4);glPopMatrix()
    glColor3f(0.95,0.28,0.18)
    glPushMatrix();glTranslatef(0,1.2,0);glRotatef(-90,1,0,0);gluCylinder(q,0.15,0,0.55,20,4);glPopMatrix()
    glColor3f(0.35,0.35,0.40)
    glPushMatrix();glRotatef(90,1,0,0);gluCylinder(q,0.08,0.15,0.22,20,4);glPopMatrix()
    glColor3f(0.95,0.28,0.18)
    for a in [0,90,180,270]:
        glPushMatrix();glRotatef(a,0,1,0);glBegin(GL_TRIANGLES)
        glVertex3f(0.15,0,0);glVertex3f(0.15,0.38,0);glVertex3f(0.50,-0.18,0)
        glEnd();glPopMatrix()
    gluDeleteQuadric(q)

def draw_grid():
    glColor3f(0.18,0.28,0.18);glLineWidth(1);glBegin(GL_LINES)
    for i in range(-6,7):
        glVertex3f(i*.5,-2,-3);glVertex3f(i*.5,-2,3)
        glVertex3f(-3,-2,i*.5);glVertex3f(3,-2,i*.5)
    glEnd()

def draw_axes():
    glLineWidth(2);glBegin(GL_LINES)
    glColor3f(1,.2,.2);glVertex3f(0,0,0);glVertex3f(.6,0,0)
    glColor3f(.2,1,.2);glVertex3f(0,0,0);glVertex3f(0,.6,0)
    glColor3f(.2,.2,1);glVertex3f(0,0,0);glVertex3f(0,0,.6)
    glEnd()

PW=300

def spark(surf,hist,col,x,y,w,h):
    if len(hist)<2:return
    mn,mx=min(hist),max(hist);rng=mx-mn or 1.0
    pts=[(x+int(i/len(hist)*w),y+h-int((v-mn)/rng*h)) for i,v in enumerate(hist)]
    pygame.draw.lines(surf,col,False,pts,1);pygame.draw.circle(surf,col,pts[-1],3)

def render_hud(surf,fonts,data,W,H):
    fl,fs,fm=fonts['L'],fonts['S'],fonts['M']
    panel=pygame.Surface((PW,H),pygame.SRCALPHA);panel.fill((8,10,18,210));surf.blit(panel,(0,0))
    y=10;px=10;pw=PW-20
    link_age=time.time()-data['last_packet'];ok=link_age<1.5
    def sep():
        nonlocal y
        pygame.draw.line(surf,(40,48,68),(px,y),(px+pw,y));y+=6
    def lbl(t,c=(155,162,180)):
        nonlocal y;surf.blit(fs.render(t,True,c),(px,y));y+=15
    def big(t,c=(240,240,255)):
        nonlocal y;surf.blit(fl.render(t,True,c),(px,y));y+=28
    def row(t,c=(200,215,200)):
        nonlocal y;surf.blit(fm.render(t,True,c),(px,y));y+=17
    def sp(hist,col,h=32):
        nonlocal y;spark(surf,hist,col,px,y,pw,h);y+=h+4
    lc=(0,225,100) if ok else (255,55,55)
    surf.blit(fs.render(f"{'LINK OK' if ok else 'LINK LOST'}   #{data['packet_count']}",True,lc),(px,y));y+=18
    if not ok:surf.blit(fs.render(f"last pkt {link_age:.1f}s ago",True,(200,90,90)),(px,y));y+=15
    sep()
    lbl("altitude");big(f"{data['alt_m']:.1f} m",(90,210,255));sp(data['alt_history'],(90,210,255));sep()
    lbl("pressure");big(f"{data['press_pa']:.0f} Pa",(175,175,255));sep()
    lbl("bay temperature")
    tc=data['temp_c'];tcol=(80,220,130) if tc<40 else (255,200,50) if tc<60 else (255,65,65)
    big(f"{tc:.1f} C",tcol);sp(data['temp_history'],tcol,h=24);sep()
    lbl("acceleration (g)")
    row(f"X  {data['ax']:+6.3f}");row(f"Y  {data['ay']:+6.3f}")
    azcol=(255,140,40) if abs(data['az'])>2 else (200,215,200)
    row(f"Z  {data['az']:+6.3f}",azcol);sp(data['az_history'],(255,175,70),h=28);sep()
    lbl("gyro (deg/s)")
    row(f"X  {data['gx']:+7.2f}");row(f"Y  {data['gy']:+7.2f}");row(f"Z  {data['gz']:+7.2f}");sep()
    lbl("attitude (deg)")
    row(f"roll   {data['roll']:+7.1f}");row(f"pitch  {data['pitch']:+7.1f}");row(f"yaw    {data['yaw']:+7.1f}");sep()
    lbl("drag to orbit   ESC quit",(100,105,120))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',default='COM8');p.add_argument('--baud',type=int,default=9600)
    p.add_argument('--demo',action='store_true');args=p.parse_args()
    W,H=1280,720;state=TelemetryState();attitude=AttitudeFilter(state)
    if args.demo:
        def _demo():
            t=0.0
            while True:
                t+=0.1
                f=[35+12*math.sin(t*.2),101000+600*math.sin(t*.08),27.5+1.5*math.sin(t*.04),
                   .08*math.sin(t*1.1),.06*math.cos(t*.9),1+.15*math.sin(t*1.7),
                   8*math.sin(t*.4),5*math.cos(t*.3),2*math.sin(t*.15)]
                state.update(f);attitude.update(*f[3:9]);time.sleep(0.1)
        threading.Thread(target=_demo,daemon=True).start()
    else:
        threading.Thread(target=serial_thread,args=(args.port,args.baud,state,attitude),daemon=True).start()
    pygame.init()
    pygame.display.set_mode((W,H),DOUBLEBUF|OPENGL)
    pygame.display.set_caption("Theseus — Live Telemetry")
    fonts={'L':pygame.font.SysFont('consolas',20,bold=True),
           'S':pygame.font.SysFont('consolas',12),'M':pygame.font.SysFont('consolas',13)}
    hud=pygame.Surface((W,H),pygame.SRCALPHA)
    glEnable(GL_DEPTH_TEST);glEnable(GL_LIGHTING);glEnable(GL_LIGHT0);glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE)
    glLightfv(GL_LIGHT0,GL_POSITION,[3,5,4,1]);glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
    glLightfv(GL_LIGHT0,GL_AMBIENT,[.25,.25,.25,1])
    glMatrixMode(GL_PROJECTION);glLoadIdentity();gluPerspective(45,W/H,.1,50)
    glMatrixMode(GL_MODELVIEW)
    clock=pygame.time.Clock();cy,cp=25.0,18.0;drag=False;lm=(0,0)
    while True:
        for ev in pygame.event.get():
            if ev.type==QUIT:pygame.quit();sys.exit()
            if ev.type==KEYDOWN and ev.key==K_ESCAPE:pygame.quit();sys.exit()
            if ev.type==MOUSEBUTTONDOWN and ev.button==1:drag=True;lm=ev.pos
            if ev.type==MOUSEBUTTONUP and ev.button==1:drag=False
            if ev.type==MOUSEMOTION and drag:
                dx,dy=ev.pos[0]-lm[0],ev.pos[1]-lm[1]
                cy+=dx*.35;cp=max(-85,min(85,cp+dy*.35));lm=ev.pos
        data=state.get();roll,pitch,yaw=attitude.smooth(0.18)
        glClearColor(.04,.06,.11,1);glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        cr,sr=math.cos(math.radians(cy)),math.sin(math.radians(cy))
        scp,ccp=math.sin(math.radians(cp)),math.cos(math.radians(cp))
        gluLookAt(4.5*cr*ccp,4.5*scp,4.5*sr*ccp,0,.3,0,0,1,0)
        glDisable(GL_LIGHTING);draw_grid();draw_axes();glEnable(GL_LIGHTING)
        glPushMatrix()
        glRotatef(roll,0,0,1);glRotatef(pitch,1,0,0);glRotatef(yaw,0,1,0)
        draw_rocket();glPopMatrix()
        glDisable(GL_DEPTH_TEST);glDisable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION);glPushMatrix();glLoadIdentity()
        glOrtho(0,W,H,0,-1,1);glMatrixMode(GL_MODELVIEW);glPushMatrix();glLoadIdentity()
        hud.fill((0,0,0,0));render_hud(hud,fonts,data,W,H)
        glRasterPos2i(0,0)
        glDrawPixels(W,H,GL_RGBA,GL_UNSIGNED_BYTE,pygame.image.tostring(hud,'RGBA',True))
        glMatrixMode(GL_PROJECTION);glPopMatrix()
        glMatrixMode(GL_MODELVIEW);glPopMatrix()
        glEnable(GL_DEPTH_TEST);glEnable(GL_LIGHTING)
        pygame.display.flip();clock.tick(60)

if __name__=='__main__':main()
