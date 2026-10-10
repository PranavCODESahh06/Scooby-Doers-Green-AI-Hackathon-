"""AquaMind FINAL - one-file Python/Panda3D underwater autonomous simulation.
Install: python3.14 -m pip install panda3d==1.10.16
Run: python3.14 AquaMind_FINAL.py
Controls: SPACE pause; R reset; C cycle cameras; 1-5 cameras; B blood FX;
P paths; D diagnostics; +/- speed; A auto redeploy; ESC exit.
"""
from __future__ import annotations

"""Headless autonomous mission simulation. Units: metres, seconds, modelled Wh."""
from dataclasses import dataclass, field
from enum import Enum
import heapq, math, random

class State(str, Enum):
    DOCKED='DOCKED'; PREPARING='PREPARING'; DEPLOYING='DEPLOYING'; SEARCHING='SEARCHING'
    TARGET_DETECTED='TARGET_DETECTED'; TARGET_VERIFICATION='TARGET_VERIFICATION'
    APPROACHING='APPROACHING'; CAPTURING='CAPTURING'; RETURNING_TO_BASE='RETURNING_TO_BASE'
    DOCKING='DOCKING'; UNLOADING='UNLOADING'; RECHARGING='RECHARGING'
    REDEPLOYING='REDEPLOYING'; MISSION_COMPLETE='MISSION_COMPLETE'; LOW_BATTERY='LOW_BATTERY'; SAFE_HOLD='SAFE_HOLD'

@dataclass
class Config:
    seed:int=42
    lionfish:int=19
    native_fish:int=12
    capacity:int=8
    max_speed:float=6.2
    acceleration:float=5.
    battery_wh:float=160.
    propulsion_wh_per_m:float=.12
    sensor_w:float=5.
    capture_wh:float=1.5
    reserve_wh:float=22.
    detection_range:float=33.
    capture_range:float=3.0
    exclusion:float=4.0
    recharge_seconds:float=13.
    world_radius:float=76.

@dataclass
class Vec:
    x:float=0.; y:float=0.; z:float=0.
    def __add__(self,b):return Vec(self.x+b.x,self.y+b.y,self.z+b.z)
    def __sub__(self,b):return Vec(self.x-b.x,self.y-b.y,self.z-b.z)
    def __mul__(self,s):return Vec(self.x*s,self.y*s,self.z*s)
    __rmul__=__mul__
    def length(self):return math.sqrt(self.x*self.x+self.y*self.y+self.z*self.z)
    def unit(self):return self*(1/max(self.length(),1e-9))
    def dist(self,b):return (self-b).length()
    def tuple(self):return (self.x,self.y,self.z)

@dataclass
class Fish:
    id:int; pos:Vec; native:bool=False; phase:float=0.; speed:float=.5
    heading:float=0.; active:bool=True; state:str='WANDERING'

@dataclass
class Obstacle:
    x:float; y:float; radius:float; height:float=12.; sensitive:bool=False

class Planner:
    """2D grid A* with 3D cruising altitude. Explicit safety clearance."""
    def __init__(self, obstacles:list[Obstacle], cell:float=4., limit:int=19):
        self.obstacles=obstacles; self.cell=cell; self.limit=limit
        self.calls=0
    def blocked(self,p:Vec, extra:list[tuple[Vec,float]]|None=None):
        if abs(p.x)>self.limit*self.cell or abs(p.y)>self.limit*self.cell:return True
        for ob in self.obstacles:
            if math.hypot(p.x-ob.x,p.y-ob.y)<ob.radius+2.:return True
        for q,r in extra or []:
            if math.hypot(p.x-q.x,p.y-q.y)<r:return True
        return False
    def plan(self,start:Vec,goal:Vec,extra:list[tuple[Vec,float]]|None=None)->list[Vec]:
        self.calls+=1; c=self.cell
        cell=lambda p:(round(p.x/c),round(p.y/c))
        s,g=cell(start),cell(goal)
        # destination may lie slightly inside an exclusion margin; never route through it
        if self.blocked(Vec(g[0]*c,g[1]*c,0),extra):return []
        queue=[(0.,s)]; distance={s:0.}; previous={}; found=False
        while queue:
            _,u=heapq.heappop(queue)
            if u==g:found=True;break
            for dx,dy in ((0,1),(1,0),(-1,0),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
                v=(u[0]+dx,u[1]+dy)
                p=Vec(v[0]*c,v[1]*c,goal.z)
                if self.blocked(p,extra):continue
                if dx and dy and (self.blocked(Vec((u[0]+dx)*c,u[1]*c,0),extra) or self.blocked(Vec(u[0]*c,(u[1]+dy)*c,0),extra)):continue
                nd=distance[u]+math.hypot(dx,dy)
                if nd<distance.get(v,float('inf')):
                    distance[v]=nd;previous[v]=u
                    heapq.heappush(queue,(nd+math.hypot(v[0]-g[0],v[1]-g[1]),v))
        if not found:return []
        route=[]; u=g
        while u!=s:
            route.append(Vec(u[0]*c,u[1]*c,goal.z));u=previous[u]
        route.reverse()
        if route:route[-1]=goal
        return route

class Simulation:
    def __init__(self,config:Config|None=None):
        self.cfg=config or Config();self.reset()
    def reset(self):
        c=self.cfg; r=random.Random(c.seed);self.rng=r
        self.base=Vec(-62,-60,29);self.robot=Vec(self.base.x,self.base.y,self.base.z)
        self.velocity=Vec();self.battery=c.battery_wh;self.state=State.DOCKED
        self.state_age=0.;self.elapsed=0.;self.mission_time=0.;self.mission=0;self.deployments=0
        self.onboard=0;self.total_captured=0;self.detected=set();self.target=None
        self.distance=0.;self.energy_propulsion=0.;self.energy_sensors=0.;self.energy_capture=0.
        self.recharge_cycles=0;self.return_trips=0;self.autoredeploy=True;self.running=False;self.paused=False
        self.path=[];self.search_index=0;self.search_stall=0.;self.search_last=Vec(self.robot.x,self.robot.y,self.robot.z);self.capture_age=0.;self.events=[];self.current=Vec(.11,-.065,0)
        self.obstacles=[Obstacle(-20,-27,10,15,True),Obstacle(11,11,11,17,True),Obstacle(40,-24,9,15,True),Obstacle(-35,34,8,16,False),Obstacle(34,42,8,10,False),Obstacle(-7,53,7,8,False)]
        self.planner=Planner(self.obstacles)
        self.fish=[]
        for i in range(c.lionfish+c.native_fish):
            native=i>=c.lionfish
            for _ in range(500):
                p=Vec(r.uniform(-55,60),r.uniform(-54,61),r.uniform(6,17))
                if p.dist(self.base)>17 and not self.planner.blocked(p) and all(p.dist(f.pos)>3. for f in self.fish):break
            self.fish.append(Fish(i,p,native,r.uniform(0,6.28),r.uniform(.3,.9),r.uniform(-math.pi,math.pi)))
        # snake scan covers all quadrants
        self.search_points=[Vec(x,y,10) for j,y in enumerate(range(-55,61,20)) for x in (range(-55,61,20) if j%2==0 else range(55,-61,-20))]
        self.log('System ready  |  Gulf Coast-inspired digital ecosystem')
    def log(self,msg):
        self.events.append((self.elapsed,msg));self.events=self.events[-45:]
    def transition(self,new:State,reason:str|None=None):
        if self.state!=new:
            self.state=new;self.state_age=0.;self.path=[]
            self.log(reason or new.value.replace('_',' ').title())
    def active_lions(self):return [f for f in self.fish if f.active and not f.native]
    def native_exclusions(self):return [(f.pos,self.cfg.exclusion) for f in self.fish if f.native and f.active]
    def safe_zone(self,p:Vec,target:int|None=None):
        if self.planner.blocked(p):return False
        return all(p.dist(f.pos)>self.cfg.exclusion for f in self.fish if f.native and f.active)
    def can_capture(self,f:Fish):
        return f.active and not f.native and self.onboard<self.cfg.capacity and self.robot.dist(f.pos)<=self.cfg.capture_range and self.safe_zone(f.pos)
    def energy_return(self,p:Vec):return p.dist(self.base)*self.cfg.propulsion_wh_per_m*1.7 + self.cfg.reserve_wh
    def eligible(self,f:Fish):
        if f.native or not f.active or not self.safe_zone(f.pos):return False
        estimated=(self.robot.dist(f.pos)*1.6*self.cfg.propulsion_wh_per_m + self.cfg.capture_wh + self.energy_return(f.pos))
        return estimated<self.battery
    def choose_target(self):
        options=[]
        for f in self.active_lions():
            if f.pos.dist(self.robot)>self.cfg.detection_range:continue
            self.detected.add(f.id)
            if not self.eligible(f):continue
            path=self.planner.plan(self.robot,f.pos,self.native_exclusions())
            if path:options.append((sum(path[i].dist(path[i-1] if i else self.robot) for i in range(len(path))),f.id))
        if options:return self.fish[min(options)[1]]
        return None
    def set_path(self,destination:Vec):
        self.path=self.planner.plan(self.robot,destination,self.native_exclusions())
        return bool(self.path) or self.robot.dist(destination)<2.
    def move(self,destination:Vec,dt:float,speed:float|None=None):
        c=self.cfg
        if not self.path and not self.set_path(destination):return False
        while self.path and self.robot.dist(self.path[0])<2.1:self.path.pop(0)
        if not self.path:
            if self.robot.dist(destination)<2.8:self.velocity=self.velocity*.8;return True
            self.path=[destination]
        waypoint=self.path[0]
        # Local native-fish avoidance, plan again if close to moving non-target.
        if any(waypoint.dist(f.pos)<c.exclusion for f in self.fish if f.native and f.active):
            if not self.set_path(destination):return False
            if not self.path:return True
            waypoint=self.path[0]
        desired=(waypoint-self.robot).unit()*min(speed or c.max_speed,max(1.,self.robot.dist(waypoint)*1.8))
        dv=desired-self.velocity; change=min(1.,c.acceleration*dt/max(dv.length(),1e-7))
        self.velocity=self.velocity+dv*change
        proposed=self.robot+self.velocity*dt
        if self.planner.blocked(proposed):self.path=[];self.velocity=Vec();return False
        moved=self.robot.dist(proposed);self.robot=proposed;self.distance+=moved
        e=moved*c.propulsion_wh_per_m*(1.+.05*self.velocity.length()/c.max_speed)
        self.battery=max(0.,self.battery-e);self.energy_propulsion+=e
        return self.robot.dist(destination)<2.8
    def update_fish(self,dt):
        for f in self.fish:
            if not f.active:continue
            near=self.robot.dist(f.pos)<9 and self.state not in (State.DOCKED,State.RECHARGING)
            f.state='EVADING' if near else 'WANDERING'
            if self.state==State.CAPTURING and self.target==f.id:
                f.phase+=dt*.3;continue
            f.phase+=dt*(2. if near else 1.)
            f.heading+=math.sin(f.phase*.37+f.id)*dt*.12
            direction=Vec(math.cos(f.heading),math.sin(f.heading),0)
            if near:direction=(f.pos-self.robot).unit()*.7+direction*.3
            candidate=f.pos+(direction.unit()*f.speed*(1.5 if near else 1.)+self.current*.26)*dt
            candidate.z=max(5.,min(19.,f.pos.z+math.sin(f.phase*.72)*dt*.17))
            if self.planner.blocked(candidate) or abs(candidate.x)>67 or abs(candidate.y)>67:
                f.heading+=math.pi*.67
            else:f.pos=candidate
    def start(self):
        self.running=True;self.paused=False
        if self.state in (State.DOCKED,State.MISSION_COMPLETE):self.transition(State.PREPARING,'Robot prepared for autonomous deployment')
    def update(self,dt):
        if not self.running or self.paused:return
        dt=min(max(dt,0.),.12);self.elapsed+=dt;self.state_age+=dt
        s=self.state;c=self.cfg
        if s not in (State.DOCKED,State.RECHARGING,State.UNLOADING,State.MISSION_COMPLETE):
            energy=c.sensor_w*dt/3600;self.battery=max(0.,self.battery-energy);self.energy_sensors+=energy
        if s not in (State.DOCKED,State.MISSION_COMPLETE):self.mission_time+=dt
        self.update_fish(dt)
        if s in (State.SEARCHING,State.TARGET_DETECTED,State.TARGET_VERIFICATION,State.APPROACHING) and self.battery<self.energy_return(self.robot)+3:
            self.transition(State.LOW_BATTERY,'Energy safety reserve reached');return
        if s==State.PREPARING:
            if self.state_age>.8:
                self.mission+=1;self.deployments+=1;self.mission_time=0.;self.search_index=0
                self.transition(State.DEPLOYING,'Deployment initiated');self.set_path(Vec(-48,-48,11))
        elif s==State.DEPLOYING:
            if self.move(Vec(-48,-48,11),dt):self.transition(State.SEARCHING,'Autonomous search active')
        elif s==State.SEARCHING:
            if self.onboard>=c.capacity or not self.active_lions():self.return_home('Capture capacity reached' if self.onboard>=c.capacity else 'No lionfish remaining');return
            target=self.choose_target()
            if target:
                self.target=target.id;self.transition(State.TARGET_DETECTED,f'Lionfish L-{target.id:02d} detected — simulated sonar')
            else:
                wp=self.search_points[self.search_index%len(self.search_points)]
                if self.robot.dist(self.search_last)<.16:self.search_stall+=dt
                else:self.search_stall=0.;self.search_last=Vec(self.robot.x,self.robot.y,self.robot.z)
                if self.search_stall>12.:
                    self.search_index+=1;self.path=[];self.search_stall=0.;self.log('Search grid replanned around moving exclusion')
                elif not self.path and not self.set_path(wp):
                    self.search_index+=1 # skip temporarily obstructed scan point
                elif self.move(wp,dt):
                    self.search_index+=1;self.path=[]
                if self.search_index>=len(self.search_points):
                    if self.onboard==0:
                        self.autoredeploy=False
                        self.log('Full scan with no captures — auto mode suspended')
                    self.return_home('Search coverage sweep completed');return
        elif s==State.TARGET_DETECTED:
            if self.state_age>.35:self.transition(State.TARGET_VERIFICATION,'Checking simulated species label & habitat safety')
        elif s==State.TARGET_VERIFICATION:
            f=self.fish[self.target] if self.target is not None else None
            if not f or not self.eligible(f):self.target=None;self.transition(State.SEARCHING,'Target rejected by safety filter')
            elif self.state_age>.4:
                if self.set_path(f.pos):self.transition(State.APPROACHING,f'Lionfish L-{f.id:02d} verified — safe approach')
                else:self.target=None;self.transition(State.SEARCHING,'Approach path blocked — replanning')
        elif s==State.APPROACHING:
            f=self.fish[self.target] if self.target is not None else None
            if not f or not self.eligible(f):self.target=None;self.transition(State.SEARCHING,'Aborted: unsafe target or return reserve');return
            if self.path and f.pos.dist(self.path[-1])>7:self.path=[]
            self.move(f.pos,dt,c.max_speed*(.24 if self.robot.dist(f.pos)<12 else 1.0))
            if self.can_capture(f):self.capture_age=0.;self.transition(State.CAPTURING,'Rapid containment sphere extended — target neutralization simulated')
            elif self.state_age>32:self.target=None;self.transition(State.SEARCHING,'Target pursuit timed out')
        elif s==State.CAPTURING:
            f=self.fish[self.target] if self.target is not None else None
            if not f or not f.active:self.target=None;self.transition(State.SEARCHING,'Target unavailable');return
            self.velocity=self.velocity*.78
            # Animation has time to close before success is recorded.
            if self.state_age>=3.2:
                if f.native or self.onboard>=c.capacity or not self.safe_zone(f.pos) or self.robot.dist(f.pos)>c.capture_range+0.6:
                    self.log('Containment cancelled — safety check');self.target=None;self.transition(State.SEARCHING);return
                f.active=False;self.onboard+=1;self.energy_capture+=c.capture_wh;self.battery=max(0.,self.battery-c.capture_wh)
                self.log(f'Target neutralized and stored: L-{f.id:02d}  |  {self.onboard}/8');self.target=None
                if self.onboard>=c.capacity:self.return_home('Eight-fish capacity reached — returning to dock')
                else:self.transition(State.SEARCHING,'Capture secured — search resumed')
        elif s==State.LOW_BATTERY:self.return_home('Low battery — safe return prioritized')
        elif s==State.RETURNING_TO_BASE:
            if not self.path and not self.set_path(self.base):self.transition(State.SAFE_HOLD,'Return route unavailable — recovery required')
            elif self.move(self.base,dt,c.max_speed*.95):self.transition(State.DOCKING,'Fishing vessel rendezvous — docking' )
        elif s==State.DOCKING:
            if self.robot.dist(self.base)>3:self.transition(State.RETURNING_TO_BASE,'Dock alignment lost — retry')
            elif self.state_age>1.7:self.transition(State.UNLOADING,'Boat docked — transferring payload to fisherman')
        elif s==State.UNLOADING:
            if self.state_age>2.2:
                self.total_captured+=self.onboard;self.log(f'Fisherman received {self.onboard} secured lionfish');self.onboard=0
                self.transition(State.RECHARGING,'Recharging power system')
        elif s==State.RECHARGING:
            self.battery=min(c.battery_wh,self.battery+c.battery_wh*dt/c.recharge_seconds)
            if self.battery>=c.battery_wh-1e-6:
                self.recharge_cycles+=1;self.log('Recharge completed — battery 100%')
                if self.autoredeploy and self.active_lions():self.transition(State.REDEPLOYING,'Autonomous redeployment queued')
                else:self.transition(State.MISSION_COMPLETE,'Mission complete — awaiting operator')
        elif s==State.REDEPLOYING:
            if self.state_age>.8:self.transition(State.PREPARING,'Redeploying for next mission')
        elif s==State.SAFE_HOLD:self.velocity=Vec()
    def return_home(self,why):
        self.target=None;self.return_trips+=1
        self.transition(State.RETURNING_TO_BASE,why)
        if not self.set_path(self.base):self.transition(State.SAFE_HOLD,'No safe return path available')
    @property
    def total_energy(self):return self.energy_propulsion+self.energy_sensors+self.energy_capture
    def snapshot(self):
        return dict(state=self.state.value,battery=round(self.battery,2),mission=self.mission,onboard=self.onboard,
                    total=self.total_captured,lionfish=len(self.active_lions()),detected=len(self.detected),
                    distance=round(self.distance,1),energy=round(self.total_energy,2),deployments=self.deployments,
                    returns=self.return_trips,recharges=self.recharge_cycles,elapsed=round(self.elapsed,1))


"""Procedural Panda3D renderer, HUD and keyboard controls. No third-party assets."""
import math, random
from panda3d.core import (loadPrcFileData, Vec3, Vec4, Geom, GeomNode, GeomTriangles,
 GeomVertexData, GeomVertexFormat, GeomVertexWriter, TransparencyAttrib, Fog,
 AmbientLight, DirectionalLight, PointLight, LineSegs, TextNode, CardMaker)
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText

loadPrcFileData('', 'window-title AquaMind | Autonomous Reef Guardian\nwin-size 1450 880\nsync-video true\nshow-frame-rate-meter false\nframebuffer-multisample 1\nmultisamples 4')

def mesh(parent, verts, tris, color):
    data=GeomVertexData('procedural',GeomVertexFormat.getV3n3(),Geom.UHStatic)
    data.setNumRows(len(verts));w=GeomVertexWriter(data,'vertex');n=GeomVertexWriter(data,'normal')
    for x,y,z in verts:
        w.addData3(x,y,z);v=Vec3(x,y,z).normalized();n.addData3(v if v.length()>0 else Vec3(0,0,1))
    pr=GeomTriangles(Geom.UHStatic)
    for a,b,c in tris:pr.addVertices(a,b,c)
    pr.closePrimitive();g=Geom(data);g.addPrimitive(pr);gn=GeomNode('surface');gn.addGeom(g)
    node=parent.attachNewNode(gn);node.setColor(*color);node.setTwoSided(True);return node

def sphere(parent,color,scale=(1,1,1),pos=(0,0,0),rings=10,slices=16):
    vv=[];tt=[]
    for i in range(rings+1):
        phi=math.pi*i/rings
        for j in range(slices+1):
            t=2*math.pi*j/slices;vv.append((math.sin(phi)*math.cos(t),math.sin(phi)*math.sin(t),math.cos(phi)))
    for i in range(rings):
        for j in range(slices):
            a=i*(slices+1)+j;b=a+slices+1;tt.extend(((a,b,a+1),(b,b+1,a+1)))
    node=mesh(parent,vv,tt,color);node.setScale(*scale);node.setPos(*pos);return node

def cylinder(parent,color,radius=1,length=1,sides=10):
    vv=[];tt=[]
    for z in (-length/2,length/2):
        for j in range(sides):
            theta=2*math.pi*j/sides;vv.append((radius*math.cos(theta),radius*math.sin(theta),z))
    for j in range(sides):
        k=(j+1)%sides;tt.extend(((j,k,j+sides),(k,k+sides,j+sides)))
    return mesh(parent,vv,tt,color)

def plate(parent,color,points):
    return mesh(parent,points,[(0,i,i+1) for i in range(1,len(points)-1)],color)

def line(parent,points,color,thick=2):
    if len(points)<2:return None
    seg=LineSegs();seg.setThickness(thick);seg.setColor(*color);seg.moveTo(*points[0]);
    for p in points[1:]:seg.drawTo(*p)
    return parent.attachNewNode(seg.create())

class AquaMindApp(ShowBase):
    def __init__(self):
        super().__init__();self.disableMouse()
        self.sim=Simulation(Config());self.clock=0.;self.speed=1.;self.show_paths=True;self.show_diagnostics=False
        self.camera_mode=0;self.cam_angles=[0,0,0,0,0];self.quality=True
        self.setBackgroundColor(.012,.10,.16,1);self.camLens.setFov(75);self.camLens.setNearFar(.2,520)
        fog=Fog('ocean haze');fog.setColor(.009,.11,.17);fog.setExpDensity(.009)
        self.render.setFog(fog)
        ambient=AmbientLight('ocean fill');ambient.setColor((.30,.52,.60,1));self.render.setLight(self.render.attachNewNode(ambient))
        light=DirectionalLight('filtered sun');light.setColor((.62,.87,1.,1));n=self.render.attachNewNode(light);n.setHpr(-35,-62,0);self.render.setLight(n)
        self.world=self.render.attachNewNode('ecosystem');self.make_world()
        self.models={};self.make_fish();self.make_robot();self.make_hud();self.make_pov_overlay();self.make_controls()
        self.scan_fx=self.render.attachNewNode("rotating sonar returns")
        self.scan_time=0.0;self.capture_flash=0.0;self.show_blood=True
        self.navigation_node=self.render.attachNewNode('planned_route');self.capture_fx=self.render.attachNewNode('containment sphere')
        self.particles=[];r=random.Random(11)
        for _ in range(125):
            p=sphere(self.render,(.5,.88,1,.28),(.08,.08,.08),(r.uniform(-80,80),r.uniform(-80,80),r.uniform(3,33)),5,7)
            p.setTransparency(TransparencyAttrib.MAlpha);p.setLightOff();self.particles.append((p,r.uniform(.2,.8)))
        self.taskMgr.add(self.frame,'AquaMind-Loop');self.sim.start()
    def make_world(self):
        r=random.Random(5)
        # Individually shaded seabed tiles make the surface visibly textured at a distance.
        size=10
        for x in range(-9,9):
            for y in range(-9,9):
                c=r.uniform(-.028,.028);col=(.10+c,.25+c,.24+c,1)
                a=x*size;b=y*size
                plate(self.world,col,[(a,b,-1.4),(a+size,b,-1.4),(a+size,b+size,-1.4),(a,b+size,-1.4)])
        # Biogenic structures, asymmetric rocks and artificial reef columns.
        for ob in self.sim.obstacles:
            root=self.world.attachNewNode('reef exclusion habitat');root.setPos(ob.x,ob.y,0)
            sphere(root,(.18,.28,.27,1),(ob.radius,ob.radius*.8,ob.height*.45),(0,0,ob.height*.25))
            for j in range(7):
                ang=j*math.tau/7;bx=math.cos(ang)*ob.radius*.65;by=math.sin(ang)*ob.radius*.65
                stem=cylinder(root,(.24,.40,.36,1),r.uniform(.5,1.3),r.uniform(4,9),8)
                stem.setPos(bx,by,2+r.uniform(0,3));stem.setHpr(ang*57, r.uniform(-22,22),0)
                if j%2==0:sphere(root,(.68,.35,.32,1),(1.4,1.4,.8),(bx,by,6))
            # visible conservation perimeter
            points=[(math.cos(i*math.tau/48)*(ob.radius+2),math.sin(i*math.tau/48)*(ob.radius+2),.4) for i in range(49)]
            ring=line(root,points,(.35,.91,.83,.31),1.5)
        for _ in range(75):
            x=r.uniform(-77,77);y=r.uniform(-77,77)
            if math.hypot(x+62,y+60)<14:continue
            h=r.uniform(.3,2.2)
            sphere(self.world,(r.uniform(.12,.21),r.uniform(.26,.42),.28,1),(h*1.6,h,h*.7),(x,y,0))
        # Underwater base: three joined pressure vessel modules, port, lights, suspended deck
        base=self.world.attachNewNode('Research & containment station');base.setPos(-62,-60,0)
        plate(base,(.10,.33,.39,1),[(-12,-11,2),(12,-11,2),(12,11,2),(-12,11,2)])
        for x in (-6,6):
            hull=sphere(base,(.16,.39,.46,1),(5,6,5),(x,1,7))
            sphere(base,(.11,.77,.83,1),(3.6,.45,3.6),(x,-5,7))
            for dz in (4,7,10):
                sphere(base,(.8,.96,1,1),(.35,.3,.35),(x-1.4,-5.5,dz))
        for x in (-9,9):
            for y in (-9,9):cylinder(base,(.26,.46,.5,1),.8,4).setPos(x,y,1)
        self.dock=sphere(base,(.22,.8,.88,1),(4.1,3.1,.3),(0,0,9));self.dock.setTransparency(TransparencyAttrib.MAlpha)
        for x in (-10,10):
            lamp=PointLight('beacon');lamp.setColor((.20,.85,1,1));lamp.setAttenuation((1,0,.02))
            node=base.attachNewNode(lamp);node.setPos(x,-8,13);base.setLight(node)
            sphere(base,(.45,1,1,1),(.55,.55,.55),(x,-8,13))
        # Faint illuminated ocean surface, visible from below without blocking view.
        surface=plate(self.world,(.12,.58,.68,.17),[(-95,-95,32),(95,-95,32),(95,95,32),(-95,95,32)])
        surface.setTransparency(TransparencyAttrib.MAlpha);surface.setTwoSided(True);surface.setLightOff()
        # Fishing support vessel at the surface; the AUV surfaces underneath
        # for handover and rapid battery exchange. Ocean surface is at z=32.
        self.boat=self.world.attachNewNode('fisherman support vessel')
        self.boat.setPos(-62,-60,32)
        sphere(self.boat,(.86,.88,.83,1),(10,3.5,1.8),(0,0,0))
        sphere(self.boat,(.14,.37,.48,1),(8.9,3.1,.85),(0,0,.85))
        plate(self.boat,(.85,.89,.87,1),[(-4,-2,1.35),(3,-2,1.35),(3,2,1.35),(-4,2,1.35)])
        sphere(self.boat,(.9,.92,.9,1),(2.5,2.1,2.4),(-1,0,3.2))
        sphere(self.boat,(.11,.67,.78,1),(1.9,2.12,.72),(.2,0,3.5))
        mast=cylinder(self.boat,(.79,.80,.76,1),.13,9,10);mast.setPos(-1.5,0,7)
        line(self.boat,[(-1.5,0,12),(-6,0,3.5)],(.87,.89,.86,1),2)
        for yy in (-2.2,2.2):
            sphere(self.boat,(.98,.68,.19,1),(.57,.57,.55),(4,yy,1.0))
        # Procedural crew silhouette is an illustrative fisherman (not human simulation).
        crew=self.boat.attachNewNode('fisherman character')
        sphere(crew,(.15,.25,.33,1),(.48,.40,1.0),(4.8,0,2.65))
        sphere(crew,(.66,.47,.35,1),(.35,.33,.43),(4.8,0,3.86))
        cylinder(crew,(.21,.28,.36,1),.68,.17,12).setPos(4.8,0,4.27)
        for y in (-.23,.23):
            leg=cylinder(crew,(.18,.24,.29,1),.18,.9,10);leg.setPos(4.8,y,1.63)
        self.payload_lights=[]
        # Animated kelp and branching coral silhouettes.
        self.kelp=[]
        for i in range(92):
            xx=r.uniform(-72,72); yy=r.uniform(-72,72)
            if math.hypot(xx+62,yy+60)<16:continue
            parent=self.world.attachNewNode('current-driven sea vegetation')
            parent.setPos(xx,yy,0)
            h=r.uniform(1.8,5.2)
            for j in range(3):
                a=j*2.094+r.uniform(-.3,.3)
                line(parent,[(0,0,0),(.3*math.cos(a),.3*math.sin(a),h*.4),(.9*math.cos(a),.9*math.sin(a),h)],(.16,.48,.32,1),r.uniform(1.5,3.2))
            self.kelp.append((parent,r.uniform(0,6.28)))
        # Layered sea fans and coral branches for silhouettes at varied scales.
        for n in range(135):
            x=r.uniform(-78,78); y=r.uniform(-78,78)
            if math.hypot(x+62,y+60)<15:continue
            z=-.8
            root=self.world.attachNewNode('branching coral garden');root.setPos(x,y,z)
            hue=r.choice(((.53,.31,.27,1),(.68,.43,.29,1),(.28,.52,.48,1),(.48,.36,.53,1)))
            height=r.uniform(.6,2.8)
            for j in range(5):
                ang=j*math.tau/5+r.uniform(-.2,.2)
                end=(math.cos(ang)*height*.55, math.sin(ang)*height*.55,height)
                mid=(end[0]*.35,end[1]*.35,height*.55)
                line(root,[(0,0,0),mid,end],hue,1.3 if n%2 else 2.5)
                line(root,[mid,(end[0]*1.3,end[1]*1.3,height*.75)],hue,1.5)
        # Caustic-inspired, translucent sun shafts
        for i in range(8):
            beam=cylinder(self.world,(.15,.78,.87,.045),r.uniform(2.5,5.5),72,12)
            beam.setPos(r.uniform(-65,65),r.uniform(-65,65),28);beam.setTransparency(TransparencyAttrib.MAlpha);beam.setLightOff()
    def make_fish(self):
        for f in self.sim.fish:
            root=self.render.attachNewNode('Native reef fish' if f.native else 'Invasive lionfish')
            rig=root.attachNewNode('swimming rig')
            if f.native:
                sphere(rig,(.09,.72,.77,1),(1.35,.36,.58))
                tail=plate(rig,(.94,.85,.36,1),[(-1.1,0,0),(-2.2,0,.7),(-2.2,0,-.7)])
                sphere(rig,(.94,.89,.60,1),(.12,.14,.12),(1.,-.32,.16))
                fins=[]
            else:
                sphere(rig,(.68,.28,.24,1),(1.55,.52,.71))
                # Alternating high-contrast vertical body bands, curved via ellipsoids.
                for x in (-1.05,-.55,0,.55,1.05):
                    scale=.60 if abs(x)>1 else 1.
                    sphere(rig,(.96,.88,.74,1),(.12,.54*scale,.73*scale),(x,0,0))
                sphere(rig,(.80,.39,.32,1),(.7,.47,.55),(1.16,0,.03))
                for yy in (-.42,.42):sphere(rig,(.03,.07,.11,1),(.13,.09,.14),(1.48,yy,.22))
                for i in range(11):
                    x=-1.15+i*.22
                    spine=plate(rig,(.84,.63,.49,1),[(x,0,.44),(x+.06,0,1.5+math.sin(i*.4)*.4),(x+.16,0,.40)])
                fins=[]
                for sign in (-1,1):
                    anchor=rig.attachNewNode('pectoral fan')
                    anchor.setPos(-.1,sign*.44,-.05)
                    for j in range(8):
                        k=(j-3.5)/7
                        plate(anchor,(.68+.035*j,.45,.36,1),[(0,0,0),(-.4-k*.7,sign*(1.1+j*.13),.2+k*.7),(-.1-k*.12,sign*.16,.1)])
                    fins.append((anchor,sign))
                tail=plate(rig,(.85,.61,.42,1),[(-1.22,0,0),(-2.55,0,1.0),(-2.34,0,0),(-2.55,0,-1.)])
            self.models[f.id]=(root,rig,tail,fins)
    def make_robot(self):
        """Procedural visual interpretation of the supplied FreeCAD half-disc concept.

        The uploaded macro defines a 600 mm half-disc hull, central capture tunnel,
        two rows of flaps, side thrusters and rear containment cage. The harpoon
        shown in the original macro is intentionally not represented or actuated.
        """
        self.robot_root=self.render.attachNewNode('Half-disc capture AUV | expanded eight-fish payload')
        self.robot_root.setScale(1.72)  # 23% larger than V6; illustrative, not a physical capacity validation
        hull=self.robot_root.attachNewNode('red semi-disc wing')
        # Semi-circular wing pointing forward (+X); body is a curved planform,
        # not the generic submarine shape from the first prototype.
        for i in range(15):
            a=-math.pi/2+i*math.pi/15; b=-math.pi/2+(i+1)*math.pi/15
            plate(hull,(.53,.085,.10,1),[(.0,0,.02),(-.7+2.8*math.cos(a),2.8*math.sin(a),.02),(-.7+2.8*math.cos(b),2.8*math.sin(b),.02)])
            plate(hull,(.69,.14,.16,1),[(.0,0,-.08),(-.7+2.8*math.cos(a),2.8*math.sin(a),-.08),(-.7+2.8*math.cos(b),2.8*math.sin(b),-.08)])
        sphere(self.robot_root,(.73,.16,.18,1),(2.15,.34,.34),(-.3,0,.1))
        # Central open channel and eight-position holding chamber.
        tunnel=sphere(self.robot_root,(.08,.18,.23,1),(2.7,.39,.38),(.2,0,-.18))
        for x in (-2.,-1.2,-.4,.4,1.2):
            ring=cylinder(self.robot_root,(.82,.79,.68,1),.48,.10,14)
            ring.setPos(x,0,-.20);ring.setP(90)
        for side in (-1,1):
            pod=sphere(self.robot_root,(.16,.22,.26,1),(1.05,.33,.32),(-1.0,side*2.05,-.14))
            rim=cylinder(self.robot_root,(.17,.75,.85,1),.35,.65,12);rim.setP(90);rim.setPos(-1.55,side*2.05,-.14)
            sphere(self.robot_root,(.11,.82,.92,1),(.30,.23,.22),(1.5,side*.74,.15))
            # Pivoting safety flaps along central intake (visual only).
            for j in range(4):
                x=.95-j*.49
                plate(self.robot_root,(.89,.64,.27,1),[(x,side*.33,-.2),(x-.38,side*.54,-.38),(x-.38,side*.33,.06)])
        self.sensor=sphere(self.robot_root,(.14,.93,1,1),(.36,.39,.33),(2.18,0,.02))
        self.camera_lens=cylinder(self.robot_root,(.075,.11,.17,1),.24,.36,20)
        self.camera_lens.setP(90);self.camera_lens.setPos(2.46,0,.02)
        sphere(self.robot_root,(.08,.85,1,1),(.11,.19,.19),(2.70,0,.02))
        # Eight visible individually partitioned storage cells, represented by ribs.
        for x in (-2.35,-1.69,-1.03,-.37,.29):
            rib=cylinder(self.robot_root,(.40,.79,.82,1),.66,.065,18)
            rib.setPos(x,0,.72);rib.setP(90)
        for y in (-.54,.54):
            line(self.robot_root,[(-2.55,y,.90),(.50,y,.90)],(.45,.86,.88,.85),2.5)
        self.front_leds=[]
        for side in (-1,1):
            node=sphere(self.robot_root,(.77,1,.91,1),(.14,.17,.16),(2.04,side*.6,-.04))
            self.front_leds.append(node)
        self.reach_guide=line(self.robot_root,[(0,0,.16),(3.0,0,.16)],(.26,1,.80,.42),1.5)
        self.payload=sphere(self.robot_root,(.12,.30,.35,.82),(1.85,1.06,.68),(-1.35,0,.72))
        self.payload.setTransparency(TransparencyAttrib.MAlpha)
        self.payload_lights=[]
        for j in range(8):
            x=-2.35+(j%4)*.66; yy=-.54+(j//4)*1.08
            self.payload_lights.append(sphere(self.robot_root,(.19,.31,.38,1),(.20,.18,.14),(x,yy,1.30)))
        # Twin shrouded axial thrusters. Each rotor spins about the forward X axis;
        # the old V6 rotated flat blades around world/local Z, making them wobble.
        # Counter-rotation cancels the visible swirl, and the speed responds to
        # actual commanded movement instead of free-running at a constant rate.
        self.rotors=[]
        self.propeller_phase=[0.0, 0.0]
        for side in (-1, 1):
            nacelle=self.robot_root.attachNewNode('port ducted thruster' if side < 0 else 'starboard ducted thruster')
            nacelle.setPos(-1.76, side*2.05, -.14)
            # Cylinders are aligned to X; duct shows two rims and 16 slender braces.
            for xx in (-.34, .34):
                for j in range(24):
                    aa=j*math.tau/24;bb=(j+1)*math.tau/24
                    line(nacelle,[(xx,.44*math.cos(aa),.44*math.sin(aa)),
                                  (xx,.44*math.cos(bb),.44*math.sin(bb))],(.16,.40,.46,1),3)
            for j in range(8):
                aa=j*math.tau/8
                line(nacelle,[(-.34,.44*math.cos(aa),.44*math.sin(aa)),
                              (.34,.44*math.cos(aa),.44*math.sin(aa))],(.13,.31,.37,1),2)
            rotor=nacelle.attachNewNode('rotating axial rotor')
            # Five swept, pitched blades represented by two triangles each.
            for j in range(5):
                a=j*math.tau/5
                def pt(rad,ang,x):return (x,rad*math.cos(ang),rad*math.sin(ang))
                p0=pt(.10,a-.10,-.03)
                p1=pt(.40,a+.12,.12)
                p2=pt(.39,a+.52,.01)
                p3=pt(.13,a+.32,-.08)
                plate(rotor,(.21,.80,.86,1),[p0,p1,p2,p3])
                line(rotor,[p0,p1,p2,p3],(.78,.98,1,1),1.5)
            sphere(rotor,(.08,.23,.30,1),(.23,.13,.13),(0,0,0))
            sphere(nacelle,(.15,.28,.34,1),(.25,.15,.15),(.43,0,0))
            self.rotors.append(rotor)
    def make_hud(self):
        cm=CardMaker('hud');cm.setFrame(-1.31,-.54,-.95,.92)
        bg=self.aspect2d.attachNewNode(cm.generate());bg.setColor(.01,.07,.11,.83);bg.setTransparency(TransparencyAttrib.MAlpha)
        cm2=CardMaker('log panel');cm2.setFrame(.68,1.31,-.95,-.34)
        logbg=self.aspect2d.attachNewNode(cm2.generate());logbg.setColor(.01,.07,.11,.81);logbg.setTransparency(TransparencyAttrib.MAlpha)
        self.texts={}
        def t(key,title,x,z,size=.041,color=(.8,.96,1,1)):
            ob=OnscreenText(text=title,pos=(x,z),scale=size,fg=color,align=TextNode.ALeft,mayChange=True,parent=self.aspect2d,shadow=(0,0,0,.5));self.texts[key]=ob
        t('brand','A Q U A M I N D',-1.27,.82,.071,(.30,.96,.91,1))
        t('tag','AUTONOMOUS REEF GUARDIAN / FISHERMAN SUPPORT BOAT',-1.27,.75,.031)
        for idx,key in enumerate(('state','mission','target','battery','capacity','total','remaining','detected','distance','energy','time','power')):
            t(key,'',-1.26,.62-idx*.119,.038)
        t('controls','SPACE pause  |  R reset  |  C camera\nA auto  |  P path  |  D debug  |  +/- speed\n1-5 cameras (5 = ROBOT POV) | G quality | B blood FX',-.52,-.87,.034,(.73,.90,.96,1))
        t('logtitle','MISSION EVENT STREAM',.73,-.39,.043,(.35,.97,.83,1))
        t('log','',.73,-.49,.030,(.83,.93,.92,1))
        t('diag','',-.4,.87,.032,(1,.8,.53,1))
        t('scanhead','LIVE CAMERA / PROXIMITY',.73,.84,.036,(.34,.97,.89,1))
        t('scan','',.73,.72,.029,(.85,.97,.96,1))
        t('classifier','',.73,.48,.030,(.99,.84,.61,1))
        t('depth','',.73,.23,.031,(.83,.96,1,1))
        t('mechanism','',.73,.08,.032,(.39,1,.84,1))
        t('label','GULF COAST-INSPIRED MODEL  •  NOT REAL-WORLD SPECIES AI',-.50,.95,.03)
        t('pov','',-.49,.77,.040,(.45,1,.79,1))
    def make_pov_overlay(self):
        """Readable instrument overlays visible only in robot POV mode."""
        self.pov_overlay=self.aspect2d.attachNewNode('ROBOT CAMERA FLIGHT INSTRUMENTS')
        def label(message,x,y,scale=.040,color=(.48,1.,.86,1)):
            return OnscreenText(text=message,pos=(x,y),scale=scale,fg=color,
                                align=TextNode.ALeft,mayChange=True,parent=self.pov_overlay,
                                shadow=(0,0,0,.8))
        self.pov_instruments=label('',-.49,.64,.039)
        self.pov_bottom=label('',-.53,-.67,.037)
        # targeting brackets and center crosshair, drawn as real screen-space lines
        self.pov_crosshair=self.aspect2d.attachNewNode('camera reticle')
        self.pov_crosshair.reparentTo(self.pov_overlay)
        tint=(.30,1,.80,.90)
        for pts in [[(-.11,0,0),(-.035,0,0)],[(.035,0,0),(.11,0,0)],
                    [(0,0,-.11),(0,0,-.035)],[(0,0,.035),(0,0,.11)],
                    [(-.18,0,.15),(-.18,0,.21),(-.12,0,.21)],
                    [(.12,0,.21),(.18,0,.21),(.18,0,.15)],
                    [(-.18,0,-.15),(-.18,0,-.21),(-.12,0,-.21)],
                    [(.12,0,-.21),(.18,0,-.21),(.18,0,-.15)]]:
            line(self.pov_crosshair,pts,tint,2.6)
        self.pov_overlay.hide()
    def make_controls(self):
        self.accept('space',self.pause);self.accept('r',self.reset)
        self.accept('c',self.camera_next);self.accept('a',self.auto)
        self.accept('p',self.paths);self.accept('d',self.diagnostics)
        self.accept('+',self.faster);self.accept('=',self.faster);self.accept('-',self.slower)
        self.accept('g',self.toggle_quality);self.accept('b',self.toggle_blood)
        for i in range(5):self.accept(str(i+1),self.camera_select,[i])
        self.accept('escape',self.userExit)
    def toggle_blood(self):
        self.show_blood=not self.show_blood
        self.sim.log('Stylized blood FX: '+('ON' if self.show_blood else 'OFF'))
    def pause(self):
        if not self.sim.running:self.sim.start()
        else:self.sim.paused=not self.sim.paused
        self.sim.log('PAUSED' if self.sim.paused else 'RESUMED')
    def reset(self):
        self.sim.reset();self.sim.start()
        for f in self.sim.fish:self.models[f.id][0].show()
        self.navigation_node.removeNode();self.navigation_node=self.render.attachNewNode('navigation')
    def auto(self):self.sim.autoredeploy=not self.sim.autoredeploy;self.sim.log('Auto-redeploy: '+str(self.sim.autoredeploy))
    def paths(self):self.show_paths=not self.show_paths
    def diagnostics(self):self.show_diagnostics=not self.show_diagnostics
    def faster(self):self.speed=min(5.,self.speed+.5)
    def slower(self):self.speed=max(.5,self.speed-.5)
    def camera_next(self):self.camera_mode=(self.camera_mode+1)%5
    def camera_select(self,i):self.camera_mode=i
    def toggle_quality(self):
        self.quality=not self.quality
        for p,_ in self.particles:p.show() if self.quality else p.hide()
    def frame(self,task):
        dt=min(globalClock.getDt(),.08);self.clock+=dt
        # cap step for stable deterministic physics even on speed 5
        for _ in range(max(1,math.ceil(dt*self.speed/.06))):self.sim.update(dt*self.speed/max(1,math.ceil(dt*self.speed/.06)))
        self.update_models(dt);self.update_camera(dt);self.update_hud()
        return task.cont
    def update_models(self,dt):
        s=self.sim; rp=s.robot
        self.robot_root.setPos(*rp.tuple())
        if s.velocity.length()>.05:
            heading=math.degrees(math.atan2(s.velocity.y,s.velocity.x))
            self.robot_root.setH(heading);self.robot_root.setP(-math.degrees(math.atan2(s.velocity.z,max(.01,math.hypot(s.velocity.x,s.velocity.y)))))
        # Forward thrust in the simulation is along +X, so R rotates blades
        # in the YZ plane (H would incorrectly spin them in the XY plane).
        # Idle slowly; speed ramps with robot velocity and simulation speed.
        speed_ratio=min(1.,s.velocity.length()/max(.01,s.cfg.max_speed))
        rotor_dps=65.0+1120.0*speed_ratio
        for i,rotor in enumerate(self.rotors):
            self.propeller_phase[i]=(self.propeller_phase[i]+dt*rotor_dps*(1 if i==0 else -1))%360
            rotor.setR(self.propeller_phase[i])
        self.sensor.setColorScale(1,.6+.3*math.sin(self.clock*5),1,1)
        for i,lamp in enumerate(self.payload_lights):lamp.setColor((.25,.95,.62,1) if i<s.onboard else (.18,.31,.39,1))
        for f in s.fish:
            root,rig,tail,fins=self.models[f.id]
            if not f.active:root.hide();continue
            root.show();root.setPos(*f.pos.tuple());root.setH(math.degrees(f.heading))
            rig.setZ(math.sin(f.phase*2)*.13);tail.setH(math.sin(f.phase*6)*18)
            for node,sign in fins:node.setP(sign*(7+math.sin(f.phase*4+sign)*12))
        for kelp,phase in self.kelp:kelp.setH(math.sin(self.clock*.7+phase)*4)
        self.boat.setZ(32+math.sin(self.clock*.55)*.19)
        for p,speed in self.particles:
            x,y,z=p.getPos();p.setPos(x+dt*.10,y-dt*.065,3 if z>36 else z+dt*speed)
        # Animated sonar pulses emanate from forward sensor; proximity is a geometric
        # range calculation, NOT real machine-learning image classification.
        self.scan_fx.removeNode();self.scan_fx=self.render.attachNewNode('proximity sonar')
        if s.running and not s.paused and s.state not in (State.DOCKED,State.MISSION_COMPLETE):
            forward=self.robot_root.getQuat(self.render).xform(Vec3(1,0,0))
            origin=self.robot_root.getPos()+forward*3.70
            phase=(self.clock*1.65)%1.
            for i in range(3):
                dist=1.2+((phase+i/3)%1)*10
                radius=.25+dist*.26
                # Orient transverse sonar arcs in world space.
                right=Vec3(-forward.y,forward.x,0).normalized()
                up=forward.cross(right).normalized()
                pts=[origin+forward*dist+right*(radius*math.cos(j*math.tau/36))+up*(radius*math.sin(j*math.tau/36)) for j in range(37)]
                line(self.scan_fx,[tuple(v) for v in pts],(.24,.95,.92,.55),1.5)
            line(self.scan_fx,[tuple(origin),tuple(origin+forward*11)],(.3,.95,1,.45),1.)
        # A fast, clearly animated containment-and-neutralization demonstration.
        # The red cloud is a stylized visual effect, not real fluid dynamics.
        self.capture_fx.removeNode();self.capture_fx=self.render.attachNewNode('capture mechanism and water effects')
        if s.state==State.CAPTURING and s.target is not None:
            f=s.fish[s.target];age=s.state_age
            robot_pos=Vec3(*rp.tuple());fish_pos=Vec3(*f.pos.tuple())
            offset=fish_pos-robot_pos
            dist=max(.01,offset.length());direction=offset/dist
            # Extension is deliberately fast: 0.22 seconds to deploy a telescoping arm.
            extend=min(1.,age/.22)
            tube_end=robot_pos+direction*(min(dist,3.)*extend)
            line(self.capture_fx,[tuple(robot_pos+direction*1.1),tuple(tube_end)],(.78,.93,.94,1),8)
            line(self.capture_fx,[tuple(robot_pos+direction*1.1+Vec3(0,0,.13)),tuple(tube_end+Vec3(0,0,.13))],(.16,.36,.44,1),4)
            # The launch collar visibly travels from the bow to the fish.
            collar=sphere(self.capture_fx,(.66,.92,1,.85),(.28,.28,.28),tuple(tube_end),10,18)
            collar.setTransparency(TransparencyAttrib.MAlpha)
            active=max(0.,age-.15)
            grow=min(1.,active/.34)
            retract=min(1.,max(0.,(age-2.0)/1.1))
            radius=(.16+1.55*grow)*(1.-.50*retract)
            center=robot_pos+(fish_pos-robot_pos)*min(1.,max(0.,age/.35))
            if age>.38:center=fish_pos
            if age>2.10:
                # Visible retrieval toward the storage chamber before fish is removed.
                t=min(1.,(age-2.10)/1.05)
                # Payload is aft, below the centerline.
                storage_world=self.robot_root.getPos(self.render)+self.robot_root.getQuat(self.render).xform(Vec3(-1.35,0,.72)*self.robot_root.getSx())
                center=fish_pos*(1-t)+storage_world*t
                self.models[f.id][0].setPos(center)
                self.models[f.id][1].setZ(0)
                self.models[f.id][2].setH(0)
            bubble=sphere(self.capture_fx,(.18,.94,.96,.24),(radius,radius,radius),tuple(center),16,32)
            bubble.setTransparency(TransparencyAttrib.MAlpha);bubble.setLightOff()
            for axis in (0,1,2):
                pts=[]
                for j in range(49):
                    a=j*math.tau/48;v=[math.cos(a)*radius,math.sin(a)*radius,0]
                    if axis==1:v=[v[0],v[2],v[1]]
                    elif axis==2:v=[v[2],v[0],v[1]]
                    pts.append((center.x+v[0],center.y+v[1],center.z+v[2]))
                line(self.capture_fx,pts,(.23,1,.96,.92),3)
            # Brief red particulate plume marks the simulated lethal event.
            # It is bounded, fades, and disperses with the modelled current.
            if self.show_blood and .57<age<2.15:
                rng=random.Random(f.id+100)
                opacity=max(0.,min(1.,(age-.57)*4.,(2.15-age)*1.2))
                for i in range(65):
                    angle=rng.uniform(0,math.tau);spread=rng.uniform(.08,1.25)*(1+max(0.,age-.57)*.8)
                    drift=(age-.57)*.45
                    location=(fish_pos.x+math.cos(angle)*spread+drift,
                              fish_pos.y+math.sin(angle)*spread-drift*.45,
                              fish_pos.z+rng.uniform(-1.,1.)*spread+.12*drift)
                    mote=sphere(self.capture_fx,(.63,.045,.09,.12*opacity),
                                (.06+.075*spread,)*3,location,5,8)
                    mote.setTransparency(TransparencyAttrib.MAlpha);mote.setLightOff()
            if age>.5:
                for i in range(9):
                    a=i*math.tau/9+self.clock*2
                    dot=sphere(self.capture_fx,(.54,1,.96,.85),(.09,)*3,
                               tuple(center+Vec3(math.cos(a)*radius,math.sin(a)*radius,math.sin(a*2)*radius*.45)),5,8)
                    dot.setLightOff()
        self.navigation_node.removeNode();self.navigation_node=self.render.attachNewNode('navigation')
        if self.show_paths and s.path:
            line(self.navigation_node,[rp.tuple()]+[v.tuple() for v in s.path],(.12,1,.86,.95),3)
            for v in s.path[::2]:sphere(self.navigation_node,(.1,1,.8,1),(.3,.3,.3),v.tuple(),5,8)
    def update_camera(self,dt):
        p=self.sim.robot;rv=Vec3(*p.tuple())
        h=math.radians(self.robot_root.getH());back=Vec3(math.cos(h),math.sin(h),0)
        if self.camera_mode==0:goal=rv-back*24+Vec3(0,0,12);look=rv+back*4
        elif self.camera_mode==1:goal=Vec3(8,-128,98);look=Vec3(0,0,8)
        elif self.camera_mode==2:goal=rv-back*12+Vec3(0,0,3);look=rv+back*9
        elif self.camera_mode==3:goal=Vec3(-88,-95,59);look=Vec3(-62,-60,29)
        else:
            # Forward-mounted simulated camera, oriented along the robot's actual
            # animated local x-axis. The lens is at (2.70,0,.02) on the hull.
            # It is a view of the 3D environment, NOT real computer vision.
            # Convert the lens's LOCAL coordinates INTO world-space coordinates.
            # Previous version reversed getRelativePoint, pointing away from the scene.
            lens_world=self.render.getRelativePoint(self.robot_root,Vec3(3.7,0,.23))
            forward_world=self.render.getRelativePoint(self.robot_root,Vec3(30,0,.23))
            up_world=self.robot_root.getQuat(self.render).xform(Vec3(0,0,1))
            self.camera.setPos(lens_world)
            self.camera.lookAt(forward_world,up_world)
            return
        pos=self.camera.getPos();self.camera.setPos(pos+(goal-pos)*min(1.,dt*2.5));self.camera.lookAt(look)
    def update_hud(self):
        s=self.sim; c=s.cfg;v=self.texts
        self.pov_overlay.show() if self.camera_mode==4 else self.pov_overlay.hide()
        if self.camera_mode==4:
            depth=max(0.,32-s.robot.z)
            speed=s.velocity.length()
            alt=max(0.,s.robot.z)
            heading=(self.robot_root.getH()+360)%360
            self.pov_instruments.setText(f'DEPTH     {depth:05.1f} m\nALTITUDE  {alt:05.1f} m\nHEADING   {heading:05.1f}°\nVELOCITY  {speed:04.1f} m/s')
            tg=s.fish[s.target] if s.target is not None else None
            if tg and tg.active:
                distance=s.robot.dist(tg.pos)
                contact=f'LIONFISH LOCK   L-{tg.id:02d}  |  {distance:.1f} m'
            else:
                contact='SCANNING FOR LIONFISH  |  NO TARGET LOCK'
            self.pov_bottom.setText(f'{contact}\nBATTERY {s.battery/s.cfg.battery_wh*100:.0f}%  •  PAYLOAD {s.onboard}/8  •  {s.state.value.replace("_"," ")}')
        v['state'].setText('MODE   '+s.state.value.replace('_',' ') + ('  [PAUSED]' if s.paused else ''))
        v['pov'].setText('● ROBOT CAMERA / SIMULATED OPTICS  •  FORWARD VIEW' if self.camera_mode==4 else '')
        v['mission'].setText(f'MISSION   {s.mission:02d}    DEPLOYMENTS {s.deployments}')
        v['target'].setText('TARGET   '+('L-%02d'%s.target if s.target is not None else 'SCANNING / NONE'))
        v['battery'].setText(f'BATTERY  {s.battery/c.battery_wh*100:05.1f}%  /  {s.battery:05.1f} Wh')
        v['capacity'].setText('CAPACITY  '+'● '*s.onboard+'○ '*(c.capacity-s.onboard)+f' {s.onboard}/8')
        v['total'].setText(f'TOTAL SECURED  {s.total_captured}   |   ONBOARD {s.onboard}')
        v['remaining'].setText(f'LIONFISH REMAINING  {len(s.active_lions())}')
        v['detected'].setText(f'IDENTIFIED CONTACTS  {len(s.detected)}')
        v['distance'].setText(f'DISTANCE   {s.distance:.1f} m')
        v['energy'].setText(f'ENERGY USED   {s.total_energy:.2f} Wh')
        v['time'].setText(f'MISSION TIME   {s.mission_time:.0f} s')
        v['power'].setText(f'SPEED {self.speed:.1f}x  |  AUTO {"ON" if s.autoredeploy else "OFF"}')
        recent=s.events[-7:];v['log'].setText('\n'.join(f'{sec:04.0f}s  {msg[:35]}' for sec,msg in recent))
        nearest=min((f for f in s.fish if f.active),key=lambda f:s.robot.dist(f.pos),default=None)
        measured=s.robot.dist(nearest.pos) if nearest else None
        nearest_label=('NATIVE - EXCLUDE' if nearest.native else 'LIONFISH - CANDIDATE') if nearest else 'NO CONTACT'
        target=s.fish[s.target] if s.target is not None else None
        v['scan'].setText('SENSOR   ACTIVE / SIMULATED\nSCAN RANGE  %.0f m\nNEAREST    %s\nDISTANCE   %s' %
           (s.cfg.detection_range,'F-%02d'%nearest.id if nearest else 'NONE',f'{measured:.1f} m' if measured is not None else '--'))
        v['classifier'].setText('SPECIES FILTER\n%s\n%s' % (nearest_label,'HABITAT CLEAR' if nearest and s.safe_zone(nearest.pos) else 'EXCLUSION / NO LOCK'))
        v['depth'].setText('ROBOT DEPTH  %.1f m\nCONTACT DEPTH  %s' % (32-s.robot.z, f'{32-nearest.pos.z:.1f} m' if nearest else '--'))
        v['mechanism'].setText('CONTAINMENT  %s\nENGAGEMENT ≤ 3.0 m | BLOOD FX %s' % ('DEPLOYED' if s.state==State.CAPTURING else 'STANDBY','ON' if self.show_blood else 'OFF'))
        v['diag'].setText(f'3D NAV: {len(s.path)} waypoints | A* calls: {s.planner.calls} | {s.velocity.length():.1f} m/s | {s.energy_propulsion:.1f} Wh propulsion' if self.show_diagnostics else '')


if __name__ == "__main__":
    AquaMindApp().run()
