"""Tests logic without importing Panda3D: load only definitions above renderer imports."""
import ast
import pathlib
import unittest
import sys
import types

SOURCE=pathlib.Path(__file__).resolve().parents[1]/'AquaMind_FINAL.py'
source=SOURCE.read_text()
tree=ast.parse(source)
# Stop at the explicit Panda3D renderer imports.
core=[]
for node in tree.body:
    if isinstance(node,ast.ImportFrom) and node.module=='panda3d.core':
        break
    core.append(node)
module=ast.Module(body=core,type_ignores=[])
module_obj=types.ModuleType('aquamind_core')
sys.modules['aquamind_core']=module_obj
ns=module_obj.__dict__
exec(compile(module,str(SOURCE),'exec'),ns)
Config=ns['Config'];Simulation=ns['Simulation'];Fish=ns['Fish'];Vec=ns['Vec'];State=ns['State'];Obstacle=ns['Obstacle'];Planner=ns['Planner']

class TestAquaMind(unittest.TestCase):
    def test_initial_counts(self):
        s=Simulation();self.assertEqual(len(s.active_lions()),s.cfg.lionfish)
        self.assertEqual(sum(f.native for f in s.fish),s.cfg.native_fish)
    def test_native_cannot_capture(self):
        s=Simulation();f=next(f for f in s.fish if f.native)
        s.robot=f.pos;self.assertFalse(s.can_capture(f));self.assertFalse(s.eligible(f))
    def test_capture_capacity(self):
        s=Simulation();f=next(f for f in s.fish if not f.native)
        s.robot=f.pos;s.onboard=8;self.assertFalse(s.can_capture(f))
    def test_reset(self):
        s=Simulation();s.onboard=5;s.battery=12;s.reset()
        self.assertEqual(s.onboard,0);self.assertEqual(s.battery,s.cfg.battery_wh)
    def test_battery_bounds(self):
        s=Simulation();s.start()
        for _ in range(300):s.update(.1)
        self.assertGreaterEqual(s.battery,0);self.assertLessEqual(s.battery,s.cfg.battery_wh)
    def test_planner_obstacles(self):
        p=Planner([Obstacle(0,0,6)],cell=4,limit=20)
        route=p.plan(Vec(-20,0,10),Vec(20,0,10))
        self.assertTrue(route)
        self.assertTrue(all(not p.blocked(q) for q in route))
    def test_pause_and_resume(self):
        s=Simulation();s.start();s.paused=True;t=s.elapsed;s.update(2)
        self.assertEqual(t,s.elapsed);s.paused=False;s.update(.1)
        self.assertGreater(s.elapsed,t)
    def test_whole_mission(self):
        s=Simulation()
        s.start()
        states={s.state}
        for _ in range(40000):
            s.update(.1)
            states.add(s.state)
            if s.total_captured>=8 and s.recharge_cycles>=1:break
        self.assertGreaterEqual(s.total_captured,8)
        self.assertGreaterEqual(s.recharge_cycles,1)
        self.assertIn(State.CAPTURING,states)
        self.assertIn(State.RETURNING_TO_BASE,states)
        self.assertLessEqual(s.onboard,8)

if __name__=='__main__':unittest.main()
