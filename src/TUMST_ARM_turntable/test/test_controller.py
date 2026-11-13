import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from soarm_turntable.controller import Controller,target_count
class FakeBus:
    def __init__(self):
        self.mem=bytearray(200);self.writes=[]
        for addr,size,value in [(0,2,1200),(6,1,53),(11,1,3),(44,4,445),(48,4,4095),(108,4,10),(112,4,50),(132,4,2047),(144,2,50)]:self.mem[addr:addr+size]=value.to_bytes(size,'little')
    def read_block(self,a,n):return bytes(self.mem[a:a+n])
    def read(self,a,n):return int.from_bytes(self.read_block(a,n),'little')
    def write(self,a,n,v,tolerance=0):self.writes.append((a,n,v));self.mem[a:a+n]=v.to_bytes(n,'little');
    def close(self):pass
class Tests(unittest.TestCase):
    def test_readonly_no_writes(self):
        b=FakeBus();c=Controller(b);self.assertEqual(c.snapshot()['position_count'],2047)
        for fn in [lambda:c.goto(5,2047),lambda:c.torque(True),c.stop]:
            with self.assertRaises(RuntimeError):fn()
        self.assertEqual(b.writes,[])
    def test_explicit_control(self):
        b=FakeBus();c=Controller(b,False);c.torque(True);self.assertEqual(b.writes[:2],[(116,4,2047),(64,1,1)])
        target=c.goto(5,2047);self.assertEqual(target,2104);self.assertEqual(b.writes[-3:],[(108,4,10),(112,4,50),(116,4,2104)])
        before=list(b.writes)
        with self.assertRaises(ValueError):c.goto(360,2047)
        self.assertEqual(before,b.writes)
        c.stop();self.assertEqual(b.writes[-1],(116,4,2047))
    def test_wrong_mode_and_fault(self):
        for addr,value in [(11,4),(10,4),(70,1)]:
            b=FakeBus();b.mem[addr]=value;c=Controller(b,False)
            with self.assertRaises(RuntimeError):c.goto(5,2047)
            self.assertEqual(b.writes,[])
    def test_invalid_angles(self):
        for q in [float('nan'),float('inf'),True]:
            with self.assertRaises(ValueError):target_count(q,2047,0,4095)
if __name__=='__main__':unittest.main()
