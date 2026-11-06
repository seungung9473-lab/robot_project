"""Exercise the actual move() branch without importing its actuator entrypoint."""
import ast
from pathlib import Path
import unittest
import numpy as np


class ReachedPlanning(Exception):
    pass


class ContinuousRetreatTests(unittest.TestCase):
    def exercise(self, label, holding=False):
        source=Path(__file__).resolve().parents[1]/'verified_place_left.py'
        fn=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='move')
        q0=np.zeros(4);q1=np.array([0.,0.,.03,0.]);calls=[]
        def fk(q):
            T=np.eye(4);T[:3,3]=np.asarray(q)[:3];return T
        def phase(label):
            calls.append(label);raise ReachedPlanning()
        ns={'np':np,'holding':holding,'fresh':lambda:q0.copy(),'fk':fk,'ik':lambda p,*a,**kw:np.r_[p,0.], 'phase':phase}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),ns)
        with self.assertRaises(ReachedPlanning):ns['move'](q1,label,True)
        return calls

    def test_released_empty_cup_has_one_unsplit_segment(self):
        self.assertEqual(self.exercise('empty_retreat_from_pad_ID5'),['empty_retreat_from_pad_ID5'])

    def test_carrying_object_keeps_existing_segmented_behavior(self):
        self.assertIn('_of_',self.exercise('empty_retreat_from_pad_ID5',holding=True)[0])

    def test_lowering_is_not_accidentally_changed(self):
        self.assertIn('_of_',self.exercise('lower_payload_to_pad_ID5')[0])

    def test_retreat_does_not_enable_pump_trigger(self):
        source=Path(__file__).resolve().parents[1]/'verified_place_left.py'
        fn=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='move')
        pump_if=next(n for n in ast.walk(fn) if isinstance(n,ast.If) and any(
            isinstance(c,ast.Constant) and c.value=='/vacuum/pump_on' for c in ast.walk(n)))
        # Actual pre-suction remains gated only by a contact target.
        self.assertEqual(ast.unparse(pump_if.test),'pump_target is not None')


if __name__=='__main__':unittest.main()
