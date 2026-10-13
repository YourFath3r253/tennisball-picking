"""State-transition checks that matter for the downstream boundary controller."""
import unittest
import numpy as np
from yellow_runtime import YellowRuntime

class StateTests(unittest.TestCase):
    def runtime(self):
        # Isolate control behavior from learned weights and image extraction.
        r=YellowRuntime.__new__(YellowRuntime)
        r.threshold=.8;r.edge_y=.48;r.edge_confirm_frames=2
        r.track=None;r.on_count=0;r.misses=0;r.near_count=0;r.edge_observation=None
        r.score=lambda f:float(f[0])
        return r
    def candidate(self,score=.9,line=(20,150,300,150)):
        return dict(feature=np.array([score]),line=line,angle=0,length_ratio=.875)
    def test_needs_two_fresh_observations(self):
        r=self.runtime();c=self.candidate()
        self.assertEqual(r.update_candidates([c]).observation.state,'SAFE')
        self.assertEqual(r.update_candidates([c]).packet(),'COURT,EDGE,CENTER,YELLOW')
    def test_rejects_background_and_does_not_accumulate_missing(self):
        r=self.runtime();c=self.candidate()
        r.update_candidates([c]);r.update_candidates([])
        self.assertEqual(r.update_candidates([c]).observation.state,'SAFE')
        self.assertEqual(r.update_candidates([self.candidate(.2)]).raw_label,'unknown')
    def test_bounded_hold_and_clear(self):
        r=self.runtime();c=self.candidate();r.update_candidates([c]);r.update_candidates([c])
        held=r.update_candidates([])
        self.assertEqual(held.position_state,'held');self.assertEqual(held.observation.state,'EDGE')
        self.assertEqual(r.update_candidates([]).packet(),'COURT,SAFE,NONE,UNKNOWN')
    def test_jump_cannot_complete_near_confirmation(self):
        r=self.runtime();r.update_candidates([self.candidate()])
        self.assertEqual(r.update_candidates([self.candidate(line=(20,220,300,220))]).observation.state,'SAFE')
    def test_yellow_evidence_precedes_geometry_score(self):
        r=self.runtime()
        result=r.update_candidates([self.candidate(.1),self.candidate(.95,line=(20,190,300,190))])
        self.assertAlmostEqual(result.raw_confidence,.95)

if __name__=='__main__':unittest.main()
