"""Yellow-only boundary runtime. No four-color ranking or color switching.

Classifier scores are model scores, not calibrated probabilities. SAFE means
no confirmed near yellow marker in this observation, not guaranteed free space.
"""
import json
import math
import cv2
import numpy as np
from geometry_features import GeometryFeatures, association_cost


class Observation(object):
    def __init__(self,state='SAFE',side='NONE',near_x=None,near_y=None,count=0):
        self.state=state;self.line_side=side
        self.color='YELLOW' if state=='EDGE' else 'UNKNOWN'
        self.near_x_norm=near_x;self.near_y_norm=near_y;self.near_confirm_count=count
    def packet(self):
        return 'COURT,%s,%s,%s'%(self.state,self.line_side,self.color)


class Result(object):
    def packet(self):return self.observation.packet()


class YellowRuntime(GeometryFeatures):
    def __init__(self,model_path,edge_y=.48,edge_confirm_frames=2,**kwargs):
        with open(str(model_path),'r') as f:self.model=json.load(f)
        if self.model.get('type')!='yellow_binary_logistic_v1':
            raise ValueError('Expected yellow_binary_logistic_v1 model')
        self.mean=np.asarray(self.model['mean'],dtype=np.float32)
        self.scale=np.asarray(self.model['scale'],dtype=np.float32)
        self.weights=np.asarray(self.model['weights'],dtype=np.float32)
        if any(x.shape!=(10,) for x in (self.mean,self.scale,self.weights)):
            raise ValueError('Model must have ten features')
        if not (np.all(np.isfinite(self.mean)) and np.all(np.isfinite(self.scale)) and np.all(self.scale>0) and np.all(np.isfinite(self.weights))):
            raise ValueError('Invalid model features')
        if not 0<edge_y<1 or edge_confirm_frames<1:raise ValueError('Invalid EDGE settings')
        self.threshold=float(self.model['threshold']);self.edge_y=edge_y
        self.edge_confirm_frames=edge_confirm_frames;self.color_model_loaded=True
        self.track=None;self.on_count=0;self.misses=0;self.near_count=0
        self.edge_observation=None

    def score(self,feature):
        z=np.clip((np.asarray(feature)-self.mean)/self.scale,-8,8)
        logit=float(np.dot(z,self.weights)+self.model['bias'])
        return 1/(1+math.exp(-max(-40,min(40,logit))))

    def update_candidates(self,candidates,width=320,height=240,display_scale=(1.,1.)):
        accepted=[]
        for c in candidates:
            score=self.score(c['feature'])
            if score>=self.threshold:accepted.append((score,c))
        selected=None;score=0.
        if self.track is not None:
            matches=[(association_cost(c,self.track),s,c) for s,c in accepted]
            matches=[m for m in matches if m[0] is not None]
            if matches:
                _,score,selected=min(matches,key=lambda m:(m[0],-m[1]))
        if selected is None and accepted:
            score,selected=max(accepted,key=lambda pair:pair[0])
        result=Result();observation=Observation()
        result.raw_label='yellow' if selected is not None else 'unknown'
        result.raw_confidence=score
        if selected is not None:
            same=self.track is not None and association_cost(selected,self.track) is not None
            self.on_count=self.on_count+1 if same else 1
            if not same:self.near_count=0;self.edge_observation=None
            self.track=selected;self.misses=0
            x1,y1,x2,y2=selected['line']
            nx=(x1 if y1>y2 else x2 if y2>y1 else (x1+x2)/2.)/width
            ny=max(y1,y2)/float(height)
            self.near_count=self.near_count+1 if ny>=self.edge_y else 0
            if self.on_count>=2 and self.near_count>=self.edge_confirm_frames:
                side='LEFT' if nx<.4 else 'RIGHT' if nx>.6 else 'CENTER'
                observation=Observation('EDGE',side,nx,ny,self.near_count)
                self.edge_observation=observation
            else:
                observation=Observation(near_x=nx,near_y=ny,count=self.near_count)
                self.edge_observation=None
            result.position_state='observed'
            result.color_status='confirmed' if self.on_count>=2 else 'confirming'
        else:
            self.misses+=1;self.near_count=0;self.on_count=0
            if self.track is not None and self.misses<2:
                result.position_state='held';result.color_status='position_held'
                if self.edge_observation is not None:
                    observation=Observation('EDGE',self.edge_observation.line_side)
            else:
                self.track=None;self.edge_observation=None
                result.position_state='unavailable';result.color_status='no_line'
        result.observation=observation;result.color_model_loaded=True
        result.color_label='yellow' if self.track is not None else 'unknown'
        result.color_confidence=score;result.line=None
        if self.track is not None:
            result.line=tuple(int(round(v*display_scale[i%2])) for i,v in enumerate(self.track['line']))
        return result

    def observe(self,frame):
        h,w=frame.shape[:2]
        small=cv2.resize(frame,(self.PROCESS_W,self.PROCESS_H),interpolation=cv2.INTER_AREA)
        return self.update_candidates(self._find_candidate_lines(small),self.PROCESS_W,self.PROCESS_H,(w/float(self.PROCESS_W),h/float(self.PROCESS_H)))
