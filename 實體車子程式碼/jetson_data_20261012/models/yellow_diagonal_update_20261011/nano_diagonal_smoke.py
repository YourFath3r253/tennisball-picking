import json,time,sys
from pathlib import Path
import cv2,numpy as np
from yellow_runtime import YellowRuntime
root=Path(__file__).resolve().parent;results={}
if '--single-thread' in sys.argv:cv2.setNumThreads(1)
for name in ['yellow_sample','white_sample','diagonal_sample','steep_sample']:
    frame=cv2.imread(str(root/(name+'.jpg')))
    if frame is None:raise RuntimeError('Missing '+name)
    detector=YellowRuntime(root/'yellow_model.json');mask=None
    if name in ['diagonal_sample','steep_sample']:
        mask=np.zeros(frame.shape[:2],np.uint8);mask[:52,:]=255;mask[306:370,:345]=255
    detector.observe(frame,mask)
    timings=[]
    for _ in range(6):
        start=time.time();result=detector.observe(frame,mask);timings.append((time.time()-start)*1000)
    results[name]=dict(packet=result.packet(),position=result.position_state,line=result.line,median_processing_ms=float(np.median(timings)))
    if name=='white_sample':assert result.observation.state=='SAFE'
    else:assert result.observation.state=='EDGE',(name,result.packet())
print(json.dumps(results,indent=2))
(root/'nano_diagonal_smoke.json').write_text(json.dumps(results,indent=2))
