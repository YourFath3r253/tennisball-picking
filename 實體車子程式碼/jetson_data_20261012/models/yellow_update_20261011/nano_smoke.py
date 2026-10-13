"""Offline GPU and boundary smoke test; never opens camera or UART."""
import json,os,time
from pathlib import Path
import cv2
import numpy as np
import realtime_yellow_ball_uart as app
from yellow_runtime import YellowRuntime

root=Path(__file__).resolve().parent
model=root/'yellow_model.json'
results={}
for name in ['yellow','white']:
    frame=cv2.imread(str(root/(name+'_sample.jpg')))
    if frame is None:raise RuntimeError('Missing '+name+' sample')
    detector=YellowRuntime(model)
    detector.observe(frame);result=detector.observe(frame)
    results[name]=dict(packet=result.packet(),score=result.raw_confidence)
assert results['yellow']['packet'].startswith('COURT,EDGE,'),results
assert results['white']['packet']=='COURT,SAFE,NONE,UNKNOWN',results
frame=cv2.imread('/home/hp/raw_640x480.jpg')
if frame is None:frame=cv2.imread(str(root/'yellow_sample.jpg'))
engine=app.load_engine('/home/hp/models/'+app.ENGINE_PATH)
if engine is None:raise RuntimeError('TensorRT engine failed to load')
context=engine.create_execution_context()
inputs,outputs,bindings,stream=app.allocate_buffers(engine)
image,_=app.preprocess(frame)
start=time.time();output=app.infer(context,bindings,inputs,outputs,stream,image)
detections=app.parse_yolo_output(output)
results['yolo']=dict(engine_loaded=True,output_shapes=[list(o.shape) for o in output],detections=len(detections),first_inference_ms=(time.time()-start)*1000)
results['hardware']=dict(camera_devices=list(map(str,Path('/dev').glob('video*'))),uart_devices=list(map(str,Path('/dev').glob('ttyACM*'))))
print(json.dumps(results,indent=2))
(root/'nano_smoke_result.json').write_text(json.dumps(results,indent=2))
