import datetime,hashlib,json,os,shutil,subprocess
from pathlib import Path
root=Path('/home/hp/models');stage=root/'yellow_diagonal_update_20261011'
expected={'yellow_runtime.py':'a0d214b7cc8b69e185000db3282bd6a811a84ce1dfbef9a2985608ce45cea86b','geometry_features.py':'3aef1ea31cdba2135673a8cd3158c0e8ba1a695a41fbb080d9238e8c026df094','yellow_model.json':'37d7aa7c572781e2fadbd30d7b76f15c9a6745805716ec4f86a33fa0c2148d6f'}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for name,value in expected.items():
    if digest(root/name)!=value:raise RuntimeError('Unexpected installed version: '+name)
backup=root/('yellow_diagonal_backup_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));backup.mkdir()
manifest=dict(backup=str(backup),files={})
for name in expected:
    shutil.copy2(str(root/name),str(backup/name))
    tmp=root/(name+'.diagonal_install_tmp');shutil.copy2(str(stage/name),str(tmp));os.replace(str(tmp),str(root/name))
    assert digest(root/name)==digest(stage/name)
    manifest['files'][name]=digest(root/name)
shutil.copy2(str(root/'realtime_camera_trt_distance_uart_court_line.py'),str(backup/'realtime_camera_trt_distance_uart_court_line.py'))
restore='#!/bin/sh\nset -eu\n'
for name in expected:restore+="cp -p '%s/%s' '/home/hp/models/%s'\n"%(backup,name,name)
(backup/'restore.sh').write_text(restore);os.chmod(str(backup/'restore.sh'),0o700)
(stage/'deployment_manifest.json').write_text(json.dumps(manifest,indent=2));(backup/'deployment_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
