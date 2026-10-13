"""Backup and atomically install the authorized yellow boundary replacement."""
import datetime,hashlib,json,os,shutil
from pathlib import Path
root=Path('/home/hp/models');stage=root/'yellow_update_20261011'
main=root/'realtime_camera_trt_distance_uart_court_line.py'
expected='ece6d5ca60f1937bbe9a0aa1eb4ccca32d72ea4df94ab6d07e4d48319b16f739'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
if digest(main)!=expected:raise RuntimeError('Active source changed since inspection; refusing overwrite')
backup=root/('yellow_backup_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
backup.mkdir()
preserved=[]
for name in ['realtime_camera_trt_distance_uart_court_line.py','court_line_runtime.py','color_models_v2.json','yellow_runtime.py','geometry_features.py','yellow_model.json']:
    path=root/name
    if path.exists():shutil.copy2(str(path),str(backup/name));preserved.append(name)
mapping={'yellow_runtime.py':'yellow_runtime.py','geometry_features.py':'geometry_features.py','yellow_model.json':'yellow_model.json','realtime_yellow_ball_uart.py':'realtime_camera_trt_distance_uart_court_line.py'}
manifest=dict(backup=str(backup),preserved=preserved,files={})
for source,dest in mapping.items():
    tmp=root/(dest+'.yellow_install_tmp')
    shutil.copy2(str(stage/source),str(tmp));os.replace(str(tmp),str(root/dest))
    assert digest(stage/source)==digest(root/dest)
    manifest['files'][dest]=digest(root/dest)
restore='#!/bin/sh\nset -eu\ncd /home/hp/models\n'
for name in preserved:
    restore+="cp -p '%s/%s' '%s'\n"%(str(backup),name,name)
(backup/'restore.sh').write_text(restore);os.chmod(str(backup/'restore.sh'),0o700)
(backup/'deployment_manifest.json').write_text(json.dumps(manifest,indent=2))
(stage/'deployment_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
