"""Run on Nano with python3. Adjust only the live entrypoint's EDGE threshold."""
from pathlib import Path
import datetime
import re
import shutil
import ast

target = Path('/home/hp/models/realtime_camera_trt_distance_uart_court_line.py')
source = target.read_text(encoding='utf-8')
pattern = r'(?m)^(COURT_EDGE_Y\s*=\s*)([0-9.]+)(\s*(?:#.*)?)$'
matches = list(re.finditer(pattern, source))
if len(matches) != 1:
    raise RuntimeError('Expected exactly one COURT_EDGE_Y setting; no files changed')
old = float(matches[0].group(2))
if old not in (0.48, 0.65):
    raise RuntimeError('Unexpected threshold %s; no files changed' % old)
if old == 0.65:
    print('COURT_EDGE_Y already 0.65; restart the camera program to use it.')
else:
    updated = re.sub(pattern, lambda m: m.group(1) + '0.65' + m.group(3), source)
    ast.parse(updated)
    backup = target.parent / ('yellow_turn_backup_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir()
    shutil.copy2(str(target), str(backup / target.name))
    temporary = target.with_name(target.name + '.turn_update_tmp')
    shutil.copy2(str(target), str(temporary))
    temporary.write_text(updated, encoding='utf-8')
    temporary.replace(target)
    print('COURT_EDGE_Y: 0.48 -> 0.65')
    print('Backup: ' + str(backup / target.name))
    print('Restart the camera program to apply. UART format and YOLO unchanged.')
