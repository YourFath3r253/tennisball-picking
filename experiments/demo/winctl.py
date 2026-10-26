"""錄影前整理視窗 (X11 EWMH，沒有 wmctrl/xdotool 時用)。
用法: python3 experiments/demo/winctl.py list
      python3 experiments/demo/winctl.py minimize <名稱片段>   (例如 Claude)
      python3 experiments/demo/winctl.py raise <名稱片段>
"""
import sys

from Xlib import X, display, protocol

d = display.Display()
root = d.screen().root
NET_CLIENT_LIST = d.intern_atom('_NET_CLIENT_LIST')
NET_WM_NAME = d.intern_atom('_NET_WM_NAME')
UTF8 = d.intern_atom('UTF8_STRING')
NET_ACTIVE = d.intern_atom('_NET_ACTIVE_WINDOW')
WM_CHANGE_STATE = d.intern_atom('WM_CHANGE_STATE')


def windows():
    ids = root.get_full_property(NET_CLIENT_LIST, X.AnyPropertyType).value
    out = []
    for wid in ids:
        w = d.create_resource_object('window', wid)
        p = w.get_full_property(NET_WM_NAME, UTF8)
        name = p.value.decode('utf8', 'replace') if p else (w.get_wm_name() or '')
        cls = w.get_wm_class() or ('', '')
        out.append((w, name, cls[1]))
    return out


def send(w, atom, data):
    ev = protocol.event.ClientMessage(window=w, client_type=atom, data=(32, data + [0] * (5 - len(data))))
    root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)


cmd = sys.argv[1]
key = sys.argv[2].lower() if len(sys.argv) > 2 else ''
for w, name, cls in windows():
    hit = key and (key in name.lower() or key in cls.lower())
    if cmd == 'list':
        g = w.get_geometry()
        print(f'{w.id:#x} [{cls}] {name}  {g.width}x{g.height}')
    elif cmd == 'minimize' and hit:
        send(w, WM_CHANGE_STATE, [3])          # IconicState
        print('minimized', name)
    elif cmd == 'raise' and hit:
        send(w, NET_ACTIVE, [2, X.CurrentTime, 0])
        print('raised', name)
d.flush()
