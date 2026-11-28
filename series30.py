import json, glob, sys, time
B='/home/newtonrattapong/jaka_ws/dual_arm_app'
sys.path.insert(0, B + '/backend')
from experimental_controller_timebase import analyze_controller_timebase
URDF = B + '/web/assets/dual_jaka_a12_web.urdf'
out = {}
for d in sorted(glob.glob(B + '/experiment_runs/exp-20261123T0[45]*')):
    m = json.load(open(d + '/manifest.json'))
    if not m.get('label', '').startswith('Real Exp'):
        continue
    raw = json.load(open(d + '/raw.json'))
    r = analyze_controller_timebase(raw, m, URDF)
    out[m['label']] = {'run_id': m['run_id'],
        'methods': {k: {'rms': v['translation_rms_mm'], 'max': v['translation_max_mm'],
                        's': v['samples_t_mm_deg']} for k, v in r['methods'].items()}}
    print(m['label'], r['status'], flush=True)
json.dump(out, open('/tmp/series30.json', 'w'), separators=(',', ':'))
print('done', len(out))
