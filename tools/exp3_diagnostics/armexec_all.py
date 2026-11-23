"""Read-only: relate ARM time, execution start and Port10000 gaps for every saved run."""
import json, sys, os, datetime as D
base = sys.argv[1]
def ms(s): return D.datetime.fromisoformat(s.replace('Z', '+00:00')).timestamp() * 1000
print("run label arm->exec_s gaps>250(L) last_gap_rel_arm_s raw_MB")
for d in sorted(os.listdir(base)):
    p = os.path.join(base, d)
    if not d.startswith('exp-') or not os.path.isfile(os.path.join(p, 'raw.json')):
        continue
    try:
        m = json.load(open(os.path.join(p, 'manifest.json')))
        r = json.load(open(os.path.join(p, 'raw.json')))
    except Exception as e:
        print(d, 'ERR', e); continue
    arm = ms(m['created_at'])
    ex = r.get('execution') or {}
    st = ex.get('start_time_unix_ns')
    P = (r.get('actual_packets') or {}).get('left') or []
    w = [q['received_at_ms'] for q in P if 'received_at_ms' in q]
    g = [w[i] for i in range(len(w) - 1) if w[i + 1] - w[i] > 250]
    a2e = f"{(st / 1e6 - arm) / 1000:.1f}" if st else "NA"
    last = f"{(g[-1] - arm) / 1000:.1f}" if g else "-"
    print(d[4:19], d[-8:], (m.get('label') or '')[:22], a2e, len(g), last,
          f"{os.path.getsize(os.path.join(p, 'raw.json')) / 1e6:.1f}")
