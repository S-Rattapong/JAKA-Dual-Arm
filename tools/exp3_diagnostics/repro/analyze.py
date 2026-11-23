"""Analyze harness output: receive gaps inside the work window vs outside, bursts,
L/R coincidence, and receive latency vs the fake controller's send time."""
import json, sys, statistics as st
for mode in sys.argv[1:]:
    o = json.load(open(f"out_{mode}.json"))
    ph = o["phase"]; w0, w1 = ph["work_start_ns"], ph["work_end_ns"]
    sends = {"left": {}, "right": {}}; accepts = []
    for line in open(f"send_{mode}.txt"):
        a = line.split()
        if a[0] == "accept": accepts.append(a[1]); continue
        sends[a[0]][int(a[1])] = int(a[2])
    print(f"== {mode}: work {ph['work_s']:.2f}s rows={ph.get('rows')} accepts={accepts} errors={o['errors'][:2]}")
    starts = {}
    for side in ("left", "right"):
        R = o["recv"][side]
        t = [x[0] for x in R]; idx = [x[1] for x in R]
        lost = sum(1 for i in range(len(idx) - 1) if idx[i + 1] - idx[i] != 1)
        lat = [(x[0] - sends[side][x[1]]) / 1e6 for x in R if x[1] in sends[side]]
        def stats(sel):
            g = [(t[i + 1] - t[i]) / 1e6 for i in range(len(t) - 1) if sel(t[i])]
            l = [lat[i] for i in range(len(lat)) if sel(t[i])]
            if not g: return "n/a"
            return (f"n={len(g)} max_gap={max(g):.0f}ms gaps>250={sum(1 for x in g if x > 250)} "
                    f"<5ms={sum(1 for x in g if x < 5)} median_int={st.median(g):.0f}ms "
                    f"latency med={st.median(l):.1f} max={max(l):.0f}ms")
        print(f"  {side} IN-WORK : {stats(lambda x: w0 <= x <= w1)}")
        print(f"  {side} OUTSIDE : {stats(lambda x: not (w0 <= x <= w1))}  seq-breaks={lost}")
        starts[side] = [t[i] for i in range(len(t) - 1) if (t[i + 1] - t[i]) / 1e6 > 250]
    L, Rr = starts["left"], starts["right"]
    if L:
        co = sum(1 for x in L if any(abs(x - y) <= 150e6 for y in Rr))
        print(f"  L gap starts matched to R within 150ms: {co}/{len(L)}")
