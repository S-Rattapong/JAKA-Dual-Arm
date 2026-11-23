"""Opt-in per-run receive-clock evidence. Caller serializes all mutations."""
from collections import deque
from copy import deepcopy
from threading import Event
from math import degrees, isfinite


def driver_dispatch_summary(feedback):
    """Copy only the normalized terminal driver diagnostics for this execution."""
    arms = {}
    for side in ('left', 'right'):
        driver = feedback.get('drivers', {}).get(side) or {}
        groups = {}
        if (driver.get('valid') is not False and driver.get('available') is not False
                and driver.get('trajectory_id', feedback['trajectory_id']) == feedback['trajectory_id']):
            for key in ('dispatch_timing', 'servo_j_call_duration', 'servo_stream'):
                if isinstance(driver.get(key), dict) and driver[key]:
                    groups[key] = deepcopy(driver[key])
        status = 'AVAILABLE' if len(groups) == 3 else ('PARTIAL' if groups else 'UNAVAILABLE')
        arms[side] = dict(status=status, **groups)
    return dict(source='Authoritative terminal Phase5 host-driver feedback',
                semantics='Per-arm aggregates and stream boundaries on the host driver. '
                          'No controller acquisition timestamps or individual per-tick trace.',
                arms=arms)


class EndpointCapture:
    defaults = dict(pre_ms=1000, post_ms=1000, timeout_ms=5000, angle_deg=.05,
                    speed_deg_s=.5, hold_ms=100, max_gap_ms=250)

    def __init__(self, options, goals):
        if not isinstance(options, dict) or options.keys() - self.defaults.keys():
            raise ValueError("Invalid endpoint_evidence options")
        self.policy = self.defaults | options
        if any(type(v) not in (int, float) or not isfinite(v) or v <= 0 for v in self.policy.values()):
            raise ValueError("Endpoint settings must be finite positive numbers")
        if min(self.policy['pre_ms'], self.policy['post_ms']) < 1000 or self.policy['timeout_ms'] < self.policy['post_ms']:
            raise ValueError("Endpoint windows require >= 1000 ms and timeout >= post")
        self.wake_finalizer = Event()
        self.goals = goals
        self.rows = {side: deque() for side in goals}
        self.quiet_since = dict.fromkeys(goals)
        self.gap_stats = {side: dict(count=0, sum_ns=0, max_ns=None, nonincreasing=0) for side in goals}
        self.invalid_clock = dict.fromkeys(goals, 0)
        self.bound = self.terminal = self.settled_ns = None

    def trim(self, rows, now):
        cutoff = now - self.policy['pre_ms'] * 1e6
        # Keep one predecessor to bracket the beginning of the baseline.
        while len(rows) > 1 and rows[1]['received_monotonic_ns'] <= cutoff:
            rows.popleft()

    def bind(self, now):
        self.bound = now
        for rows in self.rows.values():
            self.trim(rows, now)

    def append(self, packet, delivered, appended):
        side = packet['side']
        stamp = packet['received_monotonic_ns']
        if self.settled_ns is not None or (self.terminal is not None and appended > self.deadline):
            return
        if type(stamp) is not int or stamp < 0 or stamp > delivered:
            self.invalid_clock[side] += 1
            return
        rows = self.rows[side]
        prev = rows[-1] if rows else None
        if prev:
            gap = stamp - prev['received_monotonic_ns']
            stats = self.gap_stats[side]
            if gap <= 0:
                stats['nonincreasing'] += 1
            else:
                stats['count'] += 1
                stats['sum_ns'] += gap
                stats['max_ns'] = max(stats['max_ns'] or 0, gap)
        rows.append(dict(packet, recorder_delivery_ns=delivered, recorder_append_ns=appended))
        if self.bound is None:
            self.trim(rows, stamp)
        if self.terminal is None or stamp < self.terminal:
            return
        dt = (stamp - prev['received_monotonic_ns']) / 1e6 if prev else 0
        quiet = False
        if prev and prev['received_monotonic_ns'] >= self.terminal and 0 < dt <= self.policy['max_gap_ms']:
            pos = packet['actual_joints_rad']
            quiet = all(abs(degrees(x-g)) <= self.policy['angle_deg'] and
                        abs(degrees(x-y))*1000/dt <= self.policy['speed_deg_s']
                        for x, g, y in zip(pos, self.goals[side], prev['actual_joints_rad']))
        if not quiet:
            self.quiet_since[side] = None
        elif self.quiet_since[side] is None:
            self.quiet_since[side] = stamp
        if all(self.qualified(arm, appended) for arm in self.rows):
            self.settled_ns = appended
            self.wake_finalizer.set()

    @property
    def deadline(self):
        return self.terminal + int(self.policy['timeout_ms'] * 1e6)

    def qualified(self, side, now):
        since = self.quiet_since[side]
        if since is None:
            return False
        last = self.rows[side][-1]['received_monotonic_ns']
        return (last - since >= self.policy['hold_ms'] * 1e6 and
                last - self.terminal >= self.policy['post_ms'] * 1e6 and
                min(now, self.deadline) - last <= self.policy['max_gap_ms'] * 1e6)

    def outcome(self, now):
        if self.settled_ns is not None:
            return 'SETTLED'
        return 'TIMEOUT' if now >= self.deadline else None

    def snapshot(self, outcome, now):
        evaluated_at = self.settled_ns if self.settled_ns is not None else now
        return dict(schema_version='exp2-host-endpoints/v1', policy=self.policy,
                    bind_observed_ns=self.bound, terminal_observed_ns=self.terminal,
                    outcome=outcome, finalized_ns=now, settled_observed_ns=self.settled_ns,
                    tail_qualified={side: self.qualified(side, evaluated_at) for side in self.rows},
                    receive_gaps_since_arm=self.gap_stats, invalid_receive_clock=self.invalid_clock,
                    packets={s: list(rows) for s, rows in self.rows.items()})
