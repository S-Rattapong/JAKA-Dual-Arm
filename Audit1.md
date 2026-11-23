You are working in:

~/jaka\_ws

Branch:

feature/coupled-ik-v1

TASK:

PHASE-3 IK BRANCH SWITCH / GLOBAL GRAPH SEARCH AUDIT

SOURCE + OFFLINE TEST AUDIT ONLY.

DO NOT modify production code.

DO NOT connect physical robots.

DO NOT launch JAKA driver.

DO NOT send motion.

\============================================================

OBSERVED PROBLEM

\============================================================

During Planned Ghost preview:

\- motion toward W1 keeps a normal/expected arm configuration

\- during motion from W1 toward W2, the arm configuration suddenly

changes to a radically different IK branch

\- the Center/Cartesian waypoint itself only continues moving normally

\- visually, a simple continuation of the previous arm configuration

appears likely to be possible

This looks like an unnecessary IK branch switch.

Do NOT assume the IK solver itself is wrong.

The current planner uses multi-seed IK candidates plus the exact layered

DAG / Global Graph Search.

We need to determine WHY the selected path changes branch.

\============================================================

1\. TRACE CURRENT GLOBAL COST

\============================================================

Inspect the current Phase-3 production planner and report exactly how

the edge/path cost between consecutive IK candidates is calculated.

Identify:

\- joint-space transition cost formula

\- per-joint weights, if any

\- whether cost is sum, L1, L2, squared L2, max delta, etc.

\- whether Left and Right arms are weighted separately

\- any start-state cost

\- any terminal/future cost

\- any collision penalty

\- any singularity/manipulability term

\- any branch-consistency term

\- any joint-center/limit penalty

Give exact files/functions/lines.

\============================================================

2\. CHECK ANGLE WRAPPING

\============================================================

For every revolute joint, determine whether candidate-to-candidate

difference uses:

raw:

abs(q2 - q1)

or shortest angular distance:

abs(wrapToPi(q2 - q1))

or another normalization.

Check whether IK candidate normalization can represent equivalent joint

angles differing by approximately 2\*pi.

Determine whether +pi/-pi or +/-2\*pi aliasing can distort Global Graph

Search cost.

Do NOT change normalization yet.

\============================================================

3\. CHECK IK CANDIDATE GENERATION

\============================================================

Trace multi-seed candidate generation for each trajectory sample.

Report:

\- number of attempts per arm

\- deduplication tolerance

\- candidate normalization

\- candidate sorting/order

\- whether previous selected solution is used as a seed

\- whether the planning start state is used as a seed

\- whether candidates preserve an implicit IK branch identity

\- whether candidate ordering can affect tie-breaking

\============================================================

4\. CHECK W1 -> W2 BRANCH CONTINUITY

\============================================================

Using the current offline planning configuration/waypoints if available,

inspect the samples around the W1 -> W2 transition.

For several samples immediately:

\- before the visible branch change

\- at the branch change

\- after the branch change

report for Left and Right:

\- sample index

\- Cartesian target

\- selected IK candidate index

\- selected 6-joint vector

\- previous selected 6-joint vector

\- per-joint delta

\- total edge cost

Also identify alternative IK candidates at the branch-change sample.

For each relevant alternative report:

\- candidate index

\- joint vector

\- edge cost from previous selected candidate

\- validity/pruning reason

\- whether it continues the previous visual branch

\============================================================

5\. DETERMINE WHETHER BRANCH CHANGE IS FORCED

\============================================================

Classify the branch switch as exactly one of:

A. FORCED

Previous branch has no valid candidate because of:

\- IK failure

\- joint limit

\- self collision

\- inter-arm collision

\- sampled path collision

\- other explicit pruning

B. OPTIONAL BUT CHEAPER

Previous branch remains valid, but Global Graph Search chooses another

branch because current objective gives it a lower total path cost.

C. COST / ANGLE REPRESENTATION BUG

Equivalent joint values / wrapping / normalization cause incorrect edge

cost.

D. TIE-BREAK / CANDIDATE ORDER EFFECT

Several equal/nearly equal paths exist and deterministic ordering picks

the visually discontinuous branch.

E. OTHER

Provide evidence.

\============================================================

6\. GLOBAL VS GREEDY

\============================================================

Compare the selected Global Graph path with the existing Greedy path

around the same W1 -> W2 region.

Report:

\- whether Greedy stays on the previous branch

\- whether Global changes branch

\- cumulative cost of each

\- exact point where paths diverge

\- whether the Global improvement is numerically meaningful or tiny

A tiny global-cost improvement must be called out explicitly if it

causes a huge visual/configuration change.

\============================================================

7\. JOINT-JUMP RELATION

\============================================================

Inspect current Phase-4 joint continuity / joint jump thresholds.

Without changing Phase-4:

determine whether this planned branch change should currently be

detected by:

\- joint\_continuity

\- joint\_jump

Report the maximum consecutive joint delta around the branch change.

If Phase-4 would PASS this trajectory despite the visually radical

configuration change, explain why.

\============================================================

8\. START-STATE / BRANCH ANCHORING

\============================================================

Determine how strongly the first planning layer is anchored to:

planning\_start\_state\_rad

Then determine whether there is any concept of maintaining branch

continuity after the first layer.

Specifically check whether the graph objective only minimizes numerical

joint distance and has no explicit preference for staying in the same

IK family/configuration.

\============================================================

9\. DO NOT PATCH YET

\============================================================

Do NOT:

\- increase joint-jump thresholds

\- remove IK candidates

\- hard-code elbow-up/elbow-down

\- force one candidate index

\- reduce multi-seed IK

\- replace Global Graph Search with Greedy

\- change collision behavior

\- change Phase-4

\- add arbitrary penalties

First identify the exact mechanism.

\============================================================

10\. RECOMMEND SMALLEST CORRECT FIX

\============================================================

After identifying the root cause, recommend the smallest robust fix.

Possible categories may include, only if supported by evidence:

\- shortest-angular-distance edge metric

\- consistent candidate normalization

\- explicit branch-switch penalty

\- continuity-aware tie breaking

\- previous-branch preference within a small cost epsilon

\- seed continuity improvement

\- graph-state enrichment

Do NOT implement it in this audit.

The desired behavior is:

If two IK paths achieve the same Cartesian trajectory and remain valid,

prefer a smooth continuation of the current robot configuration rather

than an unnecessary large configuration/branch change.

But a branch switch must still remain possible if the current branch

becomes infeasible.

\============================================================

SAFETY

\============================================================

Offline only.

No:

\- /api/program/run

\- Jog

\- Home

\- joint\_move

\- linear\_move

\- real robot connection

Preserve dirty worktree.

Do not stage/commit/merge/push.

\============================================================

DELIVERABLE

\============================================================

Report:

1\. Global graph edge-cost formula

2\. Angle wrapping behavior

3\. IK candidate generation/dedup behavior

4\. Start-state anchoring

5\. Selected path around W1 -> W2

6\. Alternative candidates around branch change

7\. Exact branch-change sample

8\. Per-joint delta at branch change

9\. Whether old branch remains feasible

10\. Forced vs optional classification

11\. Any collision/limit pruning involved

12\. Global vs Greedy comparison

13\. Whether Global cost advantage is meaningful

14\. Phase-4 joint-jump expectation

15\. Exact root cause

16\. Smallest recommended fix

17\. Files/functions a later patch would touch

18\. Confirmation no production changes/no robot/no motion

End exactly:

PHASE3\_IK\_BRANCH\_SWITCH\_AUDIT\_READY

or

PHASE3\_IK\_BRANCH\_SWITCH\_AUDIT\_BLOCKED: