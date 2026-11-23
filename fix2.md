You are working in:

~/jaka\_ws

Branch:

feature/coupled-ik-v1

TASK:

PHASE-3 IK BRANCH-SWITCH DIAGNOSTIC CAPTURE

This checkpoint adds OFFLINE DIAGNOSTIC INSTRUMENTATION ONLY.

DO NOT change planner behavior.

DO NOT fix the branch switch yet.

DO NOT change IK seeds.

DO NOT change graph costs.

DO NOT change candidate ordering.

DO NOT change angle normalization.

DO NOT change Phase-4.

\============================================================

CONTEXT

\============================================================

A real browser preview reproduced a visually radical IK branch switch

during motion from W1 toward W2.

Source audit confirmed:

1\. Global edge cost is:

sum((q\_to - q\_from)^2)

over raw 12-joint coordinates.

2\. Global scoring does NOT use shortest angular distance.

3\. Candidate deduplication also uses raw joint coordinates.

4\. Global layer 0 initializes every valid node with:

cost = 0

so planning\_start\_state is not directly included in the Global

objective.

5\. There is no explicit IK branch identity or branch-switch penalty.

6\. The actual event cannot yet be classified because the production

response does not retain:

\- complete candidate graph

\- rejected candidate-pair reasons

\- Greedy path

\- DP/backpointer information

\- alternative candidates around the branch-change sample

We MUST capture the exact observed planning event before changing the

objective.

\============================================================

DESIGN REQUIREMENT

\============================================================

Add an environment-gated diagnostic capture.

Recommended interface:

JAKA\_PHASE3\_IK\_DEBUG\_CAPTURE\_PATH=/tmp/jaka\_phase3\_ik\_branch\_capture.json

If the environment variable is UNSET:

\- production behavior must remain unchanged

\- production response contract should remain unchanged unless an

existing diagnostics field can safely remain identical

\- no file should be written

\- no extra planner decisions should occur

If it is SET:

\- the exact production planning request must execute normally

\- planner result must remain identical

\- additionally write one detailed JSON diagnostic artifact to the

specified local path

This is a developer/offline diagnostic mechanism only.

Do NOT add normal operator UI for this.

\============================================================

CAPTURE EXACT INPUT AUTHORITY

\============================================================

Record enough information to uniquely identify/reproduce the request:

\- diagnostic schema/version

\- object/center waypoints

\- sample/timeline information

\- center orientation

\- locked C\_T\_L

\- locked C\_T\_R

\- grasp content revision

\- grasp lock generation/revision

\- calibration revision

\- model calibration revision

\- planning\_start\_state\_rad

\- planning-start source

\- candidate attempts per arm

\- relevant planner settings

Do not dump unrelated application state.

\============================================================

CAPTURE EACH TRAJECTORY SAMPLE / GRAPH LAYER

\============================================================

For every planning sample/layer record:

\- layer index

\- sample/time index if available

\- Center target

\- Left target pose

\- Right target pose

For every VALID graph node record:

\- node index

\- Left candidate index

\- Right candidate index

\- Left candidate provenance/seed attempt

\- Right candidate provenance/seed attempt

\- Left 6-joint vector

\- Right 6-joint vector

\- combined 12-joint vector

Retain enough provenance to tell whether candidates came from different

IK seeds/branches.

\============================================================

CAPTURE REJECTED PAIRS

\============================================================

For each Left × Right candidate pair rejected before becoming a graph

node, capture the available validity reason.

At minimum distinguish when available:

\- state-validity rejection

\- joint-limit/invalid candidate rejection

\- missing IK candidate

\- other explicit rejection reason

Do not fabricate a reason when current APIs do not expose one.

Use UNKNOWN/UNAVAILABLE truthfully where necessary.

\============================================================

CAPTURE GRAPH SEARCH INFORMATION

\============================================================

Capture the exact Global Graph Search information required to explain

the selected path:

For each valid node where applicable:

\- incoming predecessor selected by DP

\- cumulative Global cost

\- local edge cost from predecessor

Capture the FINAL selected Global node index for every layer.

Also capture the complete selected Global 12-joint path.

\============================================================

CAPTURE GREEDY PATH

\============================================================

The current production code computes Greedy comparison data but does

not serialize its selected path.

For debug capture only, retain:

\- selected Greedy node index per layer

\- selected Greedy 12-joint path

\- local edge costs

\- cumulative Greedy cost

Do NOT change how Greedy is computed.

Do NOT use Greedy for production selection.

\============================================================

RAW + ANGULAR TRANSITION DIAGNOSTICS

\============================================================

For every consecutive transition in the selected Global path record:

\- raw per-joint delta

\- shortest-angular per-joint delta

\- raw squared-L2 edge cost

\- maximum absolute raw delta

\- maximum absolute shortest-angular delta

Do the same for the Greedy path.

This is DIAGNOSTIC only.

Do NOT change Global scoring to use shortest angular distance yet.

\============================================================

AUTOMATIC BRANCH-CHANGE CANDIDATE SUMMARY

\============================================================

For diagnostic convenience, identify suspicious selected transitions

using diagnostic-only criteria.

Do NOT change planner validity.

For each suspicious transition capture:

\- from layer

\- to layer

\- selected from-node

\- selected to-node

\- selected joint vectors

\- raw deltas

\- shortest-angular deltas

\- selected edge cost

Then include ALL valid alternative nodes at the destination layer with:

\- node index

\- q12 vector

\- edge cost from the previously selected Global node

\- shortest-angular delta from previous Global node

\- whether that alternative can be connected by the current graph

\- Global cumulative cost/backpointer information if available

Do not label an IK "branch family" unless branch identity is actually

known.

Use terms such as:

"alternative candidate"

rather than claiming elbow-up/down.

\============================================================

IMPORTANT — RETAIN FUTURE-COST EVIDENCE

\============================================================

Because Global Graph Search optimizes the COMPLETE remaining path, not

just the local W1->W2 edge, capture enough DP information to answer:

"Did Global choose a locally larger/stranger transition because it

reduced cumulative future cost later?"

Therefore retain cumulative DP cost or equivalent values for all nodes

needed to compare candidate paths.

\============================================================

FILE WRITE SAFETY

\============================================================

Debug output must be JSON.

Prefer deterministic structured JSON.

Use an atomic write if practical:

write temporary file then replace target.

Do not append multiple runs into an unreadable stream.

One planning request may overwrite the configured debug path so the

latest reproduction is easy to inspect.

If diagnostic serialization/write fails:

\- do NOT alter the selected trajectory

\- surface/log a clear diagnostic-capture warning

\- do not silently change planner behavior

\============================================================

NO BEHAVIORAL CHANGES

\============================================================

Explicitly preserve these current behaviors, even though some are

suspected design gaps:

\- raw squared-L2 Global edge cost

\- current candidate dedup semantics

\- current seed generation

\- current candidate ordering

\- current layer-0 initialization

\- current Global tie-breaking

\- current Global-selected production path

\- current Greedy implementation

\- current Phase-4 thresholds

This checkpoint is OBSERVATION ONLY.

\============================================================

TESTS

\============================================================

Add focused tests proving:

1\. Capture environment variable unset:

\- no capture file

\- selected Global path unchanged

\- costs unchanged

2\. Capture enabled:

\- valid JSON is written

3\. Capture contains exact input waypoints/start/grasp authority.

4\. Every valid graph layer/node is represented.

5\. Candidate provenance is retained.

6\. Rejected pair diagnostics are retained where current source exposes

them.

7\. Global selected node/path is retained.

8\. Greedy selected node/path is retained.

9\. Global local/cumulative costs are retained.

10\. Raw deltas are retained.

11\. Shortest-angular deltas are retained.

12\. 2\*pi-equivalent synthetic candidates visibly demonstrate different:

\- raw delta

\- shortest-angular delta

without changing production cost.

13\. Diagnostic capture does NOT change selected path.

14\. Diagnostic capture does NOT change Global total cost.

15\. Diagnostic capture does NOT change Greedy total cost.

16\. Diagnostic write failure does not mutate planner selection.

17\. No frontend/operator behavior changes are required.

18\. No motion-related code is introduced.

\============================================================

REGRESSION

\============================================================

Run focused Phase-3 tests first.

Then:

python3 -m unittest discover \\

\-s dual\_arm\_app/tests \\

\-p 'test\_object\_trajectory\_ik.py' -v

python3 -m unittest discover \\

\-s dual\_arm\_app/tests \\

\-p 'test\_object\*.py' -v

python3 -m unittest discover \\

\-s dual\_arm\_app/tests \\

\-p 'test\_\*validation\*.py' -v

python3 -m unittest discover \\

\-s dual\_arm\_app/tests \\

\-p 'test\_\*.py' -v

Run:

git diff --check

\============================================================

SAFETY / WORKTREE

\============================================================

This implementation stage does not require:

\- ROS runtime

\- MoveIt runtime

\- JAKA driver

\- physical robot

Do not connect a robot.

Do not send motion.

Preserve dirty worktree.

Do NOT:

git reset

git clean

git stash

git checkout over work

git stage

git commit

git merge

git push

\============================================================

DELIVERABLE

\============================================================

Report:

1\. Files changed

2\. Environment-variable capture interface

3\. Exact input data captured

4\. Candidate/node data captured

5\. Rejected-pair data captured

6\. Global DP/path data captured

7\. Greedy path data captured

8\. Raw vs shortest-angular transition diagnostics

9\. Suspicious-transition/alternative summary

10\. Atomic/write-failure behavior

11\. Confirmation planner objective unchanged

12\. Confirmation layer-0 behavior unchanged

13\. Confirmation candidate generation/dedup unchanged

14\. Focused tests

15\. Object trajectory IK regression

16\. Object/planner regression

17\. Validation regression

18\. Full regression

19\. git diff --check

20\. no-motion confirmation

Do not commit.

End exactly:

PHASE3\_IK\_BRANCH\_CAPTURE\_READY

or

PHASE3\_IK\_BRANCH\_CAPTURE\_BLOCKED: