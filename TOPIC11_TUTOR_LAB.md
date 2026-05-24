# Topic 11 Tutor and Lab Hour

## Title

`Topic 11: Mission Analysis and Decision Making (Autonomous)`

## Big Idea

Topics `7-10` helped students answer:

- Where am I?
- What is around me?
- How do I move safely?

Topic `11` asks:

- What should the robot do next?

This is the first topic where the autonomy stack is explicitly lifted to the `mission level`.

---

## Learning Outcomes

By the end of the tutor and lab session, students should be able to:

- decompose a robotic mission into phases
- identify mission-relevant state variables
- distinguish `continuous robot state` from `discrete mission state`
- design a simple `FSM` for a quadcopter mission
- implement mission transitions in a PyBullet simulation

---

## Shared Running Example

Use one mission through the full session:

`Takeoff -> Transit -> Inspect -> Return -> Land`

Possible off-nominal conditions:

- low battery
- obstacle detected
- weak localization confidence
- timeout or stalled progress

---

## Tutor Hour

### Recommended duration

`50-60 minutes`

### Tutor-hour structure

#### 1. Mission framing: 10 min

Discuss:

- What is the mission objective?
- What counts as mission success?
- What counts as mission failure?
- What internal resources matter?

Use the quadcopter example:

- inspect a target region in a warehouse
- avoid obstacles
- return safely before battery becomes critical

#### 2. Mission decomposition: 10 min

Ask students to break the mission into phases:

- `Idle`
- `Preflight`
- `Takeoff`
- `Transit`
- `Inspect`
- `Return`
- `Land`

Then ask:

- Which transitions are nominal?
- Which transitions are emergency or recovery transitions?

#### 3. State-space thinking: 10-15 min

Introduce the mission-relevant state as the robot's `minimum useful memory`.

Discuss variables such as:

- position
- velocity
- heading
- battery level
- localization confidence
- distance to goal

Use the distinction:

- continuous state:
  `x, y, z, v_x, v_y, v_z, battery`
- discrete mission mode:
  `Takeoff, Transit, Inspect, Return`

Key teaching point:

Mission decisions are often triggered when a continuous state crosses a threshold.

Examples:

- `battery < b_thr`
- `distance_to_goal < d_thr`
- `confidence < c_thr`

#### 4. FSM design: 15 min

Build a simple mission FSM on the board.

Nominal flow:

- `Idle -> Takeoff -> Transit -> Inspect -> Return -> Land`

Then extend it:

- `Transit -> Replan` if obstacle blocks route
- `Inspect -> Return` if battery gets low
- `Any state -> Emergency Land` if critical failure occurs
- `Transit -> Safe Hover` if localization becomes unreliable

Ask students:

- What is the event?
- What is the guard condition?
- What action should be taken after the transition?

#### 5. Wrap-up discussion: 5 min

Ask:

- Why is planning alone not enough?
- Why is control alone not enough?
- Why is FSM a good starting point for mission autonomy?
- What are the limits of FSM if the mission becomes too complex?

---

## Tutor Hour Deliverable

Each student group should leave the tutor hour with:

- a mission-phase list
- a list of mission-relevant state variables
- an FSM sketch with at least one abnormal transition

---

## Lab Hour

### Recommended duration

`90-120 minutes`

### Lab objective

Implement a mission-level finite state machine for a quadcopter in PyBullet.

The mission manager should:

- advance through nominal mission states
- react to at least one off-nominal condition
- print or display the active mission state during execution

---

## Lab Assumptions

The lab assumes students already have:

- a quadcopter or aerial vehicle simulation in PyBullet
- a basic way to send waypoint or pose references
- a loop that updates the simulation at each time step

This lab does **not** require building the low-level controller from scratch.

The focus is:

- mission logic
- transitions
- supervisory autonomy

---

## Lab Tasks

### Task 1. Define the mission states

Students implement at least:

- `IDLE`
- `TAKEOFF`
- `TRANSIT`
- `INSPECT`
- `RETURN_HOME`
- `LAND`
- `EMERGENCY_LAND`

Optional:

- `REPLAN`
- `SAFE_HOVER`

### Task 2. Define mission context variables

Students should track a context object or dictionary containing values such as:

- current position
- current altitude
- distance to current waypoint
- battery level
- inspection complete flag
- localization confidence
- obstacle flag
- timer or progress counter

### Task 3. Implement transition rules

Examples:

- `IDLE -> TAKEOFF` when mission starts
- `TAKEOFF -> TRANSIT` when altitude threshold is reached
- `TRANSIT -> INSPECT` when destination threshold is reached
- `INSPECT -> RETURN_HOME` when inspection is complete
- `RETURN_HOME -> LAND` when home position is reached
- `ANY -> EMERGENCY_LAND` when battery is critical

### Task 4. Connect each mission state to a high-level action

Examples:

- `TAKEOFF`: command target altitude
- `TRANSIT`: command waypoint motion
- `INSPECT`: hold position for a fixed time
- `RETURN_HOME`: command home waypoint
- `LAND`: command descent
- `SAFE_HOVER`: hold current pose

### Task 5. Add one abnormal scenario

At minimum, require one of these:

- battery drops below threshold
- obstacle flag appears during transit
- localization confidence drops below threshold

The robot should respond sensibly by changing state.

---

## Suggested PyBullet Lab Flow

### Phase A. Nominal mission

Students first demonstrate:

- takeoff
- move to target
- hold for inspection
- return
- land

### Phase B. Add one abnormal event

Examples:

- after 20 seconds, force battery to `12%`
- when the robot reaches a corridor, set `obstacle_detected = True`
- after inspection begins, drop localization confidence

### Phase C. Demonstrate recovery behavior

Students should show:

- transition to `RETURN_HOME`, `SAFE_HOVER`, or `EMERGENCY_LAND`
- printed log of why the transition happened

---

## Recommended Console Output

Encourage a readable mission log, for example:

```text
[MISSION] State: TAKEOFF
[MISSION] Altitude reached -> TRANSIT
[MISSION] State: TRANSIT
[MISSION] Battery low -> RETURN_HOME
[MISSION] State: RETURN_HOME
[MISSION] Home reached -> LAND
```

This makes grading and debugging much easier.

---

## Assessment Rubric for Topic 11 Lab

### 1. Mission modeling: 30%

- clear mission phases
- meaningful state variables
- sensible transition logic

### 2. Working implementation: 40%

- FSM runs correctly
- mission completes in nominal case
- transitions are triggered correctly

### 3. Off-nominal handling: 20%

- at least one abnormal condition handled
- safe recovery behavior demonstrated

### 4. Explanation and reflection: 10%

- short explanation of why the chosen transitions make sense

---

## Minimum Passing Version

A minimum successful submission should:

- implement `Idle, Takeoff, Transit, Inspect, Return, Land`
- complete the nominal mission
- react correctly to one battery-based abnormal condition

---

## Stronger Extension Ideas

If students finish early, they can add:

- a `REPLAN` state
- a `SAFE_HOVER` state
- a timeout-based failure detector
- a keyboard-based human override
- a visual HUD showing current state

---

## Suggested Instructor Closing

End the session with:

`State-space modeling tells us what the robot currently is.`

`FSM tells us what the mission logic should do next.`

`Autonomous decision making begins when those two are connected.`

