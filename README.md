# Topic 11: Mission Analysis and Decision Making

This repository contains a shareable `Tutor + Lab` package for `Topic 11` in the course:

`Elements of Robotics and Automation`

The material is built around a `quadcopter mission` and assumes `PyBullet` is the simulation framework used in the course.

## Contents

- `TOPIC11_TUTOR_LAB.md`
  The main tutor-hour and lab-hour handout.
- `fsm_mission_manager.py`
  A small Python starter scaffold for mission-level finite state machine logic.
- `pybullet_delivery_emergency_demo.py`
  A visual PyBullet demo of a delivery drone that diverts to an emergency landing pad when the battery drops below a safety threshold.

## Topic Focus

Topic 11 sits above perception, planning, and control. Its role is to answer:

`What should the robot do next?`

The teaching package emphasizes:

- mission decomposition
- state-space thinking
- finite state machines (FSM)
- hybrid autonomy:
  continuous robot state + discrete mission decisions

## Suggested Use

- Use `TOPIC11_TUTOR_LAB.md` as the session guide for students or teaching assistants.
- Use `fsm_mission_manager.py` as a starting point for the lab implementation.
- Connect the FSM outputs to your existing PyBullet quadcopter controller or waypoint follower.

## Expected Lab Mission

The reference mission is:

`Takeoff -> Transit -> Inspect -> Return -> Land`

with possible transitions to:

- `Replan`
- `Safe Hover`
- `Emergency Land`

## Delivery Emergency Demo

The PyBullet demo illustrates a mission-level safety decision:

`Takeoff -> Cruise to Customer -> Low Battery Trigger -> Emergency Divert -> Emergency Descent -> Land`

What students should notice:

- the mission begins as a nominal delivery flight
- battery is monitored as an internal mission state
- the controller does not wait for battery depletion
- once the threshold is crossed, the mission supervisor changes the objective from `delivery` to `safe landing`

Run it with the Python 3.11 virtual environment used for the patched PyBullet build:

```bash
/tmp/topic11-tutor-lab/.venv311/bin/python /tmp/topic11-tutor-lab/pybullet_delivery_emergency_demo.py --gif /tmp/topic11-tutor-lab/delivery_emergency_demo.gif
```

Optional GUI mode:

```bash
/tmp/topic11-tutor-lab/.venv311/bin/python /tmp/topic11-tutor-lab/pybullet_delivery_emergency_demo.py --gui --gif /tmp/topic11-tutor-lab/delivery_emergency_demo.gif
```

## Suggested Student Deliverables

- a mission-state diagram
- a working FSM implementation
- a short demonstration in PyBullet
- a short reflection on one abnormal scenario
