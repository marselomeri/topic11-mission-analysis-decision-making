from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

import pybullet as p
import pybullet_data


class MissionState(Enum):
    TAKEOFF = auto()
    CRUISE_TO_CUSTOMER = auto()
    EMERGENCY_DIVERT = auto()
    EMERGENCY_DESCENT = auto()
    LANDED = auto()


@dataclass
class DemoConfig:
    dt: float = 1.0 / 60.0
    cruise_altitude: float = 2.5
    landing_altitude: float = 0.22
    speed_xy: float = 1.1
    speed_z: float = 0.85
    battery_drain_per_step: float = 0.12
    low_battery_threshold: float = 45.0
    render_every_n_steps: int = 3
    image_width: int = 960
    image_height: int = 540


def create_box(
    half_extents: tuple[float, float, float],
    position: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
    mass: float = 0.0,
) -> int:
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position,
    )


def create_cylinder(
    radius: float,
    length: float,
    position: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
    mass: float = 0.0,
) -> int:
    collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=length)
    visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=rgba)
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position,
    )


def create_scene() -> dict[str, object]:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.loadURDF("plane.urdf")

    create_box((0.55, 0.55, 0.25), (0.0, 0.0, 0.25), (0.75, 0.23, 0.18, 1.0))
    create_box((0.55, 0.55, 0.25), (8.0, 0.0, 0.25), (0.15, 0.55, 0.22, 1.0))

    create_box((0.8, 0.8, 1.8), (2.3, 1.9, 1.8), (0.55, 0.58, 0.65, 1.0))
    create_box((0.75, 0.75, 2.3), (4.8, -1.5, 2.3), (0.47, 0.5, 0.56, 1.0))
    create_box((0.6, 1.0, 1.5), (6.1, 1.8, 1.5), (0.52, 0.54, 0.6, 1.0))
    create_box((0.7, 0.7, 1.2), (3.8, -3.0, 1.2), (0.6, 0.62, 0.68, 1.0))

    emergency_pads = [
        np.array((3.5, 2.8, 0.02), dtype=float),
        np.array((6.4, -2.6, 0.02), dtype=float),
        np.array((1.8, -2.7, 0.02), dtype=float),
    ]
    for pad in emergency_pads:
        create_cylinder(0.55, 0.04, tuple(pad), (0.96, 0.84, 0.18, 1.0))

    drone_body = create_box((0.18, 0.18, 0.05), (0.0, 0.0, 0.22), (0.18, 0.34, 0.82, 1.0), mass=0.8)
    package = create_box((0.08, 0.08, 0.04), (0.0, 0.0, 0.1), (0.88, 0.52, 0.12, 1.0), mass=0.1)

    return {
        "drone": drone_body,
        "package": package,
        "start_xy": np.array((0.0, 0.0), dtype=float),
        "customer_xy": np.array((8.0, 0.0), dtype=float),
        "pads": emergency_pads,
    }


def move_toward(current: np.ndarray, target: np.ndarray, max_step: float) -> np.ndarray:
    delta = target - current
    distance = float(np.linalg.norm(delta))
    if distance <= max_step or distance == 0.0:
        return target.copy()
    return current + (delta / distance) * max_step


def draw_overlay(
    rgba: np.ndarray,
    state: MissionState,
    battery: float,
    emergency_pad: np.ndarray | None,
    current_xy: np.ndarray,
    step: int,
) -> np.ndarray:
    image = Image.fromarray(rgba[..., :3])
    draw = ImageDraw.Draw(image)

    draw.rounded_rectangle((18, 18, 440, 150), radius=18, fill=(14, 20, 29))
    draw.text((36, 34), "Drone Delivery Mission", fill=(245, 245, 245))
    draw.text((36, 64), f"State: {state.name.replace('_', ' ').title()}", fill=(222, 230, 236))
    draw.text((36, 90), f"Battery: {battery:5.1f}%", fill=(222, 230, 236))
    draw.text((36, 116), f"Position: ({current_xy[0]:4.1f}, {current_xy[1]:4.1f})", fill=(222, 230, 236))

    bar_left, bar_top = 250, 88
    bar_width, bar_height = 150, 18
    draw.rounded_rectangle(
        (bar_left, bar_top, bar_left + bar_width, bar_top + bar_height),
        radius=8,
        outline=(220, 220, 220),
        width=2,
    )
    fill_color = (54, 188, 108) if battery > 45 else (236, 186, 45) if battery > 30 else (214, 65, 65)
    fill_width = max(6, int((battery / 100.0) * (bar_width - 4)))
    draw.rounded_rectangle(
        (bar_left + 2, bar_top + 2, bar_left + 2 + fill_width, bar_top + bar_height - 2),
        radius=6,
        fill=fill_color,
    )

    draw.rounded_rectangle((540, 18, 930, 130), radius=18, fill=(252, 246, 228))
    if emergency_pad is None:
        draw.text((560, 40), "Nominal delivery route active.", fill=(44, 54, 69))
        draw.text((560, 72), "Emergency pads are on standby.", fill=(44, 54, 69))
    else:
        draw.text((560, 40), "Low battery detected.", fill=(132, 42, 42))
        draw.text(
            (560, 72),
            f"Diverting to pad at ({emergency_pad[0]:.1f}, {emergency_pad[1]:.1f}).",
            fill=(132, 42, 42),
        )
    draw.text((560, 102), f"Frame step: {step}", fill=(44, 54, 69))

    return np.asarray(image)


def simulate_demo(config: DemoConfig, use_gui: bool, output_gif: Path) -> dict[str, object]:
    connection_mode = p.GUI if use_gui else p.DIRECT
    client = p.connect(connection_mode)
    if client < 0:
        raise RuntimeError("PyBullet could not start.")

    try:
        if use_gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=10.0,
                cameraYaw=36.0,
                cameraPitch=-36.0,
                cameraTargetPosition=(4.0, 0.0, 1.2),
            )

        scene = create_scene()
        drone = int(scene["drone"])
        package = int(scene["package"])
        customer_xy = np.array(scene["customer_xy"], dtype=float)
        pads = [np.array(pad, dtype=float) for pad in scene["pads"]]

        state = MissionState.TAKEOFF
        battery = 100.0
        position = np.array((0.0, 0.0, config.landing_altitude), dtype=float)
        yaw = 0.0
        emergency_pad = None
        trace_points: list[np.ndarray] = [position.copy()]
        frames: list[np.ndarray] = []
        event_log = ["Mission started: package pickup complete, takeoff authorized."]

        step = 0
        while state is not MissionState.LANDED and step < 2400:
            step += 1
            battery = max(0.0, battery - config.battery_drain_per_step)
            previous_position = position.copy()

            if state is MissionState.TAKEOFF:
                target = np.array((0.0, 0.0, config.cruise_altitude), dtype=float)
                position[2] = move_toward(position[2:3], target[2:3], config.speed_z * config.dt)[0]
                if abs(position[2] - config.cruise_altitude) < 0.03:
                    state = MissionState.CRUISE_TO_CUSTOMER
                    event_log.append("Takeoff complete: cruising toward customer.")

            elif state is MissionState.CRUISE_TO_CUSTOMER:
                xy_target = customer_xy
                position[:2] = move_toward(position[:2], xy_target, config.speed_xy * config.dt)
                if battery <= config.low_battery_threshold:
                    emergency_pad = min(pads, key=lambda pad: float(np.linalg.norm(pad[:2] - position[:2])))
                    state = MissionState.EMERGENCY_DIVERT
                    event_log.append(
                        "Battery below threshold: emergency divert to nearest landing pad."
                    )

            elif state is MissionState.EMERGENCY_DIVERT:
                assert emergency_pad is not None
                position[:2] = move_toward(position[:2], emergency_pad[:2], config.speed_xy * 1.25 * config.dt)
                if np.linalg.norm(position[:2] - emergency_pad[:2]) < 0.08:
                    state = MissionState.EMERGENCY_DESCENT
                    event_log.append("Emergency pad reached: descending immediately.")

            elif state is MissionState.EMERGENCY_DESCENT:
                target = np.array((position[0], position[1], config.landing_altitude), dtype=float)
                position[2] = move_toward(position[2:3], target[2:3], config.speed_z * config.dt)[0]
                if position[2] <= config.landing_altitude + 0.01:
                    state = MissionState.LANDED
                    event_log.append("Drone landed safely on emergency pad.")

            if trace_points:
                delta_xy = position[:2] - trace_points[-1][:2]
                if np.linalg.norm(delta_xy) > 0.22:
                    trace_points.append(position.copy())
                    color = (0.15, 0.72, 0.23) if state in {MissionState.TAKEOFF, MissionState.CRUISE_TO_CUSTOMER} else (
                        0.96,
                        0.55,
                        0.18,
                    )
                    p.addUserDebugLine(trace_points[-2], trace_points[-1], color, lineWidth=3.0, lifeTime=0)

            motion_vector = position[:2] - previous_position[:2]
            if np.linalg.norm(motion_vector) > 1e-5:
                yaw = math.atan2(motion_vector[1], motion_vector[0])
            orientation = p.getQuaternionFromEuler((0.0, 0.0, yaw))
            p.resetBasePositionAndOrientation(drone, position.tolist(), orientation)
            p.resetBasePositionAndOrientation(package, (position + np.array((0.0, 0.0, -0.15))).tolist(), orientation)
            p.stepSimulation()

            if use_gui:
                import time

                time.sleep(config.dt)

            if step % config.render_every_n_steps == 0:
                view = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=(4.0, 0.0, 0.9),
                    distance=10.5,
                    yaw=34.0,
                    pitch=-38.0,
                    roll=0.0,
                    upAxisIndex=2,
                )
                projection = p.computeProjectionMatrixFOV(
                    fov=60.0,
                    aspect=config.image_width / config.image_height,
                    nearVal=0.1,
                    farVal=30.0,
                )
                _, _, rgba, _, _ = p.getCameraImage(
                    width=config.image_width,
                    height=config.image_height,
                    viewMatrix=view,
                    projectionMatrix=projection,
                    renderer=p.ER_TINY_RENDERER,
                )
                frame = np.reshape(rgba, (config.image_height, config.image_width, 4)).astype(np.uint8)
                frame = draw_overlay(frame, state, battery, emergency_pad, position[:2], step)
                frames.append(frame)

        if not frames:
            raise RuntimeError("No frames were captured from the simulation.")

        imageio.mimsave(output_gif, frames, fps=15, loop=0)
        return {
            "frames": len(frames),
            "battery_final": battery,
            "landing_xy": position[:2].copy(),
            "event_log": event_log,
            "gif_path": output_gif,
        }
    finally:
        p.disconnect()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PyBullet demo: delivery drone diverts to an emergency landing when battery is low."
    )
    parser.add_argument(
        "--gif",
        default="delivery_emergency_demo.gif",
        help="Output GIF path for the rendered simulation.",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Run the PyBullet GUI while also saving the GIF.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_gif = Path(args.gif).expanduser().resolve()
    output_gif.parent.mkdir(parents=True, exist_ok=True)

    result = simulate_demo(DemoConfig(), use_gui=args.gui, output_gif=output_gif)

    print("PyBullet delivery emergency landing demo completed.")
    print(f"GIF saved to: {result['gif_path']}")
    print(f"Frames rendered: {result['frames']}")
    print(f"Final battery: {result['battery_final']:.1f}%")
    print(f"Landing position: ({result['landing_xy'][0]:.2f}, {result['landing_xy'][1]:.2f})")
    print("Event log:")
    for event in result["event_log"]:
        print(f" - {event}")


if __name__ == "__main__":
    main()
