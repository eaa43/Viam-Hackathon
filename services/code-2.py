"""Mailbox letter extractor (generic service).

Meant for SUPERVISED, step-by-step testing: run one action at a time and watch
the arm. Poses are in the WORLD frame, in mm, and describe the gripper TCP.

do_command actions ({"action": ...}):
  - "plan":           no motion; returns the computed approach/grasp/lifted/hold
                      poses, speeds and max_attempts.
  - "go-to-approach": open gripper and move to the standoff above the grasp pose.
  - "extract-one":    approach, descend slowly, grab, lift straight up and
                      (optionally) move to hold_pose. Retries up to max_attempts.
  - "release":        open the gripper.
  - "reset-count":    reset letter_index to 0.
  - "stop":           stop the arm.
  - "status":         letter_index and the last extract-one result.
  - "sweep":          rotate ONLY the base joint (joint 0) from sweep_start_deg to
                      sweep_end_deg in sweep_step_deg steps, detect objects with the
                      segmenter at each stop, merge them, then return to the start
                      joints. Stores the detections.
  - "find":           no motion; detect objects from the current position.
  - "extract-detected": {"index": 0} extract at a stored detection (top-down grasp).

Config attributes: arm, gripper, grasp_pose {x,y,z,o_x,o_y,o_z,theta},
approach_mm, lift_mm, step_mm, step_axis ("x"|"y"), slow_speed_degs,
travel_speed_degs, max_attempts, hold_pose, camera, segmenter, sweep_start_deg,
sweep_end_deg, sweep_step_deg, settle_s, merge_mm, grasp_z_offset_mm,
min_grasp_z_mm, grasp_theta_deg.
"""

import asyncio
import math
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Pose, PoseInFrame, ResourceName, Transform
from viam.proto.component.arm import JointPositions
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.generic import Generic as GenericService
from viam.services.motion import MotionClient
from viam.services.vision import VisionClient
from viam.utils import ValueTypes, struct_to_dict

MAX_SWEEP_RANGE_DEG = 120.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def pose_from_dict(d: Optional[Mapping[str, Any]]) -> Optional[Pose]:
    if not d or not isinstance(d, Mapping):
        return None
    return Pose(
        x=float(d.get("x", 0.0)),
        y=float(d.get("y", 0.0)),
        z=float(d.get("z", 0.0)),
        o_x=float(d.get("o_x", 0.0)),
        o_y=float(d.get("o_y", 0.0)),
        o_z=float(d.get("o_z", 1.0)),
        theta=float(d.get("theta", 0.0)),
    )


def pose_to_dict(p: Optional[Pose]) -> Optional[Dict[str, float]]:
    if p is None:
        return None
    return {
        "x": p.x,
        "y": p.y,
        "z": p.z,
        "o_x": p.o_x,
        "o_y": p.o_y,
        "o_z": p.o_z,
        "theta": p.theta,
    }


def offset_z(p: Pose, dz: float) -> Pose:
    return Pose(x=p.x, y=p.y, z=p.z + dz, o_x=p.o_x, o_y=p.o_y, o_z=p.o_z, theta=p.theta)


# IMPORTANT: Do not change the class name or the MODEL triplet below.
# The platform uses these auto-generated values to identify your module.
# Changing them will break your inline module.
class MyGenericService(GenericService, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hackathons", "5d314490-f1b4-45bd-a4d8-4296e9622caf"), "generic-service")

    arm: Arm
    gripper: Gripper
    motion: MotionClient
    letter_index: int = 0
    last_result: Optional[Dict[str, Any]] = None
    segmenter: VisionClient
    detections: List[Dict[str, Any]] = []

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        arm_name = str(attrs.get("arm", "arm") or "arm")
        gripper_name = str(attrs.get("gripper", "gripper") or "gripper")
        camera_name = str(attrs.get("camera", "cam") or "cam")
        segmenter_name = str(attrs.get("segmenter", "object-segmenter") or "object-segmenter")
        axis = str(attrs.get("step_axis", "x")).lower()
        if axis not in ("x", "y"):
            raise ValueError("step_axis must be 'x' or 'y'")
        # Numeric attributes must parse if present.
        for key in (
            "approach_mm", "lift_mm", "step_mm", "slow_speed_degs", "travel_speed_degs",
            "sweep_start_deg", "sweep_end_deg", "sweep_step_deg", "settle_s", "merge_mm",
            "grasp_z_offset_mm", "min_grasp_z_mm", "grasp_theta_deg",
        ):
            if key in attrs:
                float(attrs[key])  # type: ignore[arg-type]
        if "max_attempts" in attrs:
            int(float(attrs["max_attempts"]))  # type: ignore[arg-type]
        required: List[str] = [
            arm_name, gripper_name, camera_name, segmenter_name, "rdk:service:motion/builtin"
        ]
        optional: List[str] = []
        return required, optional

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """All setup happens here; viam-server calls new() on every config change."""
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)

        self.arm_name = str(attrs.get("arm", "arm") or "arm")
        self.gripper_name = str(attrs.get("gripper", "gripper") or "gripper")

        gp = attrs.get("grasp_pose")
        self.grasp_pose: Optional[Pose] = pose_from_dict(gp if isinstance(gp, Mapping) else None)
        hp = attrs.get("hold_pose")
        self.hold_pose: Optional[Pose] = pose_from_dict(hp if isinstance(hp, Mapping) else None)

        self.approach_mm = float(attrs.get("approach_mm", 100.0))  # type: ignore[arg-type]
        self.lift_mm = float(attrs.get("lift_mm", 200.0))  # type: ignore[arg-type]
        self.step_mm = float(attrs.get("step_mm", 0.0))  # type: ignore[arg-type]
        axis = str(attrs.get("step_axis", "x")).lower()
        self.step_axis = axis if axis in ("x", "y") else "x"
        self.slow_speed = _clamp(float(attrs.get("slow_speed_degs", 5.0)), 3.0, 30.0)  # type: ignore[arg-type]
        self.travel_speed = _clamp(float(attrs.get("travel_speed_degs", 10.0)), 3.0, 30.0)  # type: ignore[arg-type]
        self.max_attempts = int(_clamp(int(float(attrs.get("max_attempts", 3))), 1, 5))  # type: ignore[arg-type]

        self.arm = dependencies[Arm.get_resource_name(self.arm_name)]  # type: ignore[assignment]
        self.gripper = dependencies[Gripper.get_resource_name(self.gripper_name)]  # type: ignore[assignment]
        self.motion = dependencies[MotionClient.get_resource_name("builtin")]  # type: ignore[assignment]

        # Camera sweep / detection settings.
        self.camera_name = str(attrs.get("camera", "cam") or "cam")
        self.segmenter_name = str(attrs.get("segmenter", "object-segmenter") or "object-segmenter")
        self.sweep_start = float(attrs.get("sweep_start_deg", -30.0))  # type: ignore[arg-type]
        self.sweep_end = float(attrs.get("sweep_end_deg", 30.0))  # type: ignore[arg-type]
        self.sweep_step = _clamp(float(attrs.get("sweep_step_deg", 15.0)), 5.0, 45.0)  # type: ignore[arg-type]
        self.settle_s = max(0.0, float(attrs.get("settle_s", 1.0)))  # type: ignore[arg-type]
        self.merge_mm = max(0.0, float(attrs.get("merge_mm", 40.0)))  # type: ignore[arg-type]
        self.grasp_z_offset = float(attrs.get("grasp_z_offset_mm", 0.0))  # type: ignore[arg-type]
        self.min_grasp_z = float(attrs.get("min_grasp_z_mm", 60.0))  # type: ignore[arg-type]
        self.grasp_theta = float(attrs.get("grasp_theta_deg", 0.0))  # type: ignore[arg-type]
        self.segmenter = dependencies[VisionClient.get_resource_name(self.segmenter_name)]  # type: ignore[assignment]
        self.detections = []

        self.letter_index = 0
        self.last_result = None
        self.logger.info(
            f"mailbox extractor ready: arm={self.arm_name} gripper={self.gripper_name} "
            f"grasp_pose configured={self.grasp_pose is not None}"
        )
        return self

    # ---------------- helpers ----------------

    def target_grasp(self) -> Optional[Pose]:
        if self.grasp_pose is None:
            return None
        g = self.grasp_pose
        off = self.letter_index * self.step_mm
        x = g.x + (off if self.step_axis == "x" else 0.0)
        y = g.y + (off if self.step_axis == "y" else 0.0)
        return Pose(x=x, y=y, z=g.z, o_x=g.o_x, o_y=g.o_y, o_z=g.o_z, theta=g.theta)

    async def move_to(self, pose: Pose, label: str = "") -> None:
        self.logger.info(f"moving to {label}: {pose_to_dict(pose)}")
        await self.motion.move(
            component_name=self.gripper_name,
            destination=PoseInFrame(reference_frame="world", pose=pose),
        )

    async def set_speed(self, v: float) -> None:
        try:
            await self.arm.do_command({"set_speed": float(v)})
        except Exception as e:
            self.logger.warning(f"set_speed({v}) failed: {e}")

    def sweep_angles(self) -> List[float]:
        start, end, step = self.sweep_start, self.sweep_end, self.sweep_step
        sign = 1.0 if end >= start else -1.0
        n = int(math.floor(abs(end - start) / step + 1e-9))
        angles = [start + sign * i * step for i in range(n + 1)]
        if abs(angles[-1] - end) > 1e-6:
            angles.append(end)
        return angles

    async def detect_world(self) -> List[Dict[str, Any]]:
        """Detect objects with the segmenter and return their centers in the world frame."""
        out: List[Dict[str, Any]] = []
        objs = await self.segmenter.get_object_point_clouds(self.camera_name)
        for obj in objs:
            try:
                geoms = list(obj.geometries.geometries)
                if not geoms:
                    continue
                g = geoms[0]
                pif = await self.motion.get_pose(
                    component_name="detected_object",
                    destination_frame="world",
                    supplemental_transforms=[
                        Transform(
                            reference_frame="detected_object",
                            pose_in_observer_frame=PoseInFrame(
                                reference_frame=self.camera_name, pose=g.center
                            ),
                        )
                    ],
                )
                p = pif.pose
                out.append({"x": p.x, "y": p.y, "z": p.z, "label": g.label})
            except Exception as e:
                self.logger.warning(f"detect_world: object skipped: {e}")
        return out

    def merge_detections(self, dets: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        clusters: List[Dict[str, Any]] = []
        for d in dets:
            match = None
            for c in clusters:
                n = c["hits"]
                cx, cy, cz = c["sx"] / n, c["sy"] / n, c["sz"] / n
                if math.dist((cx, cy, cz), (d["x"], d["y"], d["z"])) <= self.merge_mm:
                    match = c
                    break
            if match is None:
                clusters.append({
                    "sx": d["x"], "sy": d["y"], "sz": d["z"], "hits": 1,
                    "label": d.get("label", ""), "scan_angles": [d.get("scan_angle")],
                })
            else:
                match["sx"] += d["x"]
                match["sy"] += d["y"]
                match["sz"] += d["z"]
                match["hits"] += 1
                match["scan_angles"].append(d.get("scan_angle"))
        merged: List[Dict[str, Any]] = []
        for c in clusters:
            n = c["hits"]
            merged.append({
                "x": c["sx"] / n, "y": c["sy"] / n, "z": c["sz"] / n,
                "label": c["label"], "hits": n, "scan_angles": c["scan_angles"],
            })
        merged.sort(key=lambda m: math.hypot(m["x"], m["y"]))
        return merged

    def detection_grasp(self, d: Mapping[str, Any]) -> Pose:
        return Pose(
            x=float(d["x"]),
            y=float(d["y"]),
            z=max(float(d["z"]) + self.grasp_z_offset, self.min_grasp_z),
            o_x=0.0, o_y=0.0, o_z=-1.0,
            theta=self.grasp_theta,
        )

    async def _sweep(self) -> Dict[str, Any]:
        if abs(self.sweep_end - self.sweep_start) > MAX_SWEEP_RANGE_DEG:
            return {"success": False, "error": f"sweep range exceeds {MAX_SWEEP_RANGE_DEG} deg"}
        start_joints: Optional[List[float]] = None
        try:
            await self.set_speed(self.travel_speed)
            jp = await self.arm.get_joint_positions()
            start_joints = list(jp.values)
            raw: List[Dict[str, Any]] = []
            for angle in self.sweep_angles():
                vals = list(start_joints)
                vals[0] = angle
                self.logger.info(f"sweep: base joint -> {angle:.1f} deg")
                await self.arm.move_to_joint_positions(JointPositions(values=vals))
                await asyncio.sleep(self.settle_s)
                dets = await self.detect_world()
                for d in dets:
                    d["scan_angle"] = angle
                raw.extend(dets)
            self.detections = self.merge_detections(raw)
            return {"success": True, "count": len(self.detections), "detections": self.detections}
        except Exception as e:
            self.logger.error(f"sweep failed: {e}")
            try:
                await self.arm.stop()
            except Exception as se:
                self.logger.warning(f"arm.stop failed: {se}")
            return {"success": False, "error": str(e)}
        finally:
            if start_joints is not None:
                try:
                    await self.arm.move_to_joint_positions(JointPositions(values=start_joints))
                except Exception as re:
                    self.logger.error(f"sweep: return to start joints failed: {re}")

    # ---------------- actions ----------------

    async def _extract_one(self) -> Dict[str, Any]:
        grasp = self.target_grasp()
        if grasp is None:
            return {"error": "grasp_pose not configured"}
        return await self._extract(grasp, count_letter=True)

    async def _extract(self, grasp: Pose, count_letter: bool = False) -> Dict[str, Any]:
        approach = offset_z(grasp, self.approach_mm)
        lifted = offset_z(grasp, self.lift_mm)
        attempts = 0
        try:
            for attempt in range(1, self.max_attempts + 1):
                attempts = attempt
                self.logger.info(f"extract-one: attempt {attempt}/{self.max_attempts}")
                await self.gripper.open()
                await self.set_speed(self.travel_speed)
                await self.move_to(approach, "approach")
                await self.set_speed(self.slow_speed)
                await self.move_to(grasp, "grasp")
                self.logger.info("grabbing")
                got = await self.gripper.grab()
                if got:
                    self.logger.info("letter grasped, lifting")
                    await self.set_speed(self.slow_speed)
                    await self.move_to(lifted, "lifted")
                    await self.set_speed(self.travel_speed)
                    if self.hold_pose is not None:
                        await self.move_to(self.hold_pose, "hold")
                    if count_letter:
                        self.letter_index += 1
                    return {
                        "success": True,
                        "holding": True,
                        "attempts": attempt,
                        "letter_index": self.letter_index,
                    }
                self.logger.info("nothing grasped, backing off to approach")
                await self.gripper.open()
                await self.set_speed(self.slow_speed)
                await self.move_to(approach, "approach")
            await self.set_speed(self.travel_speed)
            return {
                "success": False,
                "reason": "no letter grasped (mailbox empty or misaligned)",
                "attempts": attempts,
            }
        except Exception as e:
            # Do NOT open the gripper here: it may drop a letter mid-air.
            self.logger.error(f"extract-one failed: {e}")
            try:
                await self.arm.stop()
            except Exception as se:
                self.logger.warning(f"arm.stop failed: {se}")
            await self.set_speed(self.travel_speed)
            return {"success": False, "error": str(e), "attempts": attempts}
        finally:
            await self.set_speed(self.travel_speed)

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        action = str(command.get("action", ""))
        self.logger.info(f"do_command action={action}")

        if action == "plan":
            grasp = self.target_grasp()
            return {
                "configured": grasp is not None,
                "letter_index": self.letter_index,
                "approach": pose_to_dict(offset_z(grasp, self.approach_mm)) if grasp else None,
                "grasp": pose_to_dict(grasp),
                "lifted": pose_to_dict(offset_z(grasp, self.lift_mm)) if grasp else None,
                "hold": pose_to_dict(self.hold_pose),
                "speeds": {"slow": self.slow_speed, "travel": self.travel_speed},
                "max_attempts": self.max_attempts,
                "sweep": {
                    "camera": self.camera_name,
                    "segmenter": self.segmenter_name,
                    "start_deg": self.sweep_start,
                    "end_deg": self.sweep_end,
                    "step_deg": self.sweep_step,
                    "angles": self.sweep_angles(),  # type: ignore[dict-item]
                    "settle_s": self.settle_s,
                    "merge_mm": self.merge_mm,
                    "grasp_z_offset_mm": self.grasp_z_offset,
                    "min_grasp_z_mm": self.min_grasp_z,
                    "grasp_theta_deg": self.grasp_theta,
                    "max_sweep_range_deg": MAX_SWEEP_RANGE_DEG,
                },
                "detections": self.detections,  # type: ignore[dict-item]
            }

        if action == "go-to-approach":
            grasp = self.target_grasp()
            if grasp is None:
                return {"error": "grasp_pose not configured"}
            try:
                await self.gripper.open()
                await self.set_speed(self.travel_speed)
                await self.move_to(offset_z(grasp, self.approach_mm), "approach")
                return {"success": True}
            except Exception as e:
                self.logger.error(f"go-to-approach failed: {e}")
                return {"success": False, "error": str(e)}

        if action == "extract-one":
            result = await self._extract_one()
            self.last_result = result
            return result

        if action == "sweep":
            return await self._sweep()

        if action == "find":
            try:
                dets = await self.detect_world()
                return {"success": True, "count": len(dets), "detections": dets}  # type: ignore[dict-item]
            except Exception as e:
                self.logger.error(f"find failed: {e}")
                return {"success": False, "error": str(e)}

        if action == "extract-detected":
            if not self.detections:
                return {"error": "no detections; run sweep first"}
            idx = int(command.get("index", 0) or 0)  # type: ignore[arg-type]
            if idx < 0 or idx >= len(self.detections):
                return {"error": f"index {idx} out of range (0..{len(self.detections) - 1})"}
            det = self.detections[idx]
            grasp = self.detection_grasp(det)
            result = await self._extract(grasp)
            result["grasp_pose"] = pose_to_dict(grasp)
            result["detection"] = det
            if result.get("success"):
                self.detections.pop(idx)
            result["remaining_detections"] = len(self.detections)
            self.last_result = result
            return result

        if action == "release":
            await self.gripper.open()
            return {"success": True}

        if action == "reset-count":
            self.letter_index = 0
            return {"success": True, "letter_index": 0}

        if action == "stop":
            await self.arm.stop()
            return {"stopped": True}

        if action == "status":
            return {"letter_index": self.letter_index, "last_result": self.last_result}

        return {"error": "unknown action"}

    async def get_status(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        return {"letter_index": self.letter_index, "last_result": self.last_result}
