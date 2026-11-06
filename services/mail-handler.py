import asyncio
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple, cast

from typing_extensions import Self
from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.components.switch import Switch
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Pose, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.generic import Generic as GenericService
from viam.utils import ValueTypes, struct_to_dict


def _pose_to_dict(p: Pose) -> Dict[str, Any]:
    return {
        "x": p.x,
        "y": p.y,
        "z": p.z,
        "o_x": p.o_x,
        "o_y": p.o_y,
        "o_z": p.o_z,
        "theta": p.theta,
    }


# IMPORTANT: Do not change the class name or the MODEL triplet below.
# The platform uses these auto-generated values to identify your module.
# Changing them will break your inline module.
class MyGenericService(GenericService, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hackathons", "80afae66-e74b-4bd2-a075-22d8e5baef8b"), "generic-service")
    arm: Arm
    gripper: Gripper
    switch: Switch
    y_offset_mm: float
    z_offset_mm: float
    reader_pose_switch: Optional[Switch]
    gripper_pose_switch: Optional[Switch]
    pick_x_offset_mm: float

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        required_deps = []
        for key in ("arm", "gripper", "switch"):
            val = attrs.get(key)
            if not isinstance(val, str) or not val:
                raise ValueError(f"attribute '{key}' (non-empty string) is required")
            required_deps.append(val)
        if "y_offset_mm" in attrs and not isinstance(attrs["y_offset_mm"], (int, float)):
            raise ValueError("attribute 'y_offset_mm' must be a number")
        if "z_offset_mm" in attrs and not isinstance(attrs["z_offset_mm"], (int, float)):
            raise ValueError("attribute 'z_offset_mm' must be a number")
        if "pick_x_offset_mm" in attrs and not isinstance(attrs["pick_x_offset_mm"], (int, float)):
            raise ValueError("attribute 'pick_x_offset_mm' must be a number")
        for key in ("reader_pose", "gripper_pose"):
            if key in attrs and attrs[key] is not None:
                val = attrs[key]
                if not isinstance(val, str):
                    raise ValueError(f"attribute '{key}' must be a string")
                if val:
                    required_deps.append(val)
        return required_deps, []

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """Called on startup and after every config change; all setup lives here."""
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        self.arm = cast(Arm, dependencies[Arm.get_resource_name(str(attrs["arm"]))])
        self.gripper = cast(Gripper, dependencies[Gripper.get_resource_name(str(attrs["gripper"]))])
        self.switch = cast(Switch, dependencies[Switch.get_resource_name(str(attrs["switch"]))])
        self.y_offset_mm = float(cast(float, attrs.get("y_offset_mm", 50.0)))
        self.z_offset_mm = float(cast(float, attrs.get("z_offset_mm", 20.0)))
        self.pick_x_offset_mm = float(cast(float, attrs.get("pick_x_offset_mm", -100.0)))
        reader_name = attrs.get("reader_pose")
        gripper_pose_name = attrs.get("gripper_pose")
        self.reader_pose_switch = (
            cast(Switch, dependencies[Switch.get_resource_name(str(reader_name))])
            if reader_name
            else None
        )
        self.gripper_pose_switch = (
            cast(Switch, dependencies[Switch.get_resource_name(str(gripper_pose_name))])
            if gripper_pose_name
            else None
        )
        return self

    async def _wait_until_stopped(self, timeout_s: float = 30.0) -> None:
        loop = asyncio.get_event_loop()
        deadline = loop.time() + timeout_s
        while await self.arm.is_moving():
            if loop.time() > deadline:
                raise TimeoutError(f"arm still moving after {timeout_s}s")
            await asyncio.sleep(0.2)

    async def _handle_mail(self, offset: float, z_offset: float) -> Dict[str, Any]:
        # 1) go to saved pose
        try:
            await self.switch.set_position(2)
            await self._wait_until_stopped(30.0)
        except Exception as e:
            return {"status": "error", "step": "go_to_pose", "error": str(e)}

        # 1b) raise along +z by z_offset
        try:
            zpose = await self.arm.get_end_position()
            raised_pose = Pose(
                x=zpose.x,
                y=zpose.y,
                z=zpose.z + z_offset,
                o_x=zpose.o_x,
                o_y=zpose.o_y,
                o_z=zpose.o_z,
                theta=zpose.theta,
            )
            await self.arm.move_to_position(raised_pose)
        except Exception as e:
            return {"status": "error", "step": "raise_z", "error": str(e)}
        # 2) grab
        try:
            grabbed = await self.gripper.grab()
        except Exception as e:
            return {"status": "error", "step": "grab", "error": str(e)}

        # 3) move along +y by offset
        try:
            pose = await self.arm.get_end_position()
            new_pose = Pose(
                x=pose.x,
                y=pose.y + offset,
                z=pose.z,
                o_x=pose.o_x,
                o_y=pose.o_y,
                o_z=pose.o_z,
                theta=pose.theta,
            )
            await self.arm.move_to_position(new_pose)
            final = await self.arm.get_end_position()
        except Exception as e:
            return {"status": "error", "step": "move_y", "error": str(e)}

        # 4) open gripper
        try:
            await self.gripper.open()
        except Exception as e:
            return {"status": "error", "step": "open_gripper", "error": str(e)}

        return {
            "status": "ok",
            "grabbed": bool(grabbed),
            "start_pose": _pose_to_dict(pose),
            "end_pose": _pose_to_dict(final),
            "gripper_opened": True,
            "z_offset_mm": z_offset,
        }

    async def _pick_mail(self, offset: float) -> Dict[str, Any]:
        reader = self.reader_pose_switch
        gripper_pose = self.gripper_pose_switch
        if reader is None or gripper_pose is None:
            return {"status": "error", "error": "reader_pose/gripper_pose not configured"}

        # 0) open gripper first
        try:
            await self.gripper.open()
        except Exception as e:
            return {"status": "error", "step": "open_gripper", "error": str(e)}

        # 1) go to reader pose
        try:
            await reader.set_position(2)
            await self._wait_until_stopped(30.0)
        except Exception as e:
            return {"status": "error", "step": "go_to_reader_pose", "error": str(e)}

        # 2) move along x by offset
        try:
            pose = await self.arm.get_end_position()
            new_pose = Pose(
                x=pose.x + offset,
                y=pose.y,
                z=pose.z,
                o_x=pose.o_x,
                o_y=pose.o_y,
                o_z=pose.o_z,
                theta=pose.theta,
            )
            await self.arm.move_to_position(new_pose)
            await self._wait_until_stopped(30.0)
        except Exception as e:
            return {"status": "error", "step": "move_x", "error": str(e)}

        # 3) go to gripper pose
        try:
            await gripper_pose.set_position(2)
            await self._wait_until_stopped(30.0)
        except Exception as e:
            return {"status": "error", "step": "go_to_gripper_pose", "error": str(e)}

        # 4) grab
        try:
            grabbed = await self.gripper.grab()
            final = await self.arm.get_end_position()
        except Exception as e:
            return {"status": "error", "step": "grab", "error": str(e)}

        return {
            "status": "ok",
            "grabbed": bool(grabbed),
            "x_offset_mm": offset,
            "end_pose": _pose_to_dict(final),
            "gripper_opened_first": True,
        }

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        if "handle_mail" in command:
            args = command.get("handle_mail")
            offset = self.y_offset_mm
            z_offset = self.z_offset_mm
            if isinstance(args, dict) and "y_offset_mm" in args:
                try:
                    offset = float(cast(float, args["y_offset_mm"]))
                except (TypeError, ValueError):
                    return {"error": "y_offset_mm must be a number"}
            if isinstance(args, dict) and "z_offset_mm" in args:
                try:
                    z_offset = float(cast(float, args["z_offset_mm"]))
                except (TypeError, ValueError):
                    return {"error": "z_offset_mm must be a number"}
            return await self._handle_mail(offset, z_offset)
        if "pick_mail" in command:
            pargs = command.get("pick_mail")
            x_offset = self.pick_x_offset_mm
            if isinstance(pargs, dict) and "x_offset_mm" in pargs:
                try:
                    x_offset = float(cast(float, pargs["x_offset_mm"]))
                except (TypeError, ValueError):
                    return {"error": "x_offset_mm must be a number"}
            return await self._pick_mail(x_offset)
        return {"error": f"unknown command; supported: handle_mail, pick_mail. got: {list(command.keys())}"}

    async def get_status(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        return {}
