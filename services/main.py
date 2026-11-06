import asyncio
import time
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


class StepError(Exception):
    pass


# IMPORTANT: Do not change the class name or the MODEL triplet below.
# The platform uses these auto-generated values to identify your module.
# Changing them will break your inline module.
class MyGenericService(GenericService, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hackathons", "42606b52-e245-497f-aec9-d0423cc1fcf5"), "generic-service")

    REQUIRED_ATTRS: ClassVar[Tuple[str, ...]] = (
        "arm", "gripper", "mail_handler", "reader", "sorter", "reader_pose", "bin_pose",
    )

    arm: Arm
    gripper: Gripper
    reader_pose: Switch
    bin_pose: Switch
    mail_handler: GenericService
    reader: GenericService
    sorter: GenericService

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        required_deps = []
        for key in cls.REQUIRED_ATTRS:
            val = attrs.get(key)
            if not isinstance(val, str) or not val:
                raise ValueError(f"attribute '{key}' (non-empty string) is required")
            required_deps.append(val)
        if "bin_y_offset_mm" in attrs and not isinstance(attrs["bin_y_offset_mm"], (int, float)):
            raise ValueError("attribute 'bin_y_offset_mm' must be a number")
        if "lift_before_bin_mm" in attrs and not isinstance(attrs["lift_before_bin_mm"], (int, float)):
            raise ValueError("attribute 'lift_before_bin_mm' must be a number")
        for key in ("sort_section", "sort_field", "sort_condition"):
            if key in attrs and not isinstance(attrs[key], str):
                raise ValueError(f"attribute '{key}' must be a string")
        return required_deps, []

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        # viam-server calls new() on startup and after every config change; all setup lives here.
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)

        self.arm = cast(Arm, dependencies[Arm.get_resource_name(str(attrs["arm"]))])
        self.gripper = cast(Gripper, dependencies[Gripper.get_resource_name(str(attrs["gripper"]))])
        self.reader_pose = cast(Switch, dependencies[Switch.get_resource_name(str(attrs["reader_pose"]))])
        self.bin_pose = cast(Switch, dependencies[Switch.get_resource_name(str(attrs["bin_pose"]))])
        self.mail_handler = cast(GenericService, dependencies[GenericService.get_resource_name(str(attrs["mail_handler"]))])
        self.reader = cast(GenericService, dependencies[GenericService.get_resource_name(str(attrs["reader"]))])
        self.sorter = cast(GenericService, dependencies[GenericService.get_resource_name(str(attrs["sorter"]))])

        self.bin_y_offset_mm = float(attrs.get("bin_y_offset_mm", 50) or 50)
        lift_raw = attrs.get("lift_before_bin_mm", 100)
        self.lift_before_bin_mm = float(lift_raw) if isinstance(lift_raw, (int, float)) else 100.0
        self.sort_section = str(attrs.get("sort_section") or "receiver")
        self.sort_field = str(attrs.get("sort_field") or "address")
        self.sort_condition = str(attrs.get("sort_condition") or "")
        self.group: Optional[str] = None
        return self

    async def wait_still(self, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        while True:
            if not await self.arm.is_moving():
                return
            if time.monotonic() > deadline:
                raise TimeoutError(f"arm still moving after {timeout_s}s")
            await asyncio.sleep(0.2)

    @staticmethod
    def _check_result(res: Any) -> None:
        if isinstance(res, Mapping):
            if "error" in res:
                raise StepError(str(res["error"]))
            if res.get("status") == "error":
                raise StepError(str(res.get("message", res)))

    async def _run(self, args: Mapping[str, Any]) -> Dict[str, Any]:
        section = str(args.get("section") or self.sort_section)
        field = str(args.get("field") or self.sort_field)
        condition = str(args.get("condition") or self.sort_condition)
        offset_raw = args.get("bin_y_offset_mm")
        offset = float(offset_raw) if isinstance(offset_raw, (int, float)) else self.bin_y_offset_mm
        lift_raw = args.get("lift_mm")
        lift_mm = float(lift_raw) if isinstance(lift_raw, (int, float)) else self.lift_before_bin_mm

        if not condition:
            return {"status": "error", "error": "sort condition is empty; set sort_condition attribute or pass condition"}

        steps: Dict[str, Any] = {}
        state: Dict[str, Any] = {}

        async def s_mail_handler() -> Any:
            res = await self.mail_handler.do_command({"handle_mail": {}})
            self._check_result(res)
            return res

        async def s_go_reader() -> Any:
            await self.reader_pose.set_position(2)
            await self.wait_still()
            return "ok"

        async def s_read() -> Any:
            mail = await self.reader.do_command({"read_mail": {}})
            self._check_result(mail)
            state["mail"] = mail
            return {"sender": mail.get("sender"), "receiver": mail.get("receiver")}

        async def s_sort() -> Any:
            res = await self.sorter.do_command({"sort": {
                "section": section, "field": field, "condition": condition, "mail": state["mail"],
            }})
            self._check_result(res)
            group = str(res["pile"])
            self.group = group
            state["group"] = group
            return {"pile": group}

        async def s_pick() -> Any:
            res = await self.mail_handler.do_command({"pick_mail": {}})
            self._check_result(res)
            return res

        async def s_lift_before_bin() -> Any:
            pose = await self.arm.get_end_position()
            await self.arm.move_to_position(Pose(x=pose.x, y=pose.y, z=pose.z + lift_mm,
                                                 o_x=pose.o_x, o_y=pose.o_y, o_z=pose.o_z, theta=pose.theta))
            await self.wait_still()
            return {"lift_mm": lift_mm}

        async def s_go_bin() -> Any:
            await self.bin_pose.set_position(2)
            await self.wait_still()
            return "ok"

        async def s_move_y() -> Any:
            pose = await self.arm.get_end_position()
            dy = offset if state["group"] == "A" else -offset
            target = Pose(x=pose.x, y=pose.y + dy, z=pose.z,
                          o_x=pose.o_x, o_y=pose.o_y, o_z=pose.o_z, theta=pose.theta)
            await self.arm.move_to_position(target)
            await self.wait_still()
            return {"dy": dy}

        async def s_release() -> Any:
            await self.gripper.open()
            return "ok"

        plan = [
            ("mail_handler", s_mail_handler),
            ("go_to_reader_pose", s_go_reader),
            ("read_mail", s_read),
            ("sort", s_sort),
            ("pick_mail", s_pick),
            ("lift_before_bin", s_lift_before_bin),
            ("go_to_bin_pose", s_go_bin),
            ("move_y", s_move_y),
            ("release", s_release),
        ]
        for name, fn in plan:
            try:
                steps[name] = await fn()
            except Exception as e:
                self.logger.error(f"step {name} failed: {e}")
                return {"status": "error", "step": name, "error": str(e), "steps": steps}

        mail = state.get("mail", {})
        return {
            "status": "ok",
            "group": state.get("group"),
            "mail": {"sender": mail.get("sender"), "receiver": mail.get("receiver")},
            "steps": steps,
        }

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        if "run" in command:
            args = command.get("run")
            return await self._run(args if isinstance(args, Mapping) else {})
        if "get_group" in command:
            return {"group": self.group}
        return {"error": f"unknown command; expected one of 'run', 'get_group', got {list(command.keys())}"}

    async def get_status(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        return {"group": self.group}
