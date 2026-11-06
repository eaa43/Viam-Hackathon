import importlib
import logging
import os
import shutil
import site
import subprocess
import sys
from typing import Dict as _Dict
from typing import List as _List
from typing import Optional as _Optional

_REQUIRED_DEPS = [
    ("PIL", "Pillow"),
    ("numpy", "numpy"),
    ("rapidocr_onnxruntime", "rapidocr_onnxruntime"),
]
_DEP_ERRORS: _Dict[str, str] = {}


def _find_uv() -> _Optional[str]:
    found = shutil.which("uv")
    if found:
        return found
    candidates = [
        os.path.join(os.environ.get("VIAM_MODULE_DATA", ""), "bin", "uv"),
        os.path.expanduser("~/.local/bin/uv"),
    ]
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def _run(cmd: _List[str]) -> None:
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"command {cmd!r} exited {proc.returncode}: {(proc.stderr or proc.stdout or '').strip()}"
        )


def _install(pip_name: str) -> None:
    uv = _find_uv()
    if uv:
        _run([uv, "pip", "install", "--python", sys.executable, pip_name])
    else:
        _run([sys.executable, "-m", "ensurepip", "--upgrade"])
        _run([sys.executable, "-m", "pip", "install", "--quiet", pip_name])


def _refresh_paths() -> None:
    importlib.invalidate_caches()
    try:
        for p in site.getsitepackages():
            if p not in sys.path:
                sys.path.append(p)
    except Exception:  # noqa: BLE001
        pass


def _ensure_deps() -> None:
    """Install missing third-party deps at import time; never raise."""
    for import_name, pip_name in _REQUIRED_DEPS:
        try:
            importlib.import_module(import_name)
            _DEP_ERRORS.pop(import_name, None)
            continue
        except ImportError:
            pass
        try:
            _install(pip_name)
            _refresh_paths()
            importlib.import_module(import_name)
            _DEP_ERRORS.pop(import_name, None)
        except Exception as e:  # noqa: BLE001
            msg = f"failed to install {pip_name}: {e}"
            _DEP_ERRORS[import_name] = msg
            logging.getLogger(__name__).error(msg)


_ensure_deps()

import asyncio  # noqa: E402
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple  # noqa: E402

from typing_extensions import Self  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.proto.app.robot import ComponentConfig  # noqa: E402
from viam.proto.common import ResourceName  # noqa: E402
from viam.resource.base import ResourceBase  # noqa: E402
from viam.resource.easy_resource import EasyResource  # noqa: E402
from viam.resource.types import Model, ModelFamily  # noqa: E402
from viam.services.generic import Generic as GenericService  # noqa: E402
from viam.utils import ValueTypes, struct_to_dict  # noqa: E402


def _median(vals: List[float]) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


def _group_rows(items: List[Dict[str, Any]], median_h: float) -> List[List[Dict[str, Any]]]:
    """Group items into rows (top-to-bottom), each row sorted left-to-right."""
    thresh = max(median_h, 1.0) * 0.5
    rows: List[List[Dict[str, Any]]] = []
    for it in sorted(items, key=lambda i: i["yc"]):
        if rows:
            row = rows[-1]
            row_y = sum(r["yc"] for r in row) / len(row)
            if abs(it["yc"] - row_y) < thresh:
                row.append(it)
                continue
        rows.append([it])
    return [sorted(r, key=lambda i: i["left"]) for r in rows]


# IMPORTANT: Do not change the class name or the MODEL triplet below.
# The platform uses these auto-generated values to identify your module.
# Changing them will break your inline module.
class MyGenericService(GenericService, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hackathons", "404121c6-ed45-489c-8f7d-eba4bb8e99b1"), "generic-service")
    camera: Camera
    _ocr: Any = None
    _ocr_lock: asyncio.Lock
    min_score: float = 0.3
    last_mail: Optional[Dict[str, Any]] = None

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        cam = attrs.get("camera")
        if not isinstance(cam, str) or not cam:
            raise ValueError("attribute 'camera' (non-empty string) is required")
        return [cam], []

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        camera_name: str = str(attrs["camera"])
        self.camera = dependencies[Camera.get_resource_name(camera_name)]  # type: ignore[assignment]
        raw_min = attrs.get("min_score", 0.3)
        try:
            self.min_score = float(str(raw_min))
        except (TypeError, ValueError):
            self.min_score = 0.3
        self._ocr = None
        self._ocr_lock = asyncio.Lock()
        self.last_mail = None
        return self

    async def _get_ocr(self) -> Any:
        async with self._ocr_lock:
            if self._ocr is None:
                from rapidocr_onnxruntime import RapidOCR  # type: ignore[import-not-found,import-untyped]
                self._ocr = await asyncio.to_thread(RapidOCR)
        return self._ocr

    @staticmethod
    def _block_lines(block: List[Dict[str, Any]], median_h: float) -> List[str]:
        lines: List[str] = []
        for row in _group_rows(block, median_h):
            t = " ".join(it["text"] for it in row).strip()
            if t:
                lines.append(t)
        return lines

    @staticmethod
    def _build_party(lines: List[str]) -> Dict[str, Optional[str]]:
        name: Optional[str] = lines[0] if lines else None
        rest = lines[1:]
        address: Optional[str] = ", ".join(rest) if rest else None
        return {"name": name, "address": address}

    def _parse(self, items: List[Dict[str, Any]], width: int, height: int) -> Dict[str, Any]:
        good = [it for it in items if it["score"] >= self.min_score]
        median_h = _median([it["h"] for it in good]) or 1.0
        gap_max = 1.5 * median_h
        left_tol = 0.25 * width

        # Group lines into blocks, checking every open block (not just the last).
        blocks: List[Dict[str, Any]] = []
        for it in sorted(good, key=lambda i: i["top"]):
            best: Optional[Dict[str, Any]] = None
            best_gap = float("inf")
            for b in blocks:
                gap = it["top"] - b["bottom"]
                if gap <= gap_max and abs(it["left"] - b["left"]) <= left_tol:
                    if gap < best_gap:
                        best, best_gap = b, gap
            if best is not None:
                best["items"].append(it)
                best["bottom"] = max(best["bottom"], it["bottom"])
                best["left"] = min(best["left"], it["left"])
            else:
                blocks.append({
                    "items": [it],
                    "top": it["top"],
                    "bottom": it["bottom"],
                    "left": it["left"],
                })
        for b in blocks:
            b["lines"] = self._block_lines(b["items"], median_h)

        sender_block: Optional[Dict[str, Any]] = None
        left_blocks = [b for b in blocks if b["left"] < 0.5 * width]
        if left_blocks:
            sender_block = min(left_blocks, key=lambda b: b["top"])

        receiver_block: Optional[Dict[str, Any]] = None
        if sender_block is not None:
            below = [
                b for b in blocks
                if b is not sender_block and b["top"] >= sender_block["bottom"]
            ]
            multi = [b for b in below if len(b["lines"]) >= 2]
            pool = multi or below
            if pool:
                receiver_block = min(pool, key=lambda b: b["top"])

        sender = self._build_party(sender_block["lines"] if sender_block else [])
        receiver = self._build_party(receiver_block["lines"] if receiver_block else [])
        return {"sender": sender, "receiver": receiver}

    @staticmethod
    def _envelope_crop(arr: Any) -> Any:
        """Crop the bright envelope from a darker background (Otsu threshold)."""
        import numpy as np

        height, width = arr.shape[0], arr.shape[1]
        if height < 2 or width < 2:
            return arr
        rgb = arr.astype(np.float64)
        gray = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]

        hist, _ = np.histogram(gray, bins=256, range=(0, 256))
        hist = hist.astype(np.float64)
        total = hist.sum()
        if total <= 0:
            return arr
        levels = np.arange(256, dtype=np.float64)
        w0 = np.cumsum(hist)
        w1 = total - w0
        sum0 = np.cumsum(hist * levels)
        sum_total = sum0[-1]
        with np.errstate(divide="ignore", invalid="ignore"):
            mean0 = sum0 / w0
            mean1 = (sum_total - sum0) / w1
            between = w0 * w1 * (mean0 - mean1) ** 2
        between = np.nan_to_num(between, nan=0.0, posinf=0.0, neginf=0.0)
        threshold = int(np.argmax(between))

        mask = gray > threshold
        row_ok = mask.mean(axis=1) > 0.3
        col_ok = mask.mean(axis=0) > 0.3

        def longest_run(flags: Any) -> Optional[Tuple[int, int]]:
            best: Optional[Tuple[int, int]] = None
            start = -1
            for i, f in enumerate(list(flags) + [False]):
                if f and start < 0:
                    start = i
                elif not f and start >= 0:
                    if best is None or (i - start) > (best[1] - best[0] + 1):
                        best = (start, i - 1)
                    start = -1
            return best

        rows = longest_run(row_ok)
        cols = longest_run(col_ok)
        if rows is None or cols is None:
            return arr
        y0, y1 = rows
        x0, x1 = cols
        box_h = y1 - y0 + 1
        box_w = x1 - x0 + 1
        frac = (box_h * box_w) / float(height * width)
        if frac < 0.05 or frac > 0.98:
            return arr
        my = int(round(0.01 * box_h))
        mx = int(round(0.01 * box_w))
        y0, y1 = y0 + my, y1 - my
        x0, x1 = x0 + mx, x1 - mx
        if y1 <= y0 or x1 <= x0:
            return arr
        return np.ascontiguousarray(arr[y0:y1 + 1, x0:x1 + 1])

    async def _read_mail(self) -> Dict[str, Any]:
        if _DEP_ERRORS:
            await asyncio.to_thread(_ensure_deps)
        if _DEP_ERRORS:
            return {"error": "missing dependencies: " + "; ".join(_DEP_ERRORS.values())}

        import io

        import numpy as np
        from PIL import Image as PILImage
        from viam.media.utils.pil import viam_to_pil_image
        from viam.media.video import NamedImage, ViamImage

        images, _ = await self.camera.get_images()
        candidates = [
            im for im in (images or [])
            if "dep" not in str(getattr(im, "mime_type", "") or "").lower()
        ]
        if not candidates:
            return {"error": "no color image from camera"}
        named_image = next(
            (im for im in candidates if "color" in str(getattr(im, "name", "") or "").lower()),
            candidates[0],
        )

        if issubclass(NamedImage, ViamImage):
            pil_img = viam_to_pil_image(named_image)
        else:
            pil_img = PILImage.open(io.BytesIO(named_image.data))
        full_arr = np.array(pil_img.convert("RGB"))
        arr = await asyncio.to_thread(self._envelope_crop, full_arr)
        height, width = arr.shape[0], arr.shape[1]
        try:
            ocr = await self._get_ocr()
            result, _ = await asyncio.to_thread(ocr, arr)
        except Exception as e:  # noqa: BLE001
            return {"error": f"OCR failed: {e}"}

        items: List[Dict[str, Any]] = []
        for entry in result or []:
            box, text, score = entry[0], entry[1], float(entry[2])
            text = str(text).strip()
            if not text:
                continue
            xs = [float(p[0]) for p in box]
            ys = [float(p[1]) for p in box]
            items.append({
                "text": text,
                "score": score,
                "yc": sum(ys) / len(ys),
                "left": min(xs),
                "top": min(ys),
                "bottom": max(ys),
                "h": max(ys) - min(ys),
            })

        all_median_h = _median([it["h"] for it in items]) or 1.0
        row_texts = [
            " ".join(it["text"] for it in row)
            for row in _group_rows(items, all_median_h)
        ]

        parsed = self._parse(items, width, height)
        return {
            "full_text": "\n".join(row_texts),
            "sender": parsed["sender"],
            "receiver": parsed["receiver"],
        }

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        if "read_mail" in command:
            try:
                result = await self._read_mail()
            except Exception as e:
                self.logger.error(f"read_mail failed: {e}")
                return {"error": str(e)}
            if "error" not in result:
                self.last_mail = result
            return result
        if "get_last_mail" in command:
            if self.last_mail is not None:
                return {"mail": self.last_mail}
            return {"error": "no mail has been read yet"}
        return {"error": f"unknown command; supported: read_mail, get_last_mail. got: {list(command.keys())}"}

    async def get_status(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        return {}
