import re
from typing import Any, ClassVar, List, Mapping, Optional, Sequence, Tuple, cast

from typing_extensions import Self
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict, ValueTypes
from viam.services.generic import Generic as GenericService

VALID_SECTIONS = ("sender", "receiver")
VALID_FIELDS = ("name", "address")

# ---- classify rules (edit these lists to tune the classifier) ----
# Addressee words that signal bulk mail (+2 once if any match).
SPAM_ADDRESSEE_WORDS = ["or current resident", "current resident", "resident", "occupant"]
# Bulk postage markings (+2 once if any match).
SPAM_POSTAGE_MARKS = ["prsrt std", "presorted", "standard mail", "bulk rate", "ecrwss", "postal customer"]
# Generic spam keywords (+1 each).
SPAM_KEYWORDS = [
    "free", "winner", "you've won", "congratulations", "limited time", "act now",
    "special offer", "pre-approved", "preapproved", "sweepstakes", "coupon", "sale",
    "discount", "% off", "credit card offer", "refinance", "warranty", "no obligation",
]
# Counts as spam (+1) only when the sender has no real name.
SPAM_NO_SENDER_KEYWORDS = ["final notice"]
# Important keywords (+2 each).
IMPORTANT_KEYWORDS = [
    "irs", "internal revenue", "social security", "ssa", "dmv", "court", "jury",
    "department of", "bank", "statement", "invoice", "bill", "payment due", "tax",
    "w-2", "1099", "medical", "hospital", "insurance", "pharmacy", "university",
    "school", "city of", "county", "state of", "usps certified", "certified mail",
    "return service requested", "personal", "confidential",
]
IMPORTANT_SENDERS: List[str] = []
BLOCKED_SENDERS: List[str] = []
DEFAULT_MIN_MARGIN = 2.0
MIN_TEXT_LEN = 5


def _str_list(value: Any, name: str) -> List[str]:
    if value is None:
        return []
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"attribute '{name}' must be a list of strings")
    return [str(v).strip().lower() for v in value if str(v).strip()]


def _kw_match(keyword: str, text: str) -> bool:
    """Case-insensitive match; alphanumeric edges require word boundaries (so 'ssa' doesn't match 'message')."""
    kw = keyword.strip().lower()
    if not kw or not text:
        return False
    pattern = re.escape(kw)
    if kw[0].isalnum():
        pattern = r"(?<![a-z0-9])" + pattern
    if kw[-1].isalnum():
        pattern = pattern + r"(?![a-z0-9])"
    return re.search(pattern, text) is not None


def _clean(v: Any) -> str:
    if v is None:
        return ""
    s = str(v).strip()
    return "" if s.lower() in ("none", "null") else s


# IMPORTANT: Do not change the class name or the MODEL triplet below.
# The platform uses these auto-generated values to identify your module.
# Changing them will break your inline module.
class MyGenericService(GenericService, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hackathons", "a0c3a692-38ff-4b20-9714-f549cca1af24"), "generic-service")
    reader: GenericService
    spam_keywords: List[str]
    important_keywords: List[str]
    important_senders: List[str]
    blocked_senders: List[str]
    min_margin: float

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        reader = attrs.get("reader")
        if not isinstance(reader, str) or not reader.strip():
            raise ValueError("attribute 'reader' (non-empty string, name of a generic service) is required")
        for key in ("spam_keywords", "important_keywords", "important_senders", "blocked_senders"):
            _str_list(attrs.get(key), key)
        if attrs.get("min_margin") is not None:
            try:
                mm = float(str(attrs.get("min_margin")))
            except (TypeError, ValueError):
                raise ValueError("attribute 'min_margin' must be a number")
            if mm <= 0:
                raise ValueError("attribute 'min_margin' must be > 0")
        return [reader], []

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        # viam-server calls new() on startup and after every config change.
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        reader_name: str = str(attrs["reader"])
        self.reader = cast(GenericService, dependencies[GenericService.get_resource_name(reader_name)])
        self.spam_keywords = list(SPAM_KEYWORDS) + [
            k for k in _str_list(attrs.get("spam_keywords"), "spam_keywords") if k not in SPAM_KEYWORDS
        ]
        self.important_keywords = list(IMPORTANT_KEYWORDS) + [
            k for k in _str_list(attrs.get("important_keywords"), "important_keywords") if k not in IMPORTANT_KEYWORDS
        ]
        self.important_senders = list(IMPORTANT_SENDERS) + _str_list(attrs.get("important_senders"), "important_senders")
        self.blocked_senders = list(BLOCKED_SENDERS) + _str_list(attrs.get("blocked_senders"), "blocked_senders")
        mm: Any = attrs.get("min_margin")
        try:
            self.min_margin = float(str(mm)) if mm is not None else DEFAULT_MIN_MARGIN
        except (TypeError, ValueError):
            self.min_margin = DEFAULT_MIN_MARGIN
        if self.min_margin <= 0:
            self.min_margin = DEFAULT_MIN_MARGIN
        return self

    def _rules(self) -> Mapping[str, ValueTypes]:
        return {
            "spam_addressee_words": list(SPAM_ADDRESSEE_WORDS),
            "spam_postage_marks": list(SPAM_POSTAGE_MARKS),
            "spam_keywords": list(self.spam_keywords),
            "spam_no_sender_keywords": list(SPAM_NO_SENDER_KEYWORDS),
            "important_keywords": list(self.important_keywords),
            "important_senders": list(self.important_senders),
            "blocked_senders": list(self.blocked_senders),
            "min_margin": self.min_margin,
        }

    async def _get_mail(self, mail: Any, fresh: bool) -> Tuple[Any, str, Optional[str]]:
        """Same order as 'sort': provided mail > get_last_mail (if not fresh) > read_mail."""
        if mail is not None:
            if not isinstance(mail, dict):
                return None, "", "'mail' must be an object in the reader's output format"
            return mail, "provided", None
        result: Any = None
        source = ""
        if not fresh:
            try:
                last: Any = await self.reader.do_command({"get_last_mail": {}})
                last_mail: Any = last.get("mail") if isinstance(last, Mapping) else None
                if isinstance(last, Mapping) and not last.get("error") and isinstance(last_mail, Mapping):
                    result = last_mail
                    source = "last_mail"
            except Exception:
                result = None
        if result is None:
            try:
                result = await self.reader.do_command({"read_mail": {}})
            except Exception as e:
                return None, "", f"reader call failed: {e}"
            source = "read_mail"
        if not isinstance(result, Mapping):
            return None, "", "reader returned an unexpected result"
        if result.get("error"):
            return None, "", f"reader error: {result.get('error')}"
        return result, source, None

    async def _classify(self, params: Any, command: Mapping[str, ValueTypes]) -> Mapping[str, ValueTypes]:
        if params is None:
            params = {}
        if not isinstance(params, dict):
            return {"error": "'classify' must be an object"}
        fresh: Any = params.get("fresh", True)
        if not isinstance(fresh, bool):
            return {"error": "'fresh' must be a boolean"}
        result, mail_source, err = await self._get_mail(params.get("mail", command.get("mail")), fresh)
        if err is not None:
            return {"error": err}

        sender_raw: Any = result.get("sender")
        receiver_raw: Any = result.get("receiver")
        sender: Any = dict(sender_raw) if isinstance(sender_raw, Mapping) else None
        receiver: Any = dict(receiver_raw) if isinstance(receiver_raw, Mapping) else None
        s_name = _clean(sender.get("name")) if sender else ""
        s_addr = _clean(sender.get("address")) if sender else ""
        r_name = _clean(receiver.get("name")) if receiver else ""
        r_addr = _clean(receiver.get("address")) if receiver else ""
        full_text = _clean(result.get("full_text"))

        if len(full_text) < MIN_TEXT_LEN:
            return {
                "pile": None,
                "label": "unreadable",
                "needs_review": True,
                "confidence": 0.0,
                "spam_score": 0,
                "important_score": 0,
                "reasons": ["OCR returned no usable text"],
                "sender": sender,
                "receiver": receiver,
                "mail_source": mail_source,
            }

        text = " \n ".join([s_name, s_addr, r_name, r_addr, full_text]).lower()
        sender_text = f"{s_name} {s_addr}".lower()
        spam = 0
        important = 0
        reasons: List[str] = []

        # --- spam signals ---
        hits = [w for w in SPAM_ADDRESSEE_WORDS if _kw_match(w, text)]
        if hits:
            spam += 2
            reasons.append(f"spam +2: addressee '{hits[0]}'")
        hits = [w for w in SPAM_POSTAGE_MARKS if _kw_match(w, text)]
        if hits:
            spam += 2
            reasons.append(f"spam +2: bulk postage '{hits[0]}'")
        for kw in self.spam_keywords:
            if _kw_match(kw, text):
                spam += 1
                reasons.append(f"spam +1: keyword '{kw}'")
        if not s_name:
            for kw in SPAM_NO_SENDER_KEYWORDS:
                if _kw_match(kw, text):
                    spam += 1
                    reasons.append(f"spam +1: '{kw}' without sender name")
        if not s_name and not s_addr:
            spam += 1
            reasons.append("spam +1: sender missing")
        for b in self.blocked_senders:
            if sender_text.strip() and _kw_match(b, sender_text):
                spam += 3
                reasons.append(f"spam +3: blocked sender '{b}'")
                break

        # --- important signals ---
        r_name_l = r_name.lower()
        if r_name and not any(_kw_match(w, r_name_l) for w in SPAM_ADDRESSEE_WORDS + SPAM_POSTAGE_MARKS):
            important += 1
            reasons.append("important +1: receiver has personal name")
        if r_addr:
            important += 1
            reasons.append("important +1: receiver address present")
        if s_name and s_addr:
            important += 1
            reasons.append("important +1: sender name and address present")
        for kw in self.important_keywords:
            if _kw_match(kw, text):
                important += 2
                reasons.append(f"important +2: keyword '{kw}'")
        for s in self.important_senders:
            if sender_text.strip() and _kw_match(s, sender_text):
                important += 3
                reasons.append(f"important +3: important sender '{s}'")
                break

        diff = spam - important
        if diff >= self.min_margin:
            pile, label, needs_review = "B", "spam", False
        elif -diff >= self.min_margin:
            pile, label, needs_review = "A", "important", False
        else:
            # Uncertain mail is never auto-sent to spam.
            pile, label, needs_review = "A", "uncertain", True
        confidence = min(1.0, abs(diff) / (self.min_margin * 2))

        return {
            "pile": pile,
            "label": label,
            "needs_review": needs_review,
            "confidence": round(confidence, 3),
            "spam_score": spam,
            "important_score": important,
            "reasons": reasons,
            "sender": sender,
            "receiver": receiver,
            "mail_source": mail_source,
        }

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        if "classify" in command:
            return await self._classify(command.get("classify"), command)
        if "rules" in command:
            return self._rules()
        if "sort" not in command:
            return {"error": "unknown command; expected {'sort': {...}}, {'classify': {...}} or {'rules': {}}"}
        params: Any = command.get("sort")
        if not isinstance(params, dict):
            return {"error": "'sort' must be an object"}

        section = params.get("section")
        field = params.get("field")
        condition = params.get("condition")
        if section not in VALID_SECTIONS:
            return {"error": f"'section' must be one of {list(VALID_SECTIONS)}"}
        if field not in VALID_FIELDS:
            return {"error": f"'field' must be one of {list(VALID_FIELDS)}"}
        if not isinstance(condition, str) or not condition.strip():
            return {"error": "'condition' must be a non-empty string"}

        fresh: Any = params.get("fresh", False)
        if not isinstance(fresh, bool):
            return {"error": "'fresh' must be a boolean"}

        result: Any = None
        mail_source = ""
        mail: Any = params.get("mail", command.get("mail"))
        if mail is not None:
            if not isinstance(mail, dict):
                return {"error": "'mail' must be an object in the reader's output format"}
            result = mail
            mail_source = "provided"
        else:
            if not fresh:
                try:
                    last: Any = await self.reader.do_command({"get_last_mail": {}})
                    last_mail: Any = last.get("mail") if isinstance(last, Mapping) else None
                    if not last.get("error") and isinstance(last_mail, Mapping):
                        result = last_mail
                        mail_source = "last_mail"
                except Exception:
                    result = None
            if result is None:
                try:
                    result = await self.reader.do_command({"read_mail": {}})
                except Exception as e:
                    return {"error": f"reader call failed: {e}"}
                mail_source = "read_mail"

        if not isinstance(result, Mapping):
            return {"error": "reader returned an unexpected result"}
        if result.get("error"):
            return {"error": f"reader error: {result.get('error')}"}

        section_data: Any = result.get(section)
        value: Any = section_data.get(field) if isinstance(section_data, Mapping) else None

        needle = condition.strip().lower()
        if value is not None and needle in str(value).lower():
            pile = "A"
        else:
            pile = "B"
        return {
            "pile": pile,
            "section": section,
            "field": field,
            "value": value,
            "condition": condition,
            "mail": dict(result),
            "mail_source": mail_source,
        }

    async def get_status(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        return {}
