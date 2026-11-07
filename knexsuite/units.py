"""Lengths are stored in millimeters (K'nex parts are metric).
They are shown in inches or millimeters, whichever you pick.

Typing a length accepts:  128   128mm   12.8cm   5.04"   5.04in   3' 2"   1.5ft
A bare number is read in the display unit.
"""
import re

MM_PER_IN = 25.4

_UNIT = re.compile(r"^\s*(?P<num>-?(?:\d+(?:\.\d*)?|\.\d+))\s*(?P<u>mm|cm|m|in|inch|inches|\"|ft|feet|foot|')?\s*$",
                   re.IGNORECASE)
_FT_IN = re.compile(r"""^\s*(?P<sign>-)?\s*(?P<ft>\d+(?:\.\d*)?)\s*(?:'|ft|feet|foot)\s*
                        (?P<inch>\d+(?:\.\d*)?|\.\d+)\s*(?:"|in|inch|inches)?\s*$""",
                    re.IGNORECASE | re.VERBOSE)

_FACTOR = {"mm": 1.0, "cm": 10.0, "m": 1000.0, "in": MM_PER_IN, "inch": MM_PER_IN,
           "inches": MM_PER_IN, '"': MM_PER_IN, "ft": 12 * MM_PER_IN, "feet": 12 * MM_PER_IN,
           "foot": 12 * MM_PER_IN, "'": 12 * MM_PER_IN}


def parse_length(text, unit="in", default=None):
    """Return millimeters, or `default` if the text can't be read."""
    if text is None:
        return default
    t = str(text).strip()
    if not t:
        return default
    m = _FT_IN.match(t)
    if m:
        v = float(m.group("ft")) * 12 + float(m.group("inch"))
        return (-v if m.group("sign") else v) * MM_PER_IN
    m = _UNIT.match(t)
    if not m:
        return default
    num = float(m.group("num"))
    u = (m.group("u") or unit).lower()
    return num * _FACTOR.get(u, MM_PER_IN if unit == "in" else 1.0)


def _trim(x, places):
    s = f"{x:.{places}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def fmt(mm, unit="in", places=None):
    """Format millimeters in the display unit."""
    if unit == "mm":
        return f"{_trim(mm, 1 if places is None else places)} mm"
    inch = mm / MM_PER_IN
    if abs(inch) >= 24:
        sign = "-" if inch < 0 else ""
        a = abs(inch)
        ft = int(a // 12)
        rem = a - ft * 12
        if round(rem, 2) >= 12:
            ft, rem = ft + 1, 0.0
        return f"{sign}{ft}' {_trim(rem, 2)}\""
    return f"{_trim(inch, 3 if places is None else places)}\""


def fmt_both(mm, unit="in"):
    """Primary unit first, the other in brackets: 5.04" (128 mm)."""
    if unit == "mm":
        return f"{_trim(mm, 1)} mm ({_trim(mm / MM_PER_IN, 3)}\")"
    return f"{fmt(mm, 'in')} ({_trim(mm, 1)} mm)"
