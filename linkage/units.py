"""Length parsing and formatting in inches and feet.

Accepted input examples (all mean the same length):
    38.5      38.5"      38.5in      3' 2.5"      3ft 2.5in      3.2083'
"""
import re

_FT_IN = re.compile(
    r"""^\s*(?P<sign>-)?\s*
        (?:(?P<ft>\d+(?:\.\d*)?|\.\d+)\s*(?:'|ft|feet|foot)\s*)?
        (?:(?P<inch>\d+(?:\.\d*)?|\.\d+)\s*(?:"|in|inch|inches|'')?\s*)?$""",
    re.IGNORECASE | re.VERBOSE,
)


def parse_length(text, default=None):
    """Return inches as float, or `default` if the text can't be read."""
    if text is None:
        return default
    t = str(text).strip()
    if not t:
        return default
    m = _FT_IN.match(t)
    if not m or (m.group("ft") is None and m.group("inch") is None):
        try:
            return float(t)
        except ValueError:
            return default
    ft = float(m.group("ft")) if m.group("ft") else 0.0
    inch = float(m.group("inch")) if m.group("inch") else 0.0
    val = ft * 12.0 + inch
    return -val if m.group("sign") else val


def _trim(x, places=2):
    s = f"{x:.{places}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def fmt_in(inches, places=2):
    """Plain inches: 38.5\""""
    return f'{_trim(inches, places)}"'


def fmt_ft_in(inches, places=2):
    """Feet and inches when 12\" or longer: 3' 2.5\"   otherwise 5.25\""""
    sign = "-" if inches < 0 else ""
    a = abs(inches)
    if a < 12.0:
        return f'{sign}{_trim(a, places)}"'
    ft = int(a // 12)
    rem = a - ft * 12
    if round(rem, places) >= 12.0:
        ft += 1
        rem = 0.0
    if round(rem, places) == 0:
        return f"{sign}{ft}'"
    return f"{sign}{ft}' {_trim(rem, places)}\""
