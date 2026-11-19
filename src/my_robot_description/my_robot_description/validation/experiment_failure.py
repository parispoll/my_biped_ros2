"""Extract an actionable child-process failure for experiment runner errors."""

import re

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def launch_failure(path):
    """Surface the triggering exception before secondary shutdown exceptions."""
    text = ANSI.sub("", path.read_text(errors="replace"))
    errors = re.findall(
        r"(?:^|\] )([A-Za-z_][\w.]*(?:Error|Exception):[^\r\n]*)", text, re.MULTILINE
    )
    return (
        errors[0]
        if errors
        else "No Python exception found; inspect the launch log for process exit or readiness timeout."
    )
