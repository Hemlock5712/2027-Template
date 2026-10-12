"""Writes a video's chNN-voice.json files from its brief's narration lists.

Usage: python3 make_voice.py 2-drive-to-point [question_pause_seconds, default 4.0]
"""

import json
import pathlib
import re
import sys

folder = pathlib.Path(__file__).parent / sys.argv[1]
pause = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0  # after a "Question ..." line
heading = re.compile(r"^\*\*Narration \((ch\d+)-voice\.json\):\*\*\s*$")
item = re.compile(r"^\s*(\d+)\.\s+(.*\S)\s*$")
question = re.compile(r"(^|[.!?] )Question \w+")

chapters, current = {}, None
for line in (folder / "brief.md").read_text().splitlines():
    match = heading.match(line)
    if match:
        current = match.group(1)
        chapters[current] = []
    elif current and (match := item.match(line)):
        assert int(match.group(1)) == len(chapters[current]) + 1, (current, line)
        chapters[current].append(match.group(2))
    elif current and line.strip() and chapters[current]:
        current = None  # the list ended

for old in folder.glob("ch*-voice.json"):
    old.unlink()
for name, lines in chapters.items():
    rows = []
    for number, text in enumerate(lines, 1):
        after = pause if question.search(text) else 0.3 if number == len(lines) else 0.6
        entry = {"id": f"c{name[2:]}-{number}", "text": text, "after": after}
        rows.append("    " + json.dumps(entry, ensure_ascii=False))
    (folder / f"{name}-voice.json").write_text(
        '{\n  "lead": 1.2,\n  "tail": 1.8,\n  "lines": [\n' + ",\n".join(rows) + "\n  ]\n}\n"
    )
words = sum(len(text.split()) for lines in chapters.values() for text in lines)
print(f"{folder.name}: {len(chapters)} chapters, {words} words, ~{words / 180:.1f} min")
