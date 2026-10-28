"""Check the chapter coverage of the exam blueprints.

Usage:

    python3 scripts/check-exam-coverage.py

The checker reads `docs/course-drafts/exam-papers-20261027/build/blueprint.md`.
Each question row carries a chapter code and a knowledge point id in the topic
cell, in the form `〔ch5P7〕`. The checker verifies the following:

- every paper holds 25 questions worth 100 marks,
- every paper holds the 10 single-choice, 10 fill-in-the-blank, 3 essay and
  2 programming questions,
- every paper covers every chapter from 1 to 8,
- every chapter holds at least two questions in every paper,
- no two papers test the same knowledge point in the same chapter.

The knowledge point ids must exist in the chapter digests when the digests are
present. The digests live outside the repository, so the check is skipped when
they are missing.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BLUEPRINT = ROOT / "docs/course-drafts/exam-papers-20261027/build/blueprint.md"
DIGESTS = Path("/tmp/exam-src/digests")

PAPERS = ("A", "B", "C")
CHAPTERS = tuple(range(1, 9))
MARKS = {"单选": 2, "填空": 2, "问答": 10, "程序": 15}
SECTION_COUNTS = {"单选": 10, "填空": 10, "问答": 3, "程序": 2}
POINT_ID = re.compile(r"〔(ch\d)P(\d+)〕")
CHAPTER_CODE = re.compile(r"^第?\s*(\d)\s*章")
ROW = re.compile(r"^\|\s*(\d+)\s*\|")


def read_rows(section: str) -> list:
    """Read the question rows of one paper section."""
    rows = []
    for line in section.splitlines():
        if not line.startswith("|") or line.startswith("|---") or "题号" in line:
            continue
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if len(cells) < 6:
            continue
        rows.append(cells)
    return rows


def main() -> int:
    problems = []
    if not BLUEPRINT.is_file():
        print(f"missing blueprint: {BLUEPRINT}")
        return 1
    blueprint = BLUEPRINT.read_text(encoding="utf-8")

    digest_points = {}
    if DIGESTS.is_dir():
        for path in sorted(DIGESTS.glob("ch*.md")):
            chapter = int(re.search(r"ch(\d)", path.stem).group(1))
            text = path.read_text(encoding="utf-8")
            digest_points[chapter] = {
                int(number) for number in re.findall(r"(?m)^### P(\d+)", text)
            }

    used_points = {}
    for letter in PAPERS:
        section = re.search(
            rf"### {letter}卷\n(.*?)(?=\n### |\n## |\Z)", blueprint, re.S
        )
        if not section:
            problems.append(f"no section for paper {letter}")
            continue
        rows = read_rows(section.group(1))
        if len(rows) != 25:
            problems.append(f"{letter}卷 holds {len(rows)} questions, expected 25")

        total = 0
        counts = {}
        chapters = {}
        for row in rows:
            number, topic, source, kind, points, difficulty = row[:6]
            if kind not in MARKS:
                problems.append(f"{letter}{number}: unknown question type {kind!r}")
                continue
            counts[kind] = counts.get(kind, 0) + 1
            total += int(points)
            if int(points) != MARKS[kind]:
                problems.append(
                    f"{letter}{number}: {kind} carries {points} marks, expected {MARKS[kind]}"
                )

            match = POINT_ID.search(topic)
            if not match:
                problems.append(f"{letter}{number}: topic carries no knowledge point id")
                continue
            chapter = int(match.group(1)[2:])
            point = int(match.group(2))
            if chapter not in CHAPTERS:
                problems.append(f"{letter}{number}: chapter {chapter} is out of range")
                continue
            if point not in digest_points.get(chapter, set()):
                if chapter in digest_points:
                    problems.append(
                        f"{letter}{number}: knowledge point ch{chapter}P{point} is not in the digest"
                    )
            chapters.setdefault(chapter, 0)
            chapters[chapter] += 1
            used_points.setdefault((chapter, point), set()).add(letter)

        if total != 100:
            problems.append(f"{letter}卷 carries {total} marks, expected 100")
        for kind, expected in SECTION_COUNTS.items():
            if counts.get(kind, 0) != expected:
                problems.append(
                    f"{letter}卷 holds {counts.get(kind, 0)} {kind} questions, expected {expected}"
                )
        for chapter in CHAPTERS:
            if chapters.get(chapter, 0) < 2:
                problems.append(
                    f"{letter}卷 holds {chapters.get(chapter, 0)} questions in chapter {chapter}, expected 2"
                )

    for (chapter, point), letters in sorted(used_points.items()):
        if len(letters) > 1:
            problems.append(
                f"chapter {chapter} knowledge point P{point} is tested by papers {', '.join(letters)}"
            )

    if problems:
        for problem in problems:
            print(problem)
        print(f"FAILED: {len(problems)} problems")
        return 1

    print(
        f"OK: {len(PAPERS)} papers, {len(used_points)} knowledge points, "
        f"every chapter covered by every paper, no point shared between papers."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
