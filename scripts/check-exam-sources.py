"""Check source page and paragraph references in the exam blueprints.

Usage:

    python3 scripts/check-exam-sources.py
    python3 scripts/check-exam-sources.py --extracts /path/to/pdf-text-extracts

The checker reads PDF page markers from the 11 text extracts. It checks DOCX
paragraph references against the counts in extraction-notes.md because no DOCX
text extracts exist in the current extraction directory.
"""
import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DRAFTS = ROOT / "docs/course-drafts/exam-papers-20261027/build"
DEFAULT_EXTRACTS = Path("/tmp/exam-src/files")
BLUEPRINT = DRAFTS / "blueprint.md"
EXTRACTION_NOTES = DRAFTS / "extraction-notes.md"

PDF_SOURCES = {
    "S01": "第一章 无人驾驶概述_2026",
    "S02": "第七章 无人驾驶控制理论",
    "S03": "第三章 高精度地图",
    "S04": "第二章 无人驾驶感知基础",
    "S05": "第二章-行车环境感知-激光雷达和点云",
    "S06": "第五章 无人驾驶预测系统",
    "S07": "第五章 无人驾驶预测系统实例",
    "S08": "第八章 强化深度学习案例",
    "S09": "第八章 无人驾驶强化学习",
    "S10": "第四章 无人驾驶定位系统2",
    "S11": "第四章-汽车智能驾驶定位解决方案",
}
DOCX_CODES = {"H1", "H2", "H3", "H4"}
PAGE_MARKER = re.compile(r"^=== .* p(\d+) ===$", re.M)
SOURCE_TOKEN = re.compile(r"^(S\d{2}|H\d):([0-9,-]+)$")
PAGE_RANGE = re.compile(r"^(\d+)(?:-(\d+))?$")
DOCX_COUNT = re.compile(r"^- (H[1-4])：.*，(\d+)个非空段落。$", re.M)



def cited_numbers(specification: str) -> list[int]:
    """Expand comma-separated page numbers and ranges, or reject bad syntax."""
    numbers = []
    for item in specification.split(","):
        match = PAGE_RANGE.fullmatch(item)
        if not match:
            raise ValueError(f"invalid page or paragraph range {item!r}")
        first = int(match.group(1))
        last = int(match.group(2) or first)
        if first < 1 or last < first:
            raise ValueError(f"invalid page or paragraph range {item!r}")
        numbers.extend(range(first, last + 1))
    return numbers


def main() -> int:
    parser = argparse.ArgumentParser(description="Check exam blueprint source references.")
    parser.add_argument(
        "--extracts",
        type=Path,
        default=DEFAULT_EXTRACTS,
        help=f"directory with the 11 PDF text extracts (default: {DEFAULT_EXTRACTS})",
    )
    extracts = parser.parse_args().extracts.expanduser().resolve()

    if not BLUEPRINT.is_file():
        print(f"missing blueprint: {BLUEPRINT}")
        return 1
    if not EXTRACTION_NOTES.is_file():
        print(f"missing extraction notes: {EXTRACTION_NOTES}")
        return 1
    if not extracts.is_dir():
        print(f"missing PDF text extract directory: {extracts}")
        return 1

    problems = []
    available_pages = {}
    for code, stem in PDF_SOURCES.items():
        path = extracts / f"{stem}.txt"
        if not path.is_file():
            problems.append(f"missing PDF text extract for {code}: {path}")
            continue
        page_set = {int(number) for number in PAGE_MARKER.findall(path.read_text(encoding="utf-8"))}
        if not page_set:
            problems.append(f"no PDF page markers found for {code}: {path}")
            continue
        available_pages[code] = page_set

    notes = EXTRACTION_NOTES.read_text(encoding="utf-8")
    docx_counts = {code: int(count) for code, count in DOCX_COUNT.findall(notes)}
    for code in sorted(DOCX_CODES - docx_counts.keys()):
        problems.append(f"missing non-empty paragraph count for {code}: {EXTRACTION_NOTES}")

    blueprint = BLUEPRINT.read_text(encoding="utf-8")
    checked = 0
    for letter in ("A", "B", "C"):
        section = re.search(rf"### {letter}卷\n(.*?)(?=\n### |\n## |\Z)", blueprint, re.S)
        if not section:
            problems.append(f"no section for paper {letter}")
            continue
        for line_number, line in enumerate(section.group(1).splitlines(), start=1):
            if not line.startswith("|") or line.startswith("|---") or "题号" in line:
                continue
            cells = [cell.strip() for cell in line.strip("|").split("|")]
            if len(cells) < 3:
                problems.append(f"{letter}卷 table row {line_number}: missing source column")
                continue
            question, source = cells[0], cells[2]
            if not source:
                problems.append(f"{letter}{question}: empty source reference")
                continue
            for token in source.split(chr(59)):
                token = token.strip()
                match = SOURCE_TOKEN.fullmatch(token)
                if not match:
                    problems.append(f"{letter}{question}: unreadable source token {token!r}")
                    continue
                code, specification = match.groups()
                if code in available_pages:
                    valid_numbers = available_pages[code]
                elif code in docx_counts:
                    valid_numbers = set(range(1, docx_counts[code] + 1))
                elif code in PDF_SOURCES or code in DOCX_CODES:
                    continue
                else:
                    problems.append(f"{letter}{question}: unknown source code {code}")
                    continue
                try:
                    numbers = cited_numbers(specification)
                except ValueError as error:
                    problems.append(f"{letter}{question}: {error}")
                    continue
                for number in numbers:
                    if number not in valid_numbers:
                        problems.append(f"{letter}{question}: {code} has no page or paragraph {number}")
                    else:
                        checked += 1

    if problems:
        for problem in problems:
            print(problem)
        print(f"FAILED: {len(problems)} problems, {checked} source references checked")
        return 1

    print(
        f"OK: {checked} source page and paragraph references checked. "
        "All PDF extracts and cited ranges are valid."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
