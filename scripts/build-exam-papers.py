"""Compile six exam PDFs in an isolated temporary directory.

Usage:

    python3 scripts/build-exam-papers.py --output /tmp/exam-pdfs

The script runs XeLaTeX twice for each paper and answer key. It copies the six
PDFs to the output directory only after all six compile and pass validation.
"""
import argparse
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DRAFTS = ROOT / "docs/course-drafts/exam-papers-20261027"

# The paper letter and the output name of its paper and answer key.
PAPERS = [
    ("A", "a-paper.pdf", "a-answer.pdf"),
    ("B", "b-paper.pdf", "b-answer.pdf"),
    ("C", "c-paper.pdf", "c-answer.pdf"),
]


def run(command: list[str], cwd: Path) -> str:
    """Run one command and report its output when it fails."""
    try:
        result = subprocess.run(
            command, cwd=cwd, capture_output=True, text=True, errors="replace"
        )
    except OSError as error:
        raise SystemExit(f"cannot run {command[0]}: {error}") from error
    if result.returncode != 0:
        print(result.stdout[-4000:])
        print(result.stderr[-2000:])
        raise SystemExit(
            f"command failed with exit code {result.returncode}: {' '.join(command)}"
        )
    return result.stdout


def check_log(text: str, name: str) -> None:
    """Reject a log with an error or an overfull hbox."""
    errors = len(re.findall(r"^! ", text, re.M))
    overfull = len(re.findall(r"Overfull \\hbox", text))
    if errors or overfull:
        print(f"{name}: {errors} errors, {overfull} overfull hbox")
        for line in text.splitlines():
            if line.startswith("! ") or "Overfull \\hbox" in line:
                print("   ", line[:200])
        raise SystemExit(f"{name} did not compile cleanly")


def page_count(path: Path) -> int:
    """Return the page count of one valid PDF."""
    if not path.is_file() or path.stat().st_size == 0:
        raise SystemExit(f"XeLaTeX did not create a non-empty PDF: {path.name}")
    output = run(["pdfinfo", str(path)], path.parent)
    match = re.search(r"^Pages:\s+(\d+)\s*$", output, re.M)
    if not match:
        raise SystemExit(f"pdfinfo returned no page count for {path.name}")
    return int(match.group(1))


def compile_one(work: Path, staged: Path, jobname: str, defines: str, filename: str) -> int:
    """Compile one paper or answer key and stage its validated PDF."""
    command = [
        "xelatex",
        "-interaction=nonstopmode",
        "-halt-on-error",
        f"-jobname={jobname}",
        rf"{defines}\input{{main.tex}}",
    ]
    log = ""
    for _ in range(2):
        log = run(command, work)
    check_log(log, jobname)
    built = work / f"{jobname}.pdf"
    pages = page_count(built)
    shutil.copy2(built, staged / filename)
    return pages


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the six exam PDFs.")
    parser.add_argument("--output", required=True, type=Path, help="PDF output directory")
    output = parser.parse_args().output.expanduser().resolve()
    protected = (DRAFTS, ROOT / "public")
    if any(output == path or path in output.parents for path in protected):
        raise SystemExit(f"refusing to write exam PDFs into a protected directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="exam-papers-") as temporary:
        temporary_path = Path(temporary)
        work = temporary_path / "build"
        staged = temporary_path / "output"
        shutil.copytree(
            DRAFTS,
            work,
            ignore=shutil.ignore_patterns("build", "*.pdf", "__pycache__"),
        )
        staged.mkdir()

        pages_by_file = {}
        for letter, paper, answer in PAPERS:
            pages_by_file[paper] = compile_one(
                work, staged, f"{letter}_exam", rf"\def\Paper{{{letter}}}", paper
            )
            defines = rf"\def\Paper{{{letter}}}\def\ShowAnswers{{1}}"
            pages_by_file[answer] = compile_one(
                work, staged, f"{letter}_key", defines, answer
            )

        for filename in pages_by_file:
            shutil.copy2(staged / filename, output / filename)
        for filename, pages in pages_by_file.items():
            print(f"{filename}: {pages} pages")


if __name__ == "__main__":
    main()
