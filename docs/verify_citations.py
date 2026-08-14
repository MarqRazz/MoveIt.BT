#!/usr/bin/env python3
"""Verify the file:line citations in this repository's docs against the source.

A guide full of `stage.h:360` citations is only worth what its line numbers are
worth. This script pins each citation to the *text* that was at that line when the
guide was last verified, and tells you what moved after an upstream sync.

    ./docs/verify_citations.py                  # check, exit 1 if anything drifted
    ./docs/verify_citations.py --update         # re-pin to the current tree
    ./docs/verify_citations.py --guide X.md     # a different guide

Citations are markdown inline code of the form `path:spec`, where spec is a
comma-separated list of line numbers and ranges: `move_relative.cpp:229,261,288`
and `storage.h:269-349` are both citations. Paths may be repo-relative or a
unique suffix (`stage.h`, `stages/move_to.cpp`).
"""

import argparse
import json
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_GUIDE = os.path.join(REPO, "docs", "authoring_guide.md")
DEFAULT_LOCK = os.path.join(REPO, "docs", "citations.lock.json")

SOURCE_SUFFIXES = (".h", ".hpp", ".cpp", ".py", ".rst", ".xml", ".yaml", ".txt")
SKIP_DIRS = {".git", "build", "install", "log", "pybind11"}

CITATION = re.compile(
    r"`([A-Za-z0-9_./]+\.(?:h|hpp|cpp|py|rst|xml|yaml|txt)):((?:\d+(?:-\d+)?)(?:,\d+(?:-\d+)?)*)`")


def build_index():
    """Map every path suffix to the repo-relative files it could mean."""
    index = {}
    for dirpath, dirnames, filenames in os.walk(REPO):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for name in filenames:
            if not name.endswith(SOURCE_SUFFIXES):
                continue
            rel = os.path.relpath(os.path.join(dirpath, name), REPO)
            parts = rel.split(os.sep)
            for i in range(len(parts)):
                index.setdefault("/".join(parts[i:]), set()).add(rel)
    return index


def resolve(path, index):
    """Resolve a cited path to one repo-relative file, or None with a reason."""
    candidates = sorted(index.get(path, ()))
    if not candidates:
        return None, "no such file in the repository"
    if len(candidates) == 1:
        return candidates[0], None
    # The python bindings mirror core header names; the C++ library is what the
    # guides cite, so prefer it and only complain if that is still ambiguous.
    preferred = [c for c in candidates if c.startswith("core/") and "/python/" not in c]
    if len(preferred) == 1:
        return preferred[0], None
    return None, "ambiguous, matches " + ", ".join(candidates)


def endpoints(spec):
    """The individual line numbers a spec pins: every range endpoint."""
    out = []
    for item in spec.split(","):
        if "-" in item:
            lo, hi = (int(x) for x in item.split("-"))
            out.extend([lo, hi] if hi != lo else [lo])
        else:
            out.append(int(item))
    return out


def read_lines(rel):
    with open(os.path.join(REPO, rel), encoding="utf-8") as handle:
        return handle.read().split("\n")


def collect(guide, index):
    """Every citation in the guide, resolved, in document order."""
    with open(guide, encoding="utf-8") as handle:
        text = handle.read()
    seen = {}
    for md_line, line in enumerate(text.split("\n"), 1):
        for match in CITATION.finditer(line):
            path, spec = match.group(1), match.group(2)
            key = f"{path}:{spec}"
            if key in seen:
                continue
            rel, problem = resolve(path, index)
            seen[key] = dict(key=key, path=path, spec=spec, file=rel,
                             problem=problem, md_line=md_line)
    return list(seen.values())


def snapshot(cite):
    """The text currently at each pinned line, or None if out of range."""
    lines = read_lines(cite["file"])
    pinned = {}
    for number in endpoints(cite["spec"]):
        pinned[str(number)] = lines[number - 1].strip() if 0 < number <= len(lines) else None
    return pinned


_commit = None


def current_commit():
    """Short HEAD, or "unknown" (with the reason on stderr) if git will not say."""
    global _commit
    if _commit is None:
        try:
            out = subprocess.run(["git", "-C", REPO, "rev-parse", "--short", "HEAD"],
                                 capture_output=True, text=True, check=True)
            _commit = out.stdout.strip()
        except FileNotFoundError:
            print("warning: git not found; recording commit as 'unknown'", file=sys.stderr)
            _commit = "unknown"
        except subprocess.CalledProcessError as exc:
            reason = exc.stderr.strip().split("\n")[0] if exc.stderr else "git rev-parse failed"
            print(f"warning: {reason}; recording commit as 'unknown'", file=sys.stderr)
            _commit = "unknown"
    return _commit


def do_update(cites, lock_path):
    entries = {}
    broken = []
    for cite in cites:
        if cite["problem"]:
            broken.append(cite)
            continue
        pinned = snapshot(cite)
        if any(text is None for text in pinned.values()):
            broken.append(dict(cite, problem="line number past end of file"))
            continue
        entries[cite["key"]] = dict(file=cite["file"], lines=pinned)
    if broken:
        for cite in broken:
            print(f"cannot pin `{cite['key']}` (guide line {cite['md_line']}): {cite['problem']}")
        print(f"\n{len(broken)} citation(s) could not be pinned; fix them and re-run --update.")
        return 1
    with open(lock_path, "w", encoding="utf-8") as handle:
        json.dump(dict(commit=current_commit(), entries=entries), handle, indent=1, sort_keys=True)
        handle.write("\n")
    total = sum(len(e["lines"]) for e in entries.values())
    print(f"pinned {len(entries)} citations ({total} lines) at {current_commit()} -> {os.path.relpath(lock_path, REPO)}")
    return 0


def do_check(cites, lock_path):
    if not os.path.exists(lock_path):
        print(f"no lock file at {os.path.relpath(lock_path, REPO)}; run with --update first.")
        return 1
    with open(lock_path, encoding="utf-8") as handle:
        lock = json.load(handle)
    entries = lock.get("entries", {})

    drifted, unpinned, unresolved = [], [], []
    for cite in cites:
        if cite["problem"]:
            unresolved.append(cite)
            continue
        entry = entries.get(cite["key"])
        if entry is None:
            unpinned.append(cite)
            continue
        lines = read_lines(cite["file"])
        for number, expected in sorted(entry["lines"].items(), key=lambda kv: int(kv[0])):
            actual = lines[int(number) - 1].strip() if 0 < int(number) <= len(lines) else None
            if actual == expected:
                continue
            moved = [i + 1 for i, text in enumerate(lines) if text.strip() == expected]
            drifted.append(dict(cite=cite, line=number, expected=expected,
                                actual=actual, moved=moved))

    stale = sorted(set(entries) - {c["key"] for c in cites})

    for item in drifted:
        cite = item["cite"]
        print(f"DRIFT  `{cite['key']}`  ({cite['file']} line {item['line']}, guide line {cite['md_line']})")
        print(f"         pinned: {item['expected']}")
        print(f"         now:    {item['actual'] if item['actual'] is not None else '<past end of file>'}")
        if len(item["moved"]) == 1:
            print(f"         -> that text is now at line {item['moved'][0]}; update the citation.")
        elif item["moved"]:
            print(f"         -> that text now appears at lines {item['moved']}; disambiguate by hand.")
        else:
            print("         -> that text is gone; the claim itself needs re-checking.")
    for cite in unresolved:
        print(f"UNRESOLVED  `{cite['key']}` (guide line {cite['md_line']}): {cite['problem']}")
    for cite in unpinned:
        print(f"UNPINNED    `{cite['key']}` (guide line {cite['md_line']}): verify it, then re-run --update.")
    for key in stale:
        print(f"STALE       `{key}` is pinned but no longer cited; --update will drop it.")

    checked = sum(len(entries[c["key"]]["lines"]) for c in cites if c["key"] in entries)
    problems = len(drifted) + len(unresolved) + len(unpinned)
    print(f"\n{len(cites)} citations, {checked} lines checked against {lock.get('commit', '?')}; "
          f"now at {current_commit()}.")
    if problems:
        print(f"{problems} problem(s). Fix the guide, then re-run with --update.")
        return 1
    print("all citations verify." + (f" ({len(stale)} stale lock entries)" if stale else ""))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--guide", default=DEFAULT_GUIDE, help="markdown file to check")
    parser.add_argument("--lock", default=DEFAULT_LOCK, help="lock file of pinned line contents")
    parser.add_argument("--update", action="store_true", help="re-pin every citation to the current tree")
    args = parser.parse_args()

    if not os.path.exists(args.guide):
        print(f"no guide at {args.guide}")
        return 1
    cites = collect(args.guide, build_index())
    if not cites:
        print(f"no citations found in {args.guide}")
        return 0
    return do_update(cites, args.lock) if args.update else do_check(cites, args.lock)


if __name__ == "__main__":
    sys.exit(main())
