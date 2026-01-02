#!/usr/bin/env python3
"""
Generate a minimal compile_commands.json.

What this does:
- Scans source directories for .cpp/.cc files
- Emits compile commands that use clang++ with common include paths

Usage:
  python scripts/gen_compile_commands.py
  (Run from the project root)
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import pathlib
import sys

PROJECT_ROOT = pathlib.Path.cwd()
DEFAULT_REL_SRC_DIRS = ["./"]
DEFAULT_ABS_SRC_DIRS = []
OUTPUT_FILE = "compile_commands.json"
EXCLUDE_PATTERNS: list[str] = []

# Include paths for the project
INCLUDE_PATHS = [
    # OpenCV include paths
    "C:/opencv/install/include/",
    "C:/opencv/install_11.2/include/",
    # Workspace include paths
    str(PROJECT_ROOT),
    str(PROJECT_ROOT / "libs" / "json" / "include" / "nlohmann"),
    str(PROJECT_ROOT / "libs" / "IMGUI"),
    # Spinnaker include paths (may be either one of these)
    "C:/Program Files/FLIR Systems/Spinnaker/include/",
    "C:/Program Files/Teledyne/Spinnaker/include",
]


def is_source_file(p: pathlib.Path) -> bool:
    if not p.is_file():
        return False
    ext = p.suffix.lower()
    return ext in {".cpp", ".cc", ".cxx"}


def gather_sources(root: pathlib.Path, src_dirs: list[str], recursive: bool = False) -> list[pathlib.Path]:
    """
    Gather source files from specified directories.
    
    Args:
        root: Root directory for relative paths
        src_dirs: List of relative or absolute directory paths
        recursive: If True, search recursively; if False, only search one level deep
    """
    files: list[pathlib.Path] = []
    for rel in src_dirs:
        # Handle both relative and absolute paths
        if pathlib.Path(rel).is_absolute():
            base = pathlib.Path(rel)
        else:
            base = root / rel
        
        if not base.exists():
            continue
        if not base.is_dir():
            # If it's a file, add it directly if it's a source file
            if is_source_file(base):
                files.append(base)
            continue
        
        if recursive:
            # Recursive search
            for p in base.rglob("*"):
                if is_source_file(p):
                    files.append(p)
        else:
            # Only search one level deep
            for p in base.iterdir():
                if is_source_file(p):
                    files.append(p)
    return files


def should_exclude_file(file_path: pathlib.Path, exclude_patterns: list[str]) -> bool:
    """
    Check if a file should be excluded based on glob patterns.
    Patterns can match against the filename or the full path relative to PROJECT_ROOT.
    """
    if not exclude_patterns:
        return False
    
    # Try matching against filename
    filename = file_path.name
    for pattern in exclude_patterns:
        if fnmatch.fnmatch(filename, pattern):
            return True
    
    # Try matching against path relative to PROJECT_ROOT
    try:
        rel_path = file_path.relative_to(PROJECT_ROOT)
        rel_path_str = str(rel_path).replace("\\", "/")  # Normalize separators
        for pattern in exclude_patterns:
            if fnmatch.fnmatch(rel_path_str, pattern):
                return True
            # Also try matching with **/ prefix for recursive patterns
            if fnmatch.fnmatch(rel_path_str, f"**/{pattern}"):
                return True
    except ValueError:
        # File is not relative to PROJECT_ROOT, try absolute path
        abs_path_str = str(file_path).replace("\\", "/")
        for pattern in exclude_patterns:
            if fnmatch.fnmatch(abs_path_str, pattern):
                return True
    
    return False


def build_command(file_path: pathlib.Path) -> str:
    """
    Build a minimal clang++ command for clangd indexing.
    We don't attempt to mirror MSVC flags – we just provide enough for parsing.
    """
    parts = [
        "clang++",
        "-std=c++17",
        "-x", "c++",
        "-fms-compatibility",
        "-fms-compatibility-version=19.3",
        "-fvisibility=hidden",
        "-DUNICODE",
        "-DNDEBUG",
        "-DWIN32_LEAN_AND_MEAN",
    ]

    # Add include paths (only include existing paths)
    for inc_path in INCLUDE_PATHS:
        inc = pathlib.Path(inc_path)
        if inc.exists():
            parts += ["-I", str(inc)]

    # Prevent warnings from breaking clangd parsing
    parts += ["-w", "-c", str(file_path)]
    return " ".join(parts)


def make_entry(file_path: pathlib.Path) -> dict:
    directory = str(file_path.parent.resolve())
    return {
        "directory": directory,
        "file": str(file_path.resolve()),
        "command": build_command(file_path),
    }


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Generate compile_commands.json")
    parser.add_argument(
        "--src-dir",
        action="append",
        dest="src_dirs",
        help="Additional source directory to include (can be passed multiple times)",
    )
    parser.add_argument(
        "--abs-src-dir",
        action="append",
        dest="abs_src_dirs",
        help="Additional ABSOLUTE source directory to include (can be passed multiple times)",
    )
    parser.add_argument(
        "--out",
        default=str(PROJECT_ROOT / OUTPUT_FILE),
        help="Output path for compile_commands.json",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search directories recursively (default: only one level deep)",
    )
    args = parser.parse_args(argv)

    # Merge defaults and CLI-specified dirs; absolute paths are supported
    src_dirs = DEFAULT_REL_SRC_DIRS[:] + DEFAULT_ABS_SRC_DIRS[:]
    if args.src_dirs:
        src_dirs.extend(args.src_dirs)
    if args.abs_src_dirs:
        src_dirs.extend(args.abs_src_dirs)

    sources = gather_sources(PROJECT_ROOT, src_dirs, recursive=args.recursive)
    if not sources:
        print("No source files found. Check your working directory.", file=sys.stderr)
        return 1

    # Filter out excluded files
    filtered_sources = [p for p in sources if not should_exclude_file(p, EXCLUDE_PATTERNS)]
    
    if EXCLUDE_PATTERNS and len(filtered_sources) < len(sources):
        excluded_count = len(sources) - len(filtered_sources)
        print(f"Excluded {excluded_count} file(s) matching exclude patterns", file=sys.stderr)

    entries = [make_entry(p) for p in filtered_sources]
    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(entries, f, indent=2)
        f.write("\n")

    print(f"Wrote {len(entries)} entries to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))


