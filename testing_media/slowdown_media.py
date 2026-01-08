#!/usr/bin/env python3
"""
Create 0.5x slowed-down copies of all .mp4 files in a directory.

Outputs are written alongside the originals with a `_0.5x` suffix.

Behavior:
- Skips files that already have a `_0.5x` suffix
- Skips if output exists unless --overwrite is provided
- Preserves codecs sensibly (H.264 + AAC by default)
- Handles files with and without audio tracks
- Attempts to install ffmpeg on Debian/Ubuntu if missing (best-effort)
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def is_tool_available(tool_name: str) -> bool:
    return shutil.which(tool_name) is not None


def try_install_ffmpeg() -> None:
    if is_tool_available("ffmpeg") and is_tool_available("ffprobe"):
        return
    print("ffmpeg not found. Attempting to install via apt...", flush=True)
    try:
        # Update package lists (non-interactive)
        subprocess.run(
            [
                "sudo",
                "-n",
                "apt-get",
                "update",
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        subprocess.run(
            [
                "sudo",
                "-n",
                "apt-get",
                "install",
                "-y",
                "ffmpeg",
            ],
            check=False,
        )
    except Exception:
        pass  # Best-effort only


def has_audio_track(input_path: Path) -> bool:
    try:
        result = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "a",
                "-show_entries",
                "stream=index",
                "-of",
                "csv=p=0",
                str(input_path),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        return result.stdout.strip() != ""
    except Exception:
        return False


def build_output_path(input_path: Path, suffix: str = "_0.5x") -> Path:
    return input_path.with_name(f"{input_path.stem}{suffix}{input_path.suffix}")


def slow_down_video_0_5x(input_path: Path, output_path: Path, overwrite: bool) -> int:
    audio_present = has_audio_track(input_path)

    if audio_present:
        # setpts=2.0 slows video to 0.5x; atempo=0.5 slows audio to 0.5x
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y" if overwrite else "-n",
            "-i",
            str(input_path),
            "-filter_complex",
            "[0:v]setpts=2.0*PTS[v];[0:a]atempo=0.5[a]",
            "-map",
            "[v]",
            "-map",
            "[a]",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "18",
            "-c:a",
            "aac",
            "-b:a",
            "192k",
            str(output_path),
        ]
    else:
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y" if overwrite else "-n",
            "-i",
            str(input_path),
            "-filter:v",
            "setpts=2.0*PTS",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "18",
            "-an",
            str(output_path),
        ]

    proc = subprocess.run(cmd)
    return proc.returncode


def find_mp4_files(directory: Path) -> list[Path]:
    return sorted([p for p in directory.glob("*.mp4") if p.is_file()])


def main() -> int:
    parser = argparse.ArgumentParser(description="Create 0.5x slowed versions of .mp4 files")
    parser.add_argument(
        "--media_dir",
        default="/home/lorenzo/computer_vision/testing_media",
        help="Directory containing source .mp4 files",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite outputs if they already exist",
    )
    parser.add_argument(
        "--suffix",
        default="_0.5x",
        help="Suffix to append to slowed files before extension",
    )
    args = parser.parse_args()

    try_install_ffmpeg()
    if not (is_tool_available("ffmpeg") and is_tool_available("ffprobe")):
        print(
            "Error: ffmpeg/ffprobe not available. Install ffmpeg and try again.",
            file=sys.stderr,
        )
        return 1

    media_dir = Path(args.media_dir)
    if not media_dir.exists() or not media_dir.is_dir():
        print(f"Error: media_dir not found or not a directory: {media_dir}", file=sys.stderr)
        return 1

    mp4_files = find_mp4_files(media_dir)
    if not mp4_files:
        print(f"No .mp4 files found in {media_dir}")
        return 0

    processed = 0
    skipped = 0
    failures = 0

    print(f"Found {len(mp4_files)} .mp4 files in {media_dir}")
    for input_path in mp4_files:
        # Skip already-slowed outputs
        if input_path.stem.endswith(args.suffix):
            print(f"Skip (already slowed): {input_path.name}")
            skipped += 1
            continue

        output_path = build_output_path(input_path, args.suffix)
        if output_path.exists() and not args.overwrite:
            print(f"Skip (exists): {output_path.name}")
            skipped += 1
            continue

        print(f"Slowing to 0.5x: {input_path.name} -> {output_path.name}")
        rc = slow_down_video_0_5x(input_path, output_path, overwrite=args.overwrite)
        if rc == 0:
            processed += 1
        else:
            print(f"Failed: {input_path.name}", file=sys.stderr)
            failures += 1

    print(
        f"Done. processed={processed}, skipped={skipped}, failures={failures}. Output suffix='{args.suffix}'."
    )
    return 0 if failures == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())


