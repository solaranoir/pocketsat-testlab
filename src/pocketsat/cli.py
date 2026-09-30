"""Command-line entry point for the ``pocketsat`` tool."""

import argparse
from collections.abc import Sequence

from pocketsat import __version__


def build_parser() -> argparse.ArgumentParser:
    """Build the top-level ``pocketsat`` argument parser."""
    parser = argparse.ArgumentParser(
        prog="pocketsat",
        description="PocketSat Test Lab: SIL/HIL spacecraft test platform.",
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {__version__}",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the ``pocketsat`` CLI.

    Args:
        argv: Arguments to parse, excluding the program name. Defaults to ``sys.argv[1:]``.

    Returns:
        Process exit code.
    """
    parser = build_parser()
    parser.parse_args(argv)
    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
