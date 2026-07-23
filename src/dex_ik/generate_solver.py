"""Command-line entry point for generating the native IK solver."""

import argparse
from pathlib import Path

from .solver import DexArmIK


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate the Dex IK shared library for this platform."
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output .so on Linux or .dylib on macOS.",
    )
    parser.add_argument(
        "--conda-prefix",
        type=Path,
        help="Conda environment prefix. Defaults to CONDA_PREFIX.",
    )
    parser.add_argument("--compiler", help="C compiler command. Defaults to CC or cc.")
    args = parser.parse_args()

    solver = DexArmIK()
    output = solver.export_shared_library(
        args.output,
        conda_prefix=args.conda_prefix,
        compiler=args.compiler,
    )
    print(f"Generated Dex IK solver: {output}")


if __name__ == "__main__":
    main()
