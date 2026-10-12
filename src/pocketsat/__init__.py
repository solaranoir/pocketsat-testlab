"""PocketSat Test Lab: a SIL/HIL test platform for a simulated spacecraft."""

from importlib.metadata import PackageNotFoundError, version

__version__: str
try:
    __version__ = version("pocketsat")
except PackageNotFoundError:
    # Imported from a bare source tree that was never installed: no metadata to read.
    __version__ = "0.0.0+unknown"

__all__ = ["__version__"]
