"""Navigation business APIs; importing this package never connects to a robot."""
from .client import NavigationClient
from .errors import NavigationError

__version__ = "0.1.0"
__all__ = ["NavigationClient", "NavigationError"]
