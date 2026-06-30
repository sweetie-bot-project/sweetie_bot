import os
import sys

# Make the package importable from src/ without an install (tests + CI).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
