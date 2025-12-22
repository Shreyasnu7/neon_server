
import pytest
import sys, os

# Add the camera_brain directory to path so 'laptop_ai' matches
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../ai/camera_brain")))

from laptop_ai.director_core import Director

def test_director_init():
    # Should be able to instantiate without syntax errors
    # We pass simulate=True to avoid networking
    d = Director(simulate=True)
    assert d.simulate is True
