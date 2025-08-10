import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))
from module6.morse_encoder import MorseEncoder

def test_encode():
    enc = MorseEncoder()
    assert enc.encode("hello") == "HELLO"
