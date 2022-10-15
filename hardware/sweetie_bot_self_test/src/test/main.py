#!/usr/bin/env python3
import pytest
import sys
import os 

def main():
    sys.exit(pytest.main( ["--tb=short", "--junitxml=/tmp/result.xml", os.path.dirname(os.path.realpath(__file__))] ))

