import numpy as np


def integration1(x, xDot, dt):

    xNew = x + xDot * dt

    return xNew
