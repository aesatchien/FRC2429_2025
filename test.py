import math

def deltaAngle(target, current): #returns the shorter angle (and sign/direction) between two angles.
    target2 = target + 360 if target < 0 else target
    current2 = current + 360 if current < 0 else current
    turnAmount1 = target - current
    turnAmount2 = target2 - current2
    return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2

def deltaAngle2(target, current):
    target %= 360
    current %= 360

    target2 = target % 180 if target > 180 else target + 180
    turnAmount1 = target - current
    turnAmount2 = target2 - current
    return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2

def deltaAngle3(target, current): #returns the shorter angle (and sign/direction) between two angles.
    target %= 360
    current %= 360

    if target < 0 or current < 0: raise ValueError("Angles must be positive")

    if current < target:
        current1 = current
        turnAmount1 = target - current1
        current2 = current + 360
        turnAmount2 = target - current2
        return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2

    else:
        target1 = target
        turnAmount1 = target1 - current
        target2 = target + 360
        turnAmount2 = target2 - current
    
        return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2

print(-30 % 360)