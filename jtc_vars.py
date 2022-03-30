
class JTCStatus:
    jtcFsm = 0
    jtcErrors = 0
    jtcOccurredErrors = 0
    jtcInitStatus = 0
    jtcFricType = 0
    jointsInitStatus = 0


class CANStatus:
    canStatus = 0
    canStatusFlags = 0
    canOccurredFlags = 0


class JointStatus:
    pos = 0
    vel = 0
    torque = 0
    temperature = 0
    currentFsm = 0
    mcCurrentError = 0
    mcOccurredError = 0
    currentError = 0
    currentWarning = 0
    internalErrors = 0
    internalOccurredErrors = 0

