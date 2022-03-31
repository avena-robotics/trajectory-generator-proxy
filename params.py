import enum, math

class JTC_FSM(enum.Enum):
    Null = 255
    Start = 0
    Init = enum.auto()
    HoldPos = enum.auto()
    Operate = enum.auto()
    Teaching = enum.auto()
    Error = enum.auto()


class Joint_FSM(enum.Enum):
    Start = 0
    Init = 1
    ReadyToOperate = 2
    OperationEnable = 3
    TransStartToInit = 10
    TransInitToReadyToOperate = 11
    TransReadyToOperateToOperationEnable = 12
    TransOperationEnableToReadyToOperate = 13
    TransFaultReactionActiveToFault = 14
    TransFaultToReadyToOperate = 15
    ReactionActive = 254
    Fault = 255


class Host_FT(enum.Enum):
    Header = 155
    Host_FT_null = 0
    ClearCurrentErrors = enum.auto()
    ClearOccuredErrors = enum.auto()
    JtcStatus = enum.auto()
    Trajectory = enum.auto()
    FrictionTable = enum.auto()
    FrictionTableUseDefault = enum.auto()
    PidParam = enum.auto()
    PidParamUseDefault = enum.auto()
    ArmModel = enum.auto()
    ArmModelUseDefault = enum.auto()
    TrajSetExecStatus = enum.auto()
    TeachingModeEnable = enum.auto()
    TeachingModeDisable = enum.auto()
    FrictionPolynomial = enum.auto()
    FrictionPolynomialUseDefault = enum.auto()


class Host_RxFS(enum.Enum):
    Idle = 0
    NoError = enum.auto()
    ErrorIncorrectHeader = enum.auto()
    ErrorIncorrectFrameType = enum.auto()
    ErrorIncorrectCrc = enum.auto()
    ErrorTimeout = enum.auto()


class Host_RxDS(enum.Enum):
    Idle = 0
    NoError = enum.auto()
    TrajTooManyPoints = enum.auto()
    TrajTooManySegs = enum.auto()
    TrajIncorrectSegOrder = enum.auto()
    TrajIncorrectStepTime = enum.auto()
    GetFrameIncorectData = enum.auto()


class Host_FTAS(enum.Enum):
    Null = 0
    ClearCurrentErrors = enum.auto()
    ClearOccuredErrors = enum.auto()
    Trajectory = enum.auto()
    FrictionTable = enum.auto()
    FrictionTableUseDefault = enum.auto()
    PidParam = enum.auto()
    PidParamUseDefault = enum.auto()
    ArmModel = enum.auto()
    ArmModelUseDefault = enum.auto()
    TrajSetExecStatus = enum.auto()
    TeachingModeEnable = enum.auto()
    TeachingModeDisable = enum.auto()
    FrictionPolynomial = enum.auto()
    FrictionPolynomialUseDefault = enum.auto()


class Host_FTSS(enum.Enum):
    GetJtcStatus = 0
    GetCanStatus = enum.auto()
    GetCanMissingFrameLevel = enum.auto()
    GetJointsStatus = enum.auto()
    GetJointsValues = enum.auto()


class JTC_FT(enum.Enum):
    Polynomial = 0
    Table = 1


class TES(enum.Enum):
    TES_Null = 0
    TES_Stop = enum.auto()
    TES_Pause = enum.auto()
    TES_Execute = enum.auto()
    TES_Finish = enum.auto()


COMFRAMETOSYNCHROSENDMAX = 1
COMBUFREADMAX = 10000
COMTIMESEND = 25
COMTIMEREAD = 15
COMTIMEOUT = 1000
JOINTS_MAX = 6
JOINTS_FRICCOEFFMAX = 4
JOINT_POSMAX = math.pi
JOINT_VELMAX = 2 * math.pi
JOINT_ACCMAX = 4 * math.pi
JOINT_TORQUEMAX = 256.0
MAX_INT16 = 32767.0
TRAJ_MAXPOINTINSEG = 500
ARMMODEL_DOF = 6

MAX_VEL = 0.5 # rad/s
TIME_STEP = 0.01 # sec
