#pragma once

enum class ErrorCodes
{
    // Error Codes of Precise PFLEX

    // StandardSystemErrors: -200 to -299
    // Hardware Errors: -300 to -499
    // Input Output Errors: -500 to -699
    // Language Related Errors: -700 to -999
    // Robot Related Errors: -1000 to -1499
    // Config Related Errors: -1500 to -1599
    // ControllerRelated Errors: -1600 to -1699
    // Network Related Errors: -1700 to -1799
    // GPL Related Errors: -2000 to -2999
    // Servo Related Errors: -3000 to -3999
    // Vision Related Errors: -4000 to -4100

    // Standard System Errors
    NoMemoryAvailable = -200,
    SystemInternalConsistencyError = -201,
    InvalidArgument = -202,
    FIFOoverflowed = -203,
    NotImplemented = -204,
    MissingArgument = -205,
    InvalidAutoExecuteMode = -206,
    TooMany = -207,
    ProtectionError = -208,
    ReadOnly = -209,
    OperatingSystemErrorCode = -210,
    OptionNotEnabled = -211,
    LicenseNotInstalled = -212,
    InvalidPassword = -213,
    Cancelled = -214,
    NoSystemClockInterrupts = -215,
    SystemClockInterruptsTooSlow = -216,
    SystemClockInterruptsTooFast = -217,
    InvalidTaskConfiguration = -218,
    IncompatibleFPGAVersion = -219,
    NotConfigured = -220,
    InvalidRobotType = -221,
    RemoteSoftwareIncompatible = -222,

    // Hardware Device Related Errors
    InvalidDevice = -300,
    UndefinedDevice = -301,
    InvalidDeviceUnit = -302,
    UndefinedDeviceUnit = -303,
    DeviceAlreadyInUse = -304,
    TooManyDevices = -308,
    NoPhysicalDeviceMapped = -309,
    TimeoutWaitingForDevice = -310,
    DateOrTimeNotSet = -311,
    InvalidDateOrTimeSpecification = -312,
    DeviceNotFound = -314,
    NVRAMNotResponding = -315,
    NVRAMInvalidResponse = -316,
    NVRAMDataInvalid = -317,
    RealTimeClockDisabled = -318,
    RealTimeClockBatteryFailed = -319,
    DeviceNotReady = -320,
    InvalidDeviceCommand = -321,
    I2CDeviceFailure = -322,
    DeviceFull = -323,
    MCPNotRecognized = -324,
    SIODeviceFailure = -325,
    DeviceNotEnabled = -326,
    InvalidDeviceConfiguration = -327,

    // Input and Output Errors
    DeviceIOError = -500,
    NoIOPending = -501,
    NoMessageBuffersAvailable = -502,
    DeviceNotFound = -503,
    LessDataThanExpected = -504,
    UnexpectedEndOfFile = -505,
    InputTooLong = -506,
    OutputTooLong = -507,
    FileNotFound = -508,
    CantOpenFileForWrite = -509,
    FileAlreadyExists = -510,
    CantCreateDirectory = -511,
    ErrorWritingFile = -512,
    ErrorReadingFile = -513,
    BufferAlreadyInUse = -514,
    InvalidInputCharacter = -515,
    InvalidFileName = -516,
    FilePathTooLong = -517,
    InvalidDataChecksum = -518,
    InvalidFileFormat = -519,
    FileNotOpen = -520,
    InvalidFileType = -521,
    FileInterlocked = -522,
    NoDataAvailable = -523,
    NoTimestampAvailable = -524,
    LatchInputOverrun = -525,
    LatchDataOverrun = -526,
    InvalidLatchConfiguration = -527,

    // Language Related Errors
    // Thread execution aborted
    ThreadExecutionAborted = -700,
    // Undefined thread
    UndefinedThread = -702,
    // Missing quote mark
    MissingQuoteMark = -704,
    // Value too small
    ValueTooSmall = -705,
    // Value too large
    ValueTooLarge = -706,
    // Value out-of-range
    ValueOutOfRange = -707,
    // Value infinite or NAN
    ValueInfiniteOrNAN = -708,

    // Division by 0
    DivisionBy0 = -709,
    // Arithmetic overflow
    ArithmeticOverflow = -710,
    // Singular matrix
    SingularMatrix = -711,
    // Invalid syntax
    InvalidSyntax = -712,
    // Symbol too long
    SymbolTooLong = -713,
    // Unknown command
    UnknownCommand = -714,
    // Invalid procedure step
    InvalidProcedureStep = -715,
    // Ambiguous abbreviation
    AmbiguousAbbreviation = -716,
    // Invalid number format
    InvalidNumberFormat = -717,
    // Missing parentheses
    MissingParentheses = -718,

    // Illegal use of keyword
    IllegalUseOfKeyword = -719,
    // Unexpected character in expression
    UnexpectedCharacterInExpression = -720,
    // Unexpected text at end of line
    UnexpectedTextAtEndOfLine = -722,

    // Invalid statement label
    InvalidStatementLabel = -723,
    // Unknown data type
    UnknownDataType = -725,
    // Data type required
    DataTypeRequired = -726,
    // Cannot redefine symbol
    CannotRedefineSymbol = -727,

    // Undefined symbol
    UndefinedSymbol = -729,
    // Invalid symbol type
    InvalidSymbolType = -730,
    // Unmatched parentheses
    UnmatchedParentheses = -731,
    // Invalid procedure end
    InvalidProcedureEnd = -732,
    // Not a top-level statement
    NotATopLevelStatement = -733,

    // Object not bound to class
    ObjectNotBoundToClass = -734,
    // Too many nested blocks
    TooManyNestedBlocks = -735,

    // Duplicate statement label
    DuplicateStatementLabel = -736,
    // Too many errors, compile cancelled
    TooManyErrors_CompileCancelled = -737,

    // Invalid data type
    InvalidDataType = -738,
    // Cannot change built-in symbol
    CannotChangeBuiltInSymbol = -739,

    // Argument mismatch
    ArgumentMismatch = -741,
    // Compilation errors
    CompilationErrors = -742,
    // Invalid project file
    InvalidProjectFile = -743,
    // Invalid start procedure
    InvalidStartProcedure = -744,
    // Project already exists
    ProjectAlreadyExists = -745,
    // Interlocked for read
    InterlockedForRead = -746,
    // Interlocked for write
    InterlockedForWrite = -747,
    // No matching control structure
    NoMatchingControlStructure = -748,

    // Thread already exists
    ThreadAlreadyExists = -749,

    // Invalid when thread active
    InvalidWhenThreadActive = -750,

    // Timeout starting thread
    TimeoutStartingThread = -751,
    // Timeout stopping thread
    TimeoutStoppingThread = -752,
    // Project not compiled
    ProjectNotCompiled = -753,
    // Thread execution complete
    ThreadExecutionComplete = -754,
    // Thread stack too small
    ThreadStackTooSmall = -755,

    // Member not shared
    MemberNotShared = -756,
    // Object not instantiated
    ObjectNotInstantiated = -757,
    // No Get Property defined
    NoGetPropertyDefined = -758,

    // Undefined value
    UndefinedValue = -759,

    // Invalid assignment
    InvalidAssignment = -760,
    // Cannot have list of variables
    CannotHaveListOfVariables = -761,
    // Location not a Cartesian type
    LocationNotACartesianType = -762,
    // Location not an angles type
    LocationNotAnAnglesType = -763,
    // Invalid procedure overload
    InvalidProcedureOverload = -764,
    // Array index required
    ArrayIndexRequired = -765,
    // Array index mismatch
    ArrayIndexMismatch = -766,
    // Invalid array index
    InvalidArrayIndex = -767,
    // Unsupported array access
    UnsupportedArrayAccess = -768,
    // Reference frame wrong type
    ReferenceFrameWrongType = -769,

    // Reference frame undefined data
    ReferenceFrameUndefinedData = -770,
    // Stack frame does not exist
    StackFrameDoesNotExist = -771,
    // Ambiguous Public reference
    AmbiguousPublicReference = -772,
    // Missing module end
    MissingModuleEnd = -773,
    // Too many breakpoints
    TooManyBreakpoints = -774,
    // Duplicate breakpoint
    DuplicateBreakpoint = -775,
    // No instruction at this line
    NoInstructionAtThisLine = -776,
    //   Objects not allowed for class
    ObjectsNotAllowedForClass = -778,
    // Thread paused in eval
    ThreadPausedInEval = -779,

    // Unsupported procedure reference
    UnsupportedProcedureReference = -780,
    // Missing string
    MissingString = -781,
    // Object value is Nothing
    ObjectValueIsNothing = -782,
    // Short string
    ShortString = -783,
    // Invalid property
    InvalidProperty = -784,
    // Branch not permitted
    BranchNotPermitted = -785,
    // Project generated error
    ProjectGeneratedError = -786,
    // Invalid in shared procedure
    InvalidInSharedProcedure = -787,

    // Inconsistent MOVE.TRIGGER mode
    InconsistentMOVETRIGGERMode = -788,
    // Procedure exception
    ProcedureException = -789,

    // Invalid static initializer
    InvalidStaticInitializer = -790,
    // Conveyor must be base RefFrame
    ConveyorMustBeBaseRefFrame = -791,
    // undefined
    undefined = -792,
    // Arc cannot transition conveyors
    ArcCannotTransitionConveyors = -793,

    // ZClearance property not set
    ZClearancePropertyNotSet = -794,
    // XML documents do not match
    XMLDocumentsDoNotMatch = -795,
    // No delegate defined
    NoDelegateDefined = -796,
    // Object not up-to-date
    ObjectNotUpToDate = -797,
    // No module defined
    NoModuleDefined = -798,
    // XML error
    XMLError = -799,

    // No XML document
    NoXMLDocument = -800,
    // No XML node
    NoXMLNode = -801,
    // Undefined XML name
    UndefinedXMLName = -802,
    // Invalid XML node type
    InvalidXMLNodeType = -803,
    // XML documents do not match
    XMLDocumentsDoNotMatch2 = -804,
    // Invalid circular reference
    InvalidCircularReference = -805,
    // Invalid Const reference
    InvalidConstReference = -806,
    // Invalid exception
    InvalidException = -807,
    // Branch out of Finally block not permitted
    BranchOutOfFinallyBlockNotPermitted = -808,
    // Expression too complex
    ExpressionTooComplex = -809,

    // Unexpected end of line
    UnexpectedEndOfLine = -810,
    // No Set Property defined
    NoSetPropertyDefined = -811,
    // Not Allowed In Factory Test System
    NotAllowedInFactoryTestSystem = -812,

    // Robot Related Errors

    // Invalid robot number
    InvalidRobotNumber = -1000,
    // Undefined robot
    UndefinedRobot = -1001,
    // Invalid axis number
    InvalidAxisNumber = -1002,
    // Undefined axis
    UndefinedAxis = -1003,
    // Invalid motor number
    InvalidMotorNumber = -1004,
    // Undefined motor
    UndefinedMotor = -1005,
    // Robot already attached
    RobotAlreadyAttached = -1006,
    // Robot not ready to be attached
    RobotNotReadyToBeAttached = -1007,
    // Can't detached a moving robot
    CantDetachedAMovingRobot = -1008,
    // No robot attached
    NoRobotAttached = -1009,

    // No robot selected
    NoRobotSelected = -1010,
    // Illegal during special Cartesian mode
    IllegalDuringSpecialCartesianMode = -1011,

    // Joint out-of-range
    JointOutOfRange = -1012,

    // Motor out-of-range
    MotorOutOfRange = -1013,
    // Time out during nulling
    TimeOutDuringNulling = -1014,
    // Invalid roll over spec
    InvalidRollOverSpec = -1015,
    // Torque control mode incorrect
    TorqueControlModeIncorrect = -1016,
    // Not in position control mode
    NotInPositionControlMode = -1017,
    // Not in velocity control mode
    NotInVelocityControlMode = -1018,
    // Timeout sending servo setpoint
    TimeoutSendingServoSetpoint = -1019,

    // Timeout reading servo status
    TimeoutReadingServoStatus = -1020,
    // Robot not homed
    RobotNotHomed = -1021,
    // Invalid homing parameter
    InvalidHomingParameter = -1022,
    // Missed signal during homing
    MissedSignalDuringHoming = -1023,
    // Encoder index disabled
    EncoderIndexDisabled = -1024,
    // Timeout enabling power
    TimeoutEnablingPower = -1025,
    // Timeout enabling amp
    TimeoutEnablingAmp = -1026,
    // Timeout starting commutation
    TimeoutStartingCommutation = -1027,
    // Hard E-STOP
    HardESTOP = -1028,
    // Asynchronous error
    AsynchronousError = -1029,

    // Fatal asynchronous error
    FatalAsynchronousError = -1030,
    // Analog input value too small
    AnalogInputValueTooSmall = -1031,
    // Analog input value too big
    AnalogInputValueTooBig = -1032,
    // Invalid Cartesian value
    InvalidCartesianValue = -1033,
    // Negative overtravel
    NegativeOvertravel = -1034,
    // Positive overtravel
    PositiveOvertravel = -1035,
    // Motors not commutated
    MotorsNotCommutated = -1037,
    // Project generated robot error
    ProjectGeneratedRobotError = -1038,

    // Position too close
    PositionTooClose = -1039,
    // Position too far
    PositionTooFar = -1040,

    // Invalid Base transform
    InvalidBaseTransform = -1041,
    // Can't change robot config
    CantChangeRobotConfig = -1042,
    // Asynchronous soft error
    AsynchronousSoftError = -1043,
    // Auto mode disabled
    AutoModeDisabled = -1044,
    // Soft E-STOP,
    SoftESTOP = -1045,
    // Power not enabled,
    PowerNotEnabled = -1046,
    // Virtual MCP in Jog mode,
    VirtualMCPInJogMode = -1047,
    // Hardware MCP in Jog mode,
    HardwareMCPInJogMode = -1048,
    // Timeout on homing DIN,
    TimeoutOnHomingDIN = -1049,

    // Illegal during joint motion,
    IllegalDuringJointMotion = -1050,
    // Incorrect Cartesian trajectory mode,
    IncorrectCartesianTrajectoryMode = -1051,
    // Beyond conveyor limits,
    BeyondConveyorLimits = -1052,
    // Beyond conveyor limits while tracking,
    BeyondConveyorLimitsWhileTracking = -1053,
    // Can't attach Encoder Only robot,
    CantAttachEncoderOnlyRobot = -1054,
    // Cartesian motion not configured,
    CartesianMotionNotConfigured = -1055,
    // Incompatible robot position,
    IncompatibleRobotPosition = -1056,
    TimeoutWaitingFrontPanelButton = -1057,
    CommutationDisabledByServos = -1058,
    HomedStateDisabledByServos = -1059,

    // Configuration Parameter Database, Datalogger, and CPU Monitor Errors
    InvalidDataIDCode = -1500,
    UnknownDataIDCode = -1501,
    InconsistentDuplicateDataIDCode = -1502,
    UninitializedParameterDatabase = -1505,
    MustBeMasterNode = -1507,
    InvalidDataIndex = -1509,
    DatabaseInternalConsistencyError = -1510,
    InvalidPointerValue = -1511,
    UndefinedCallbackRoutine = -1512,
    InvalidParameterArrayIndex = -1514,
    InvalidDataFileFormat = -1515,
    InvalidNumberOfAxesOrMotors = -1516,
    InvalidSignalNumber = -1517,
    UndefinedSignalNumber = -1518,
    OutputSignalRequired = -1519,
    CantOpenParameterDBFile = -1520,
    InvalidParameterDBFileNotLoaded = -1521,
    CannotWriteValueWhenPowerOn = -1522,
    CoupledAxisMustBeOnSameNode = -1523,
    SavingPDBValuesToFlashNotAllowed = -1524,
    DataIDCannotBeLogged = -1550,
    InvalidWhenDataloggerEnabled = -1551,
    DataloggerNotInitialized = -1552,
    NoDataItemsDefined = -1553,
    TriggerDataIDMustBeLogged = -1554,
    TriggerDataIDsOnDifferentNodes = -1555,
    TriggerNotAllowedForDataID = -1556,
    TooManyRemoteDataItems = -1558,
    InvalidWhenCPUMonitorEnabled = -1560,
    NoCPUMonitorDataAvailable = -1561,

    // Controller Errors
    PowerOffRequested = -1600,
    SoftwareResetDefaultSettings = -1601,
    ExternalESTOP = -1602,
    WatchdogTimerExpired = -1603,
    PowerLightFailure = -1604,
    UnknownPowerOffRequest = -1605,
    ESTOPStuckOff = -1606,
    TrajectoryTaskOverrun = -1607,
    ESTOPTimerFailed = -1609,
    ControllerOverheating = -1610,
    AutoManualSwitchSetToManual = -1611,
    PowerSupplyRelayStuck = -1612,
    PowerSupplyShorted = -1613,
    PowerSupplyOverloaded = -1614,
    NoThreePhasePower = -1615,
    ShutdownDueToOverheating = -1616,
    CPUOverheating = -1617,
    PowerSupplyNotCommunicating = -1618,
    PowerDisabledByGIOTimeout = -1619,
    SafetyDiagnosticsFailed = -1620,
    SafetySoftwareConfigurationMismatch = -1621,
    NotAllowedInSafetyMode = -1622,
    ESTOP1StuckOn = -1623,
    ESTOP2StuckOn = -1624,
    ESTOP1StuckOff = -1625,
    ESTOP2StuckOff = -1626,
    MotorPowerStuckOn = -1627,
    MotorPowerStuckOff = -1628,
    FFCENA_PWRSignalStuckOn = -1629,
    FFCENA_PWRSignalStuckOff = -1630,
    PowerDumpCircuitFailed = -1631,
    LeastOneMotorMustBeEnabled = -1632,
    MIDSWatchdogTimerFailedToDisablePower = -1633,

    // Network, Socket, and Communication Errors
    CannotGetLocalHostName = -1700,
    CannotGetLocalHostAddress = -1701,
    ConnectionRefused = -1702,
    NoConnection = -1703,
    InvalidNetworkAddress = -1704,
    NetworkTimeout = -1705,
    AlreadyConnected = -1706,
    SocketNotOpen = -1707,
    ConnectionClosed = -1708,
    InvalidProtocol = -1709,
    InvalidMulticastAddress = -1710,
    WebInterfaceNotEnabled = -1720,
    Modbus_RIOExceptionN = -1730,
    Modbus_RIODeviceTimeout = -1731,
    Modbus_RIODisableRequested = -1732,
    ServoLatencyTooLarge = -1740,

    // GPL Code Related Errors
    // Vertical search limit volated
    Vertical_search_limit_violated = -2101,
    // Horizontal search limit volated,
    Horizontal_search_limit_violated = -2102,
    // Insufficient number of Tool X samples,
    Insufficient_number_of_tool_X_samples = -2103,
    // Insufficient number of Tool TX torque samples,
    Insufficient_number_of_tool_tX_torque_samples = -2104,
    // No free air during Yaw direction,
    No_free_air_during_Yaw_direction = -2105,
    // Motor exceeded peak torque Mt:n,
    Motor_exceeded_peak_torque = -2106,
    // GPL 3.2 or later required,
    GPL_3_2_or_later_required = -2107,

    // Warning Parameter Mismatch
    WarningParameterMismatch = -2800,
    // Warning No Parameters,
    WarningNoParameters = -2801,
    // Warning Illegal move command,
    WarningIllegalMoveCommand = -2802,
    // Warning Invalid joint angles,
    WarningInvalidJointAngles = -2803,
    // Warning: Invalid Cartesian coordinate values,
    WarningInvalidCartesianCoordinateValues = -2804,
    // Unknown command,
    UnknownCommand2 = -2805,
    // Command Exception,
    CommandException = -2806,
    // Warning cannot set Input states,
    WarningCannotSetInputStates = -2807,
    // Not allowed by this thread,
    NotAllowedByThisThread = -2808,
    // Invalid robot type,
    InvalidRobotType = -2809,
    // Invalid serial command,
    InvalidSerialCommand = -2810,
    // Invalid robot number,
    InvalidRobotNumber2 = -2811,
    // Robot already selected,
    RobotAlreadySelected = -2812,
    // Modul not initialized,
    EcModNotInit = -2813,
    // Invlaid location index,
    EcInvLocIdx = -2814,
    // Undefined location,
    EcUndefLoc = -2816,
    // Undefined profile,
    EcUndefProf = -2817,
    // Undefined pallet,
    EcUndefPal = -2818,
    // Pallet not supported,
    EcNoPallet = -2819,
    // Invalid station index,
    EcInvStaIdx = -2820,
    // Undefined station,
    EcUndefSta = -2821,
    // Not a pallet,
    EcNotPallet = -2822,
    // Not a pallet origin,
    EcNotOrigin = -2823,

    // Invalid Gripper Type
    Invalid_Gripper_Type = -2850,
    // Invalid Station ID,
    Invalid_Station_ID = -2851,
    // Invalid robot state to execute command,
    // EcInvPARobotInvSta = -2852,
    Invalid_robot_state_to_execute_command = -2852,
    // Rail not at correct station,
    Rail_not_at_correct_station = -2853,
    // Invalid Station type,
    Invalid_Station_type = -2854,
    // No gripper close sensor,
    No_gripper_close_sensor = -2855,

    // Servo Related Errors
    // NULL pointer detected
    NULLPointerDetected = -3000,
    // Too many arguments,
    TooManyArguments = -3001,
    // Too few arguments,
    TooFewArguments = -3002,
    // Illegal value,
    IllegalValue = -3003,
    // Servo not initialized,
    ServoNotInitialized = -3004,
    // Servo mode transition failed,
    ServoModeTransitionFailed = -3005,
    // Servo mode locked,
    ServoModeLocked = -3006,
    // Servo hash table not found,
    ServoHashTableNotFound = -3007,
    // Servo hash entry collision,
    ServoHashEntryCollision = -3008,
    // No hash entry found,
    NoHashEntryFound = -3009,
    // Servo hash table full,
    ServoHashTableFull = -3010,
    // Illegal parameter access,
    IllegalParameterAccess = -3011,
    // Servo task submission failed,
    ServoTaskSubmissionFailed = -3013,
    // Cal parameters not set correctly,
    CalParametersNotSetCorrectly = -3014,
    // Cal position not ready,
    CalPositionNotReady = -3015,
    // Illegal cal seek command,
    IllegalCalSeekCommand = -3016,
    // No axis selected,
    NoAxisSelected = -3017,
    // Hard envelope error,
    HardEnvelopeError = -3100,
    // Illegal zero index,
    IllegalZeroIndex = -3102,
    // Missing zero index,
    MissingZeroIndex = -3103,
    // Motor duty cycle exceeded,
    MotorDutyCycleExceeded = -3104,
    // Motor stalled,
    MotorStalled = -3105,
    // Axis over-speed,
    AxisOverSpeed = -3106,
    // Amplifier over-current,
    AmplifierOverCurrent = -3107,
    // Amplifier over-voltage,
    AmplifierOverVoltage = -3108,
    // Amplifier under-voltage,
    AmplifierUnderVoltage = -3109,
    // Amplifier fault,
    AmplifierFault = -3110,
    // Brake fault,
    BrakeFault = -3111,
    // Excessive dual encoder slippage,
    ExcessiveDualEncoderSlippage = -3112,
    // Motor commutation setup failed,
    MotorCommutationSetupFailed = -3113,
    // Servo tasks overrun,
    ServoTasksOverrun = -3114,
    // Encoder quadrature error,
    EncoderQuadratureError = -3115,
    // Precise encoder index error,
    PreciseEncoderIndexError = -3116,
    // Amplifier RMS current exceeded,
    AmplifierRMSCurrentExceeded = -3117,
    // Dedicated DINs not config'ed for Hall,
    DedicatedDINsNotConfigedForHall = -3118,
    // Illegal 6-step number,
    Illegal6StepNumber = -3119,
    // Illegal commutation angle,
    IllegalCommutationAngle = -3120,
    // Encoder fault,
    EncoderFault = -3121,
    // Soft envelope error,
    SoftEnvelopeError = -3122,
    // Cannot switch serial encoder mode,
    CannotSwitchSerialEncoderMode = -3123,
    // Serial encoder busy,
    SerialEncoderBusy = -3124,
    // Illegal encoder command,
    IllegalEncoderCommand = -3125,
    // Encoder operation error,
    EncoderOperationError = -3126,
    // Encoder battery low,
    EncoderBatteryLow = -3127,
    // Encoder battery down,
    EncoderBatteryDown = -3128,
    // Invalid encoder multi-turn data,
    InvalidEncoderMultiTurnData = -3129,
    // Illegal encoder operation mode,
    IllegalEncoderOperationMode = -3130,
    // Encoder not supported or mis-matched,
    EncoderNotSupportedOrMisMatched = -3131,
    // Trajectory extrapolation limit exceeded,
    TrajectoryExtrapolationLimitExceeded = -3132,
    // Amplifier fault DC bus stuck,
    AmplifierFaultDCBusStuck = -3133,
    // Encoder data or accel/decel limit error,
    EncoderDataOrAcceldecelLimitError = -3134,
    // Phase offset too large,
    PhaseOffsetTooLarge = -3135,
    // Excessive movement during phase offset adjustment,
    ExcessiveMovementDuringPhaseOffsetAdjustment = -3136,
    // Amplifier hardware failure or invalid configuration,
    AmplifierHardwareFailureOrInvalidConfiguration = -3137,
    // Encoder position not ready,
    EncoderPositionNotReady = -3138,
    // Encoder not ready,
    EncoderNotReady = -3139,
    // Encoder communication error,
    EncoderCommunicationError = -3140,
    // Encoder overheated,
    EncoderOverheated = -3141,
    // Encoder hall sensor error,
    EncoderHallSensorError = -3142,
    // General serial bus encoder error,
    GeneralSerialBusEncoderError = -3143,
    // Amplifier overheating,
    AmplifierOverheating = -3144,
    // Motor overheating

    EncoderHardwareRelatedWarning = -3148,

    VelocityRestrictLimitExceeded = -3149,

    DumpCircutDutyCycleExceeded = -3160,
};
