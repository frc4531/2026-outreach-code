import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
from rev import ResetMode, PersistMode, SparkFlexConfig, SparkFlex


class ShooterSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_shooter_motor = SparkFlex(9, rev.SparkMax.MotorType.kBrushless)
        self.right_shooter_motor = SparkFlex(10, rev.SparkMax.MotorType.kBrushless)

        self.left_encoder = self.left_shooter_motor.getEncoder()

        self.right_shooter_config = SparkFlexConfig()
        self.left_shooter_config = SparkFlexConfig()

        self.right_shooter_config.follow(9, True)
        self.left_shooter_config.inverted(True)

        self.shooter_pid_controller = self.left_shooter_motor.getClosedLoopController()

        self.ksP =  0.00075
        self.ksI = 1 * (10 ** -7)
        self.ksD = 0
        self.ksIz = 0
        self.ksFF = 1/6784
        self.ksV = 12/6784
        self.kMinOutput = -1
        self.kMaxOutput = 1

        self.left_shooter_config.closedLoop.pidf(self.ksP, self.ksI, self.ksD, self.ksFF).IZone(self.ksIz)
        self.left_shooter_config.closedLoop.feedForward.kV(self.ksV)
        self.left_shooter_config.closedLoop.outputRange(self.kMinOutput,self.kMaxOutput)

        self.left_shooter_config.openLoopRampRate(0)
        self.left_shooter_config.closedLoopRampRate(0)

        self.right_shooter_motor.configure(self.right_shooter_config,
                                        ResetMode.kResetSafeParameters,
                                        PersistMode.kPersistParameters)
        self.left_shooter_motor.configure(self.left_shooter_config,
                                           ResetMode.kResetSafeParameters,
                                           PersistMode.kPersistParameters)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        shooter_table = nt_instance.getTable("shooter_table")

        self.shooter_velocity_entry = shooter_table.getDoubleTopic("shooter_velocity").publish()

    def periodic(self):
        self.shooter_velocity_entry.set(self.get_shooter_velocity())

    def set_shooter_speed(self, speed):
        self.left_shooter_motor.set(speed)

    def get_shooter_velocity(self):
        return self.left_encoder.getVelocity()

    def set_shooter_velocity(self, velocity):
        self.shooter_pid_controller.setReference(velocity, rev.SparkFlex.ControlType.kVelocity)
