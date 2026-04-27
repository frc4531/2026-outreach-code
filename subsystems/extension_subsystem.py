import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class ExtensionSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()
        self.extension_motor = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushless)

        self.extension_encoder = self.extension_motor.getAbsoluteEncoder()

        self.extension_config = rev.SparkMaxConfig()

        self.extension_controller = self.extension_motor.getClosedLoopController()

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        hopper_table = nt_instance.getTable("hopper_table")

        self.position_entry = hopper_table.getDoubleTopic("extension_position").publish()

        self.khP = 1.25
        self.khI = 0
        self.khD = 0
        self.hood_min_speed = -0.8
        self.hood_max_speed = 0.8

        self.extension_config.closedLoop.P(self.khP).I(self.khI).D(self.khD)
        self.extension_config.closedLoop.outputRange(self.hood_min_speed, self.hood_max_speed)
        self.extension_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)

    def periodic(self):
        self.position_entry.set(self.get_extension_position())

    def set_extension_speed(self, speed):
        self.extension_motor.set(speed)

    def get_extension_position(self):
        return self.extension_encoder.getPosition()
