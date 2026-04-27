import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class HopperSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.vectored_bar_motor = rev.SparkFlex(3, rev.SparkMax.MotorType.kBrushless)
        self.feeder_motor = rev.SparkFlex(4, rev.SparkMax.MotorType.kBrushless)

        self.feeder_motor.setInverted(True)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        hopper_table = nt_instance.getTable("hopper_table")

        self.position_entry = hopper_table.getDoubleTopic("extension_position").publish()

    def set_hopper_speed(self, speed):
        self.vectored_bar_motor.set(speed)
        self.feeder_motor.set(speed)