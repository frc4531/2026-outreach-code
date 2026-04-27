import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class IntakeSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.bottom_intake_motor = rev.SparkFlex(1, rev.SparkMax.MotorType.kBrushless)
        self.top_intake_motor = rev.SparkFlex(2, rev.SparkMax.MotorType.kBrushless)

        self.bottom_intake_motor.setInverted(True)

    def set_intake_speed(self, speed):
        self.top_intake_motor.set(speed)
        self.bottom_intake_motor.set(speed)
