import commands2
import wpilib
import wpimath.controller

from subsystems.extension_subsystem import ExtensionSubsystem


class ExtensionToPosition(commands2.PIDCommand):

    def __init__(self, extension_sub: ExtensionSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(1.75, 0, 0),
            # Close loop on absolute encoder
            lambda: extension_sub.get_extension_position(),
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: extension_sub.set_extension_speed(output),
            # Require the arm
            extension_sub
        )

    def initialize(self) -> None:
        self.getController().enableContinuousInput(0, 1)

    def isFinished(self) -> bool:
        return False