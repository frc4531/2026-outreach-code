import commands2

from subsystems.hopper_subsystem import HopperSubsystem


class HopperOut(commands2.Command):

    def __init__(self, hopper_sub: HopperSubsystem) -> None:
        super().__init__()

        self.hopper_sub = hopper_sub
        self.addRequirements(self.hopper_sub)

    def execute(self) -> None:
        self.hopper_sub.set_hopper_speed(0.9)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.hopper_sub.set_hopper_speed(0)