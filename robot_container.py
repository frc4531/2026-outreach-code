import math

import ntcore
import wpilib
import commands2
import wpimath
from commands2 import WaitCommand, InstantCommand, ParallelCommandGroup
from commands2.cmd import waitSeconds

from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.units import rotationsToDegrees

from commands.drive_command import DriveCommand
from commands.extension_to_position import ExtensionToPosition
from commands.hood_and_turret_to_positions import HoodAndTurretToPositions
from commands.hood_down import HoodDown
from commands.hood_to_positon import HoodToPosition
from commands.hood_up import HoodUp
from commands.hopper_backwards import HopperBackwards
from commands.hopper_out import HopperOut
from commands.intake_feeder import IntakeFeeder
from commands.intake_in import IntakeIn
from commands.intake_out import IntakeOut
from commands.shooter_off import ShooterOff
from commands.shooter_to_velocity import ShooterToVelocity
from commands.turret_left import TurretLeft
from commands.turret_right import TurretRight
from commands.turret_to_position import TurretToPosition
from constants.position_constants import PositionConstants
from constants.swerve_constants import OIConstants, AutoConstants, DriveConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.extension_subsystem import ExtensionSubsystem
from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.turret_subsystem import TurretSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.intake_subsystem import IntakeSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive_subsystem = DriveSubsystem()
        self.vision_subsystem = VisionSubsystem()
        self.intake_subsystem = IntakeSubsystem()
        self.shooter_subsystem = ShooterSubsystem()
        self.hopper_subsystem = HopperSubsystem()
        self.turret_subsystem = TurretSubsystem()
        self.extension_subsystem = ExtensionSubsystem()

        # The driver's controller
        self.driver_controller = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operator_controller = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(self.drive_subsystem)
        )

        self.turret_subsystem.setDefaultCommand(
            ParallelCommandGroup(
                InstantCommand(self.turret_subsystem.turret_pid_controller.setReference(self.turret_subsystem.get_turret_position())),
                InstantCommand(self.turret_subsystem.hood_pid_controller.setReference(self.turret_subsystem.get_hood_position()))
            )
        )

        self.shooter_subsystem.setDefaultCommand(
            ShooterToVelocity(self.shooter_subsystem, 3000)
        )

    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # -- OPERATOR CONTROL BLOCK --
        # Intake In
        commands2.button.JoystickButton(self.operator_controller, 1).whileTrue(
            IntakeIn(self.intake_subsystem)
        )
        # Hopper Out
        commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
            HopperOut(self.hopper_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
            IntakeFeeder(self.intake_subsystem)
        )
        # Shooter Off
        commands2.button.JoystickButton(self.operator_controller, 9).toggleOnTrue(
            ShooterOff(self.shooter_subsystem)
        )

        # -- HOOD AND TURRET MANUAL CONTROL BLOCK --
        commands2.button.JoystickButton(self.operator_controller, 11).whileTrue(
            TurretLeft(self.turret_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 12).whileTrue(
            TurretRight(self.turret_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 13).whileTrue(
            HoodUp(self.turret_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 14).whileTrue(
            HoodDown(self.turret_subsystem)
        )

        # -- DRIVER CONTROL BLOCK --
        # Hopper Extension Ungelation (e ur e ur)
        commands2.button.JoystickButton(self.driver_controller, 2).whileTrue(
            commands2.SequentialCommandGroup(
                commands2.ParallelDeadlineGroup(
                    WaitCommand(PositionConstants.kTimedAgitationIn),
                    ExtensionToPosition(self.extension_subsystem, PositionConstants.kInHopperAgitation),
                ),
            commands2.ParallelDeadlineGroup(
                WaitCommand(PositionConstants.kTimedAgitationOut),
                ExtensionToPosition(self.extension_subsystem, PositionConstants.kOutHopperExtension),
                )
            ).repeatedly()
        )
        # Driver Feeder
        commands2.button.JoystickButton(self.driver_controller, 1).whileTrue(
            HopperOut(self.hopper_subsystem)
        )
        commands2.button.JoystickButton(self.driver_controller, 1).whileTrue(
            IntakeFeeder(self.intake_subsystem)
        )

    def disable_pid_subsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def get_autonomous_command(self) -> commands2.command:
        return waitSeconds(1)
