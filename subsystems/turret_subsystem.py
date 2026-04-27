import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
from rev import PersistMode, ResetMode


class TurretSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.turret_motor = rev.SparkFlex(7, rev.SparkMax.MotorType.kBrushless)
        self.hood_motor = rev.SparkFlex(8, rev.SparkMax.MotorType.kBrushless)

        self.turret_motor.setInverted(True)
        self.hood_motor.setInverted(True)

        self.hood_encoder = self.hood_motor.getEncoder()
        self.turret_encoder = self.turret_motor.getEncoder()

        self.hood_config = rev.SparkMaxConfig()
        self.turret_config = rev.SparkMaxConfig()

        self.hood_pid_controller = self.hood_motor.getClosedLoopController()

        self.khP = 0.5
        self.khI = 0
        self.khD = 0
        self.hood_min_speed = -0.5
        self.hood_max_speed = 0.5

        self.hood_config.closedLoop.P(self.khP).I(self.khI).D(self.khD)
        self.hood_config.closedLoop.outputRange(self.hood_min_speed, self.hood_max_speed)

        self.hood_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        self.hood_config.inverted(True)

        #Start Turret Motor Config
        self.turret_pid_controller = self.turret_motor.getClosedLoopController()

        self.ktP = 0.075
        self.ktI = 0
        self.ktD = 0
        self.turret_min_speed = -1
        self.turret_max_speed = 1

        self.turret_config.closedLoop.P(self.ktP).I(self.ktI).D(self.ktD)
        self.turret_config.closedLoop.outputRange(self.turret_min_speed, self.turret_max_speed)

        self.turret_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)



        self.hood_motor.configure(self.hood_config,
                                       ResetMode.kResetSafeParameters,
                                       PersistMode.kPersistParameters)
        self.turret_motor.configure(self.turret_config,
                                        ResetMode.kResetSafeParameters,
                                        PersistMode.kPersistParameters)

        self.starting_limit_switch = wpilib.DigitalInput(0)
        self.ending_limit_switch = wpilib.DigitalInput(1)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        turret_table = nt_instance.getTable("turret_table")

        self.hood_position_entry = turret_table.getDoubleTopic("hood_position").publish()

        self.turret_position_entry = turret_table.getDoubleTopic("turret_position").publish()
        self.turret_state_entry = turret_table.getStringTopic("turret_state").publish()
        self.turret_velocity_entry = turret_table.getDoubleTopic("turret_velocity").publish()
        self.turret_pid_entry = turret_table.getDoubleTopic("turret_pid_output").publish()

        self.starting_limit_switch_entry = turret_table.getBooleanTopic("starting_limit_switch_state").publish()
        self.ending_limit_switch_entry = turret_table.getBooleanTopic("ending_limit_switch_state").publish()


        self.turret_state = "null"

    def periodic(self):
        self.hood_position_entry.set(self.get_hood_position())
        self.turret_position_entry.set(self.get_turret_position())
        self.turret_state_entry.set(self.turret_state)
        self.starting_limit_switch_entry.set(self.get_starting_limit_switch())
        self.ending_limit_switch_entry.set(self.get_ending_limit_switch())
        self.turret_velocity_entry.set(self.get_turret_velocity())

    def get_hood_position(self):
        return self.hood_encoder.getPosition()

    def get_turret_position(self):
        return self.turret_encoder.getPosition()

    def get_turret_velocity(self):
        return self.turret_encoder.getVelocity()

    def get_starting_limit_switch(self):
        return self.starting_limit_switch.get()

    def get_ending_limit_switch(self):
        return self.ending_limit_switch.get()

    def set_turret_speed(self, speed):
        if speed < 0 and self.get_starting_limit_switch() and self.get_turret_position() >= 0:
            self.turret_motor.set(speed)
            self.turret_state = "over_extending"
        elif speed > 0 and self.get_ending_limit_switch() and self.get_turret_position() <= 183:
            self.turret_motor.set(speed)
            self.turret_state = "over_extending"
        else:
            self.turret_motor.set(0)
            self.turret_state = "is_good"

    def set_hood_speed(self, speed):
        self.hood_motor.set(speed)
