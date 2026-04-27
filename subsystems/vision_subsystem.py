import math

import ntcore
import wpilib

from networktables import NetworkTables
from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d
from constants import position_constants


class VisionSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        left_table = self.inst.getTable("limelight-left")
        right_table = self.inst.getTable("limelight-right")
        turret_table = self.inst.getTable("limelight-turret")
        vision_table = self.inst.getTable("vision_table")
        side_table = self.inst.getTable("limelight-side")

        self.left_x_sub = left_table.getDoubleTopic("tx").subscribe(0.0)
        self.left_y_sub = left_table.getDoubleTopic("ty").subscribe(0.0)
        self.left_a_sub = left_table.getDoubleTopic("ta").subscribe(0.0)
        self.left_v_sub = left_table.getDoubleTopic("tv").subscribe(0.0)
        self.left_id_sub = left_table.getDoubleTopic("tid").subscribe(0)
        self.left_blue_pos_sub = left_table.getFloatArrayTopic("botpose_wpiblue").subscribe([0.0, 0.0])

        self.right_x_sub = right_table.getDoubleTopic("tx").subscribe(0.0)
        self.right_y_sub = right_table.getDoubleTopic("ty").subscribe(0.0)
        self.right_a_sub = right_table.getDoubleTopic("ta").subscribe(0.0)
        self.right_v_sub = right_table.getDoubleTopic("tv").subscribe(0.0)
        self.right_id_sub = right_table.getDoubleTopic("tid").subscribe(0)
        self.right_blue_pos_sub = right_table.getFloatArrayTopic("botpose_wpiblue").subscribe([0.0, 0.0])

        self.turret_x_sub = turret_table.getDoubleTopic("tx").subscribe(0.0)
        self.turret_y_sub = turret_table.getDoubleTopic("ty").subscribe(0.0)
        self.turret_a_sub = turret_table.getDoubleTopic("ta").subscribe(0.0)
        self.turret_v_sub = turret_table.getDoubleTopic("tv").subscribe(0.0)
        self.turret_id_sub = turret_table.getDoubleTopic("tid").subscribe(0.0)
        self.turret_blue_pos_sub = turret_table.getFloatArrayTopic("botpose_orb_wpiblue").subscribe([0.0, 0.0])
        #
        self.side_x_sub = side_table.getDoubleTopic("tx").subscribe(0.0)
        self.side_y_sub = side_table.getDoubleTopic("ty").subscribe(0.0)
        self.side_a_sub = side_table.getDoubleTopic("ta").subscribe(0.0)
        self.side_v_sub = side_table.getDoubleTopic("tv").subscribe(0.0)
        self.side_id_sub = side_table.getDoubleTopic("tid").subscribe(0.0)
        self.side_blue_pos_sub = side_table.getFloatArrayTopic("botpose_wpiblue").subscribe([0.0, 0.0])

        self.turret_imu_sub = turret_table.getFloatArrayTopic("imu").subscribe([0.0, 0.0])
        # self.side_imu_sub = side_table.getFloatArrayTopic("imu").subscribe([0.0, 0.0])

        self.left_x_entry = 0
        self.left_y_entry = 0
        self.left_a_entry = 0
        self.left_v_entry = 0
        self.left_id_entry = 0
        self.left_blue_pos = [0.0, 0.0]

        self.right_x_entry = 0
        self.right_y_entry = 0
        self.right_a_entry = 0
        self.right_v_entry = 0
        self.right_id_entry = 0
        self.right_blue_pos = [0.0, 0.0]

        self.turret_x_entry = 0
        self.turret_y_entry = 0
        self.turret_a_entry = 0
        self.turret_v_entry = 0
        self.turret_id_entry = 0
        self.turret_blue_pos = [0.0, 0.0]

        self.turret_angle = 0

        self.side_x_entry = 0
        self.side_y_entry = 0
        self.side_a_entry = 0
        self.side_v_entry = 0
        self.side_id_entry = 0
        self.side_blue_pos = 0

        # self.side_angle = 0

        self.avg_y_cord = 0
        self.avg_x_cord = 0
        self.avg_v_entry = 0
        self.avg_id_entry = 0

        self.corrected_turret_x_coord = 0.0
        self.corrected_turret_y_coord = 0.0

        self.avg_y_cord_entry = vision_table.getFloatTopic("avg_y_cord").publish()
        self.avg_x_cord_entry = vision_table.getFloatTopic("avg_x_cord").publish()
        self.avg_v_entry_publish = vision_table.getFloatTopic("avg_v_entry").publish()
        self.avg_id_entry_publish = vision_table.getFloatTopic("avg_id_entry").publish()

        self.corrected_turret_x_coord_entry = vision_table.getFloatTopic("corrected_turret_x_coord").publish()
        self.corrected_turret_y_coord_entry = vision_table.getFloatTopic("corrected_turret_y_coord").publish()

        self.turret_angle_publish = vision_table.getFloatTopic("turret_angle").publish()
        self.corrected_turret_angle_entry = vision_table.getFloatTopic("corrected_turret_angle").publish()

        self.drive_heading = 0.0

        self.drive_table = self.inst.getTable("drive_table")
        self.drive_heading_entry = self.drive_table.getFloatTopic("drive_train_heading").subscribe(0.0)

    def periodic(self):
        self.left_x_entry = self.left_x_sub.get()
        self.left_y_entry = self.left_y_sub.get()
        self.left_a_entry = self.left_a_sub.get()
        self.left_v_entry = self.left_v_sub.get()
        self.left_id_entry = self.left_id_sub.get()
        self.left_blue_pos = self.left_blue_pos_sub.get()

        self.right_x_entry = self.right_x_sub.get()
        self.right_y_entry = self.right_y_sub.get()
        self.right_a_entry = self.right_a_sub.get()
        self.right_v_entry = self.right_v_sub.get()
        self.right_id_entry = self.right_id_sub.get()
        self.right_blue_pos = self.right_blue_pos_sub.get()

        self.turret_x_entry = self.turret_x_sub.get()
        self.turret_y_entry = self.turret_y_sub.get()
        self.turret_a_entry = self.turret_a_sub.get()
        self.turret_v_entry = self.turret_v_sub.get()
        self.turret_id_entry = self.turret_id_sub.get()
        self.turret_blue_pos = self.turret_blue_pos_sub.get()

        self.side_x_entry = self.side_x_sub.get()
        self.side_y_entry = self.side_y_sub.get()
        self.side_a_entry = self.side_a_sub.get()
        self.side_v_entry = self.side_v_sub.get()
        self.side_id_entry = self.side_id_sub.get()
        self.side_blue_pos = self.side_blue_pos_sub.get()

        self.turret_angle_publish.set(self.turret_angle)

        self.turret_angle = self.turret_imu_sub.get()[0]
        self.corrected_turret_angle_entry.set(self.get_relative_angle(-90))

        self.drive_heading = self.drive_heading_entry.get()

        # Get position coordinates of the Turret relative to field

        drive_heading_in_radians = ((self.drive_heading * math.pi) / 180.0)

        self.corrected_turret_x_coord = self.avg_x_cord + (position_constants.PositionConstants.kXTurretOffset * math.sin(drive_heading_in_radians))
        self.corrected_turret_y_coord = self.avg_y_cord + (position_constants.PositionConstants.kYTurretOffset * math.cos(drive_heading_in_radians))
        self.corrected_turret_x_coord_entry.set(self.corrected_turret_x_coord)
        self.corrected_turret_y_coord_entry.set(self.corrected_turret_y_coord)

        # Avg Info Estimator
        # if self.left_v_entry == 1 and self.right_v_entry == 1:
        #     self.avg_y_cord = (self.left_blue_pos[1] + self.right_blue_pos[1]) / 2
        #     self.avg_x_cord = (self.left_blue_pos[0] + self.right_blue_pos[0]) / 2
        #     self.avg_v_entry = 1
        #     self.avg_id_entry = self.left_id_entry
        # elif self.left_v_entry == 1 and self.right_v_entry == 0:
        #     self.avg_y_cord = self.left_blue_pos[1]
        #     self.avg_x_cord = self.left_blue_pos[0]
        #     self.avg_v_entry = 1
        #     self.avg_id_entry = self.left_id_entry
        # elif self.left_v_entry == 0 and self.right_v_entry == 1:
        #     self.avg_y_cord = self.right_blue_pos[1]
        #     self.avg_x_cord = self.right_blue_pos[0]
        #     self.avg_v_entry = 1
        #     self.avg_id_entry = self.right_id_entry
        # else:
        #     self.avg_y_cord = -1
        #     self.avg_x_cord = -1
        #     self.avg_v_entry = -1
        #     self.avg_id_entry = -1

        # Avg Info Estimator
        if self.left_v_entry == 1 and self.right_v_entry == 1 and self.side_v_entry == 0:
            self.avg_y_cord = (self.left_blue_pos[1] + self.right_blue_pos[1]) / 2
            self.avg_x_cord = (self.left_blue_pos[0] + self.right_blue_pos[0]) / 2
            self.avg_v_entry = 2
            self.avg_id_entry = self.left_id_entry
        elif self.left_v_entry == 1 and self.right_v_entry == 1 and self.side_v_entry == 1:
            self.avg_y_cord = (self.left_blue_pos[1] + self.right_blue_pos[1] + self.side_blue_pos[1]) / 3
            self.avg_x_cord = (self.left_blue_pos[0] + self.right_blue_pos[0] + self.side_blue_pos[0]) / 3
            self.avg_v_entry = 3
            self.avg_id_entry = self.left_id_entry
        elif self.left_v_entry == 0 and self.right_v_entry == 1 and self.side_v_entry == 1:
            self.avg_y_cord = (self.right_blue_pos[1] + self.side_blue_pos[1]) / 2
            self.avg_x_cord = (self.right_blue_pos[0] + self.side_blue_pos[0]) / 2
            self.avg_v_entry = 2
            self.avg_id_entry = self.right_id_entry
        elif self.left_v_entry == 1 and self.right_v_entry == 0 and self.side_v_entry == 1:
            self.avg_y_cord = (self.left_blue_pos[1] + self.side_blue_pos[1]) / 2
            self.avg_x_cord = (self.left_blue_pos[0] + self.side_blue_pos[0]) / 2
            self.avg_v_entry = 2
            self.avg_id_entry = self.right_id_entry
        elif self.left_v_entry == 1 and self.right_v_entry == 0 and self.side_v_entry == 0:
            self.avg_y_cord = self.left_blue_pos[1]
            self.avg_x_cord = self.left_blue_pos[0]
            self.avg_v_entry = 1
            self.avg_id_entry = self.left_id_entry
        elif self.left_v_entry == 0 and self.right_v_entry == 1 and self.side_v_entry == 0:
            self.avg_y_cord = self.right_blue_pos[1]
            self.avg_x_cord = self.right_blue_pos[0]
            self.avg_v_entry = 1
            self.avg_id_entry = self.right_id_entry
        elif self.left_v_entry == 0 and self.right_v_entry == 0 and self.side_v_entry == 1:
            self.avg_y_cord = self.side_blue_pos[1]
            self.avg_x_cord = self.side_blue_pos[0]
            self.avg_v_entry = 1
            self.avg_id_entry = self.side_id_entry
        else:
            self.avg_y_cord = -1
            self.avg_x_cord = -1
            self.avg_v_entry = -1
            self.avg_id_entry = -1

        # camera_amount = self.left_v_entry + self.right_v_entry + self.side_v_entry
        #
        # if camera_amount != 0:
        #     self.avg_y_cord = (self.left_blue_pos[1] + self.right_blue_pos[1] + self.side_blue_pos[1])/camera_amount
        #     self.avg_x_cord = (self.left_blue_pos[0] + self.right_blue_pos[0] + self.side_blue_pos[0])/camera_amount
        #
        #     self.avg_v_entry = (self.left_v_entry + self.right_v_entry + self.side_v_entry)/camera_amount
        #     self.avg_id_entry = (self.left_id_entry + self.right_id_entry + self.side_id_entry)/camera_amount
        # else:
        #     self.avg_y_cord = -1
        #     self.avg_x_cord = -1
        #     self.avg_v_entry = -1
        #     self.avg_id_entry = -1

        self.avg_y_cord_entry.set(self.avg_y_cord)
        self.avg_x_cord_entry.set(self.avg_x_cord)
        self.avg_v_entry_publish.set(self.avg_v_entry)
        self.avg_id_entry_publish.set(self.avg_id_entry)

    def get_corrected_turret_angle(self):
        turret_rotation = -self.turret_angle
        drive_rotation = self.drive_heading

        if turret_rotation < 0:
            turret_rotation_adjusted = turret_rotation + 360
        else:
            turret_rotation_adjusted = turret_rotation

        if drive_rotation < 0:
            drive_rotation_adjusted = drive_rotation + 360
        else:
            drive_rotation_adjusted = drive_rotation

        total_turret_adjusted = turret_rotation_adjusted - drive_rotation_adjusted

        if total_turret_adjusted < 0:
            return total_turret_adjusted + 360
        else:
            return total_turret_adjusted

    def get_relative_angle(self, angle):
        turret_rotation = -angle
        drive_rotation = self.drive_heading

        if turret_rotation < 0:
            turret_rotation_adjusted = turret_rotation + 360
        else:
            turret_rotation_adjusted = turret_rotation

        if drive_rotation < 0:
            drive_rotation_adjusted = drive_rotation + 360
        else:
            drive_rotation_adjusted = drive_rotation

        total_turret_adjusted = turret_rotation_adjusted - drive_rotation_adjusted

        if total_turret_adjusted < 0:
            return total_turret_adjusted + 360
        else:
            return total_turret_adjusted

