// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    public static class LimelightConstants{
        public static final String LIMELIGHT_1_NAME = "limelight-on";
        public static final String LIMELIGH_2_NAME = "limelight-back";
    }

    public static class IntakeConstants{
        public static final Integer INTAKE_MOTOR_ID = 0;
        public static final Integer INTAKE_OPENER_MOTOR_ID = 0; 
        public static final Double GEAR_RATIO = 1.0;

        public static final Double STATOR_CURRENT_LIMIT = 20.0;

        public static final Double CRUISE_VELOCITY = 1000.0;
        public static final Double ACCERELATION = 1000.0;
        public static final Double JERK = 0.0;

        public static final Double kS = 0.0;
        public static final Double kV = 0.0;
        public static final Double kA = 0.0;
        
        public static final Double kP = 12.5;
        public static final Double kI = 0.0;
        public static final Double kD = 0.0;

        
        public static final Double FORWARD_LIMIT = 200.0;
        public static final Double REVERSE_LIMIT = 0.0;

        public static final boolean FORWARD_LIMIT_ENABLED = true;
        public static final boolean REVERSE_LIMIT_ENABLED = true;
        
        public static final Double CLOSE_INTAKE_SETPOINT = 0.0;
        public static final Double HALF_CLOSE_INTAKE_SETPOINT = 0.0;
        public static final Double OPEN_INTAKE_SETPOINT = 0.0;

        public static final Double[] SETPOINTS = {0.0, 0.0, 0.0};
        public static final Integer CLOSE_INTAKE_INDEX = 0;
        public static final Integer HALF_CLOSE_INTAKE_INDEX = 1;
        public static final Integer OPEN_INTAKE_INDEX = 2;
    }

    public static class HoodConstants{
        public static final Integer HOOD_MOTOR_ID = 36;

        public static final Double GEAR_RATIO = 1.0;

        public static final Double STATOR_CURRENT_LIMIT = 20.0;

        public static final Double CRUISE_VELOCITY = 1000.0;
        public static final Double ACCERELATION = 1000.0;
        public static final Double JERK = 0.0;

        public static final Double kS = 0.0;
        public static final Double kV = 0.0;
        public static final Double kA = 0.0;
        
        public static final Double kP = 12.5;
        public static final Double kI = 0.0;
        public static final Double kD = 0.0;

        
        public static final Double FORWARD_LIMIT = 180.0;
        public static final Double REVERSE_LIMIT = 0.0;

        public static final boolean FORWARD_LIMIT_ENABLED = true;
        public static final boolean REVERSE_LIMIT_ENABLED = true;

        
    }

    public static class TurretConstants{
        public static final Integer TURRET_MOTOR_ID = 0;
        public static final Double GEAR_RATIO = 1.0;

        public static final Double TURRET_OFFSET = -0.175;

        public static final Double STATOR_CURRENT_LIMIT = 20.0;

        public static final Double CRUISE_VELOCITY = 1000.0;
        public static final Double ACCERELATION = 1000.0;
        public static final Double JERK = 0.0;

        public static final Double kS = 0.0;
        public static final Double kV = 0.0;
        public static final Double kA = 0.0;
        
        public static final Double kP = 12.5;
        public static final Double kI = 0.0;
        public static final Double kD = 0.0;

        
        public static final Double FORWARD_LIMIT = 200.0;
        public static final Double REVERSE_LIMIT = -200.0;

        public static final boolean FORWARD_LIMIT_ENABLED = true;
        public static final boolean REVERSE_LIMIT_ENABLED = true;
    }

    public static class ShooterConstants {
        public static final Integer SHOOTER_LEADER_MOTOR_ID = 0;
        public static final Integer SHOOTER_FOLLOWER_MOTOR_ID = 1;
    }

    public static class FieldConstants{
        public static final Integer HUB_X_INDEX = 0;
        public static final Integer HUB_Y_INDEX = 1;
        public static final Integer FEEDING_1_X_INDEX = 2;
        public static final Integer FEEDING_1_Y_INDEX = 3;
        public static final Integer FEEDING_2_X_INDEX = 4;
        public static final Integer FEEDING_2_Y_INDEX = 5;
        public static final Integer AREA_STARTING_INDEX = 6;
        public static final Integer AREA_ENDING_INDEX = 7;
        public static final Integer RAMP_STARTING_INDEX = 8;
        public static final Integer RAMP_ENDING_INDEX = 9;

        public static final Double RED_HUB_X = 12.0;
        public static final Double RED_HUB_Y = 4.0;
        public static final Double RED_FEEDING_AREA_1_X = 14.0;
        public static final Double RED_FEEDING_AREA_1_Y = 1.0;
        public static final Double RED_FEEDING_AREA_2_X = 14.0;
        public static final Double RED_FEEDING_AREA_2_Y = 7.0;
        public static final Double RED_AREA_STARTING_X = 11.0;
        public static final Double RED_AREA_ENDING_X = 16.0;
        public static final Double RED_RAMP_STARTING_X = 11.0;
        public static final Double RED_RAMP_ENDING_X = 13.0;
        
        public static final Double BLUE_HUB_X = 4.6;
        public static final Double BLUE_HUB_Y = 4.0;
        public static final Double BLUE_FEEDING_AREA_1_X = 2.0;
        public static final Double BLUE_FEEDING_AREA_1_Y = 1.0;
        public static final Double BLUE_FEEDING_AREA_2_X = 2.0;
        public static final Double BLUE_FEEDING_AREA_2_Y = 7.0;
        public static final Double BLUE_AREA_STARTING_X = 0.0;
        public static final Double BLUE_AREA_ENDING_X = 5.5;
        public static final Double BLUE_RAMP_STARTING_X = 3.5;
        public static final Double BLUE_RAMP_ENDING_X = 5.5;

        public static Double[] RED_POSITIONS = {
            RED_HUB_X, RED_HUB_Y,
            RED_FEEDING_AREA_1_X, RED_FEEDING_AREA_1_Y,
            RED_FEEDING_AREA_2_X, RED_FEEDING_AREA_2_Y,
            RED_AREA_STARTING_X, RED_AREA_ENDING_X,
            RED_RAMP_STARTING_X, RED_RAMP_ENDING_X
        }; 

        public static Double[] BLUE_POSITIONS = {
            BLUE_HUB_X, BLUE_HUB_Y,
            BLUE_FEEDING_AREA_1_X, BLUE_FEEDING_AREA_1_Y,
            BLUE_FEEDING_AREA_2_X, BLUE_FEEDING_AREA_2_Y,
            BLUE_AREA_STARTING_X, BLUE_AREA_ENDING_X,
            BLUE_RAMP_STARTING_X, BLUE_RAMP_ENDING_X
        };
    }
}