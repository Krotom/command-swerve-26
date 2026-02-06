package frc.robot;

public class Constants {
    public class ShooterConstants {
        public static final int kFeederMotorID = 5;
        public static final int kSpinWheelMotorID = 6;

        public static final int kShooterLeaderID = 1;
        public static final int kShooterFollowerID = 2;

        public static final int kShooterRingMotorID = 3;

        public static final int kHoodMotorID = 4;

        public static final double kRingGearRatio = 100.0; // TODO learn the actual ratios
        public static final double kHoodGearRatio = 50.0;

        public static final double kMaxRingAngleDeg = 90.0;
        public static final double kMinHoodAngleDeg = 0.0;
        public static final double kMaxHoodAngleDeg = 45.0;
    }

    public class IntakeConstants {
        public static final int kDeployMotorID = 7;
        public static final int kIntakeMotorID = 8;
    }
}
