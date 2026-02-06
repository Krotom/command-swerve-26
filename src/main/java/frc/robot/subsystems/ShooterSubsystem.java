package frc.robot.subsystems;

// TODO add ring security measures

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX feederMotor;
    private TalonFX spinWheelMotor;

    private TalonFX shooterLeader;
    private TalonFX shooterFollower;

    private TalonFX shooterRingMotor;

    private TalonFX hoodMotor;

    public ShooterSubsystem() {
        feederMotor = new TalonFX(ShooterConstants.kFeederMotorID);
        spinWheelMotor = new TalonFX(ShooterConstants.kSpinWheelMotorID);

        shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderID);
        shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerID);

        shooterFollower.setControl(
            new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed)
        );

        shooterRingMotor = new TalonFX(ShooterConstants.kShooterRingMotorID);

        hoodMotor = new TalonFX(ShooterConstants.kHoodMotorID);

        shooterRingMotor.setPosition(0);
        hoodMotor.setPosition(0);
    }

    private double degreesToRotations(double degrees, double gearRatio) {
        return (degrees / 360.0) * gearRatio;
    }


    private void spinShooterToLife() {
        shooterLeader.setControl(new VelocityVoltage(150));
    }

    private void stopShooter() {
        shooterLeader.set(0);
        feederMotor.set(0);
    }

    private void startFeeder() {
        feederMotor.set(1);
        spinWheelMotor.set(-1);
    }

    public void aim(double x, double y) {
        x = Math.max(-1.0, Math.min(1.0, x));
        y = Math.max(-1.0, Math.min(1.0, y));

        double ringAngleDeg = x * ShooterConstants.kMaxRingAngleDeg;

        double hoodAngleDeg =
        ShooterConstants.kMinHoodAngleDeg +
        (y + 1.0) / 2.0 * (ShooterConstants.kMaxHoodAngleDeg - ShooterConstants.kMinHoodAngleDeg);

        double ringRotations =
        degreesToRotations(ringAngleDeg, ShooterConstants.kRingGearRatio);

        double hoodRotations =
            degreesToRotations(hoodAngleDeg, ShooterConstants.kHoodGearRatio);

        shooterRingMotor.setControl(
            new PositionVoltage(ringRotations)
        );

        hoodMotor.setControl(
            new PositionVoltage(hoodRotations)
        );
    }
}
