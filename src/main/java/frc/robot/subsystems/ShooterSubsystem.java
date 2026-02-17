package frc.robot.subsystems;

// TODO add ring security measures

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX feederMotor;
    private TalonFX spinWheelMotor;

    private TalonFX shooterLeader;
    private TalonFX shooterFollower;

    private TalonFX turretMotor;

    private TalonFX hoodMotor;

    public ShooterSubsystem() {
        shooterLeader = new TalonFX(ShooterConstants.SHOOTER_LEADER_MOTOR_ID);
        shooterFollower = new TalonFX(ShooterConstants.SHOOTER_FOLLOWER_MOTOR_ID);

        shooterFollower.setControl(
            new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed)
        );

        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);

        hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);

        turretMotor.setPosition(0);
        hoodMotor.setPosition(0);
    }

    private double degreesToRotations(double degrees, double gearRatio) {
        return (degrees / 360.0) * gearRatio;
    }


    public void spinShooterToLife() {
        shooterLeader.setControl(new VelocityVoltage(150));
    }

    public double getShooterRPM() {
        return shooterLeader.getVelocity().getValue().in(RPM);
    }

    public boolean atSetpoint() {
        return Math.abs(getShooterRPM() - 9000) < 100;
    }

    public void stopShooter() {
        shooterLeader.set(0);
        feederMotor.set(0);
    }

    public void startFeeder() {
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

        turretMotor.setControl(
            new PositionVoltage(ringRotations)
        );

        hoodMotor.setControl(
            new PositionVoltage(hoodRotations)
        );
    }
}
