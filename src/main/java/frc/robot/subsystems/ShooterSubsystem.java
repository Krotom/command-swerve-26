package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX feederMotor;

    private TalonFX shooterLeader;
    private TalonFX shooterFollower;

    private TalonFX shooterRingMotor;

    private TalonFX hoodMotor;

    public ShooterSubsystem() {
        feederMotor = new TalonFX(Constants.ShooterConstants.kFeederMotorID);

        shooterLeader = new TalonFX(Constants.ShooterConstants.kShooterLeaderID);
        shooterFollower = new TalonFX(Constants.ShooterConstants.kShooterFollowerID);

        shooterFollower.setControl(
            new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed)
        );

        shooterRingMotor = new TalonFX(Constants.ShooterConstants.kShooterRingMotorID);

        hoodMotor = new TalonFX(Constants.ShooterConstants.kHoodMotorID);
    }
}
