package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX deployMotor;
    private TalonFX intakeMotor;
    
    public IntakeSubsystem() {
        deployMotor = new TalonFX(IntakeConstants.kDeployMotorID);
        intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID);
    }
}
