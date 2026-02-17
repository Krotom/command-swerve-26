package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX deployMotor;
    private TalonFX intakeMotor;

    private boolean intakeState = false;
    
    public IntakeSubsystem() {
        deployMotor = new TalonFX(IntakeConstants.INTAKE_OPENER_MOTOR_ID);
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    }

    @Override
    public void periodic() {
        intake();
    }

    public void deployIntake() {
        // TODO stub, implement later
    }

    public void retractIntake() {
        // TODO stub, implement later
    }

    public void toggleIntake() {
        intakeState = !intakeState;
    }

    public void intake() {
        if (intakeState) {
            intakeMotor.set(1);
        } else {
            stop();
        }
    }

    public void outturn() {
        intakeMotor.set(-1);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
