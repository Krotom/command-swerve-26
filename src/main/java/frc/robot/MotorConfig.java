// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;

public class MotorConfig {
    public MotorConfig(){}

    public TalonFXConfiguration configTurret(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT; 

        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.CRUISE_VELOCITY; 
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.ACCERELATION; 
        config.MotionMagic.MotionMagicJerk = TurretConstants.JERK;

        config.Slot0.kS = TurretConstants.kS;
        config.Slot0.kV = TurretConstants.kV;
        config.Slot0.kA = TurretConstants.kA;

        config.Slot0.kP = TurretConstants.kP;
        config.Slot0.kI = TurretConstants.kI;
        config.Slot0.kD = TurretConstants.kD;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.FORWARD_LIMIT; 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = TurretConstants.FORWARD_LIMIT_ENABLED;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.REVERSE_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = TurretConstants.REVERSE_LIMIT_ENABLED;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;
        return config;
    }

    public TalonFXConfiguration configIntakeOpener(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT; 

        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY; 
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCERELATION; 
        config.MotionMagic.MotionMagicJerk = IntakeConstants.JERK;

        config.Slot0.kS = IntakeConstants.kS;
        config.Slot0.kV = IntakeConstants.kV;
        config.Slot0.kA = IntakeConstants.kA;

        config.Slot0.kP = IntakeConstants.kP;
        config.Slot0.kI = IntakeConstants.kI;
        config.Slot0.kD = IntakeConstants.kD;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.FORWARD_LIMIT; 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = IntakeConstants.FORWARD_LIMIT_ENABLED;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.REVERSE_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = IntakeConstants.REVERSE_LIMIT_ENABLED;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;
        return config;
    }

    public TalonFXConfiguration configHood(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT; 

        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.CRUISE_VELOCITY; 
        config.MotionMagic.MotionMagicAcceleration = HoodConstants.ACCERELATION; 
        config.MotionMagic.MotionMagicJerk = HoodConstants.JERK;

        config.Slot0.kS = HoodConstants.kS;
        config.Slot0.kV = HoodConstants.kV;
        config.Slot0.kA = HoodConstants.kA;

        config.Slot0.kP = HoodConstants.kP;
        config.Slot0.kI = HoodConstants.kI;
        config.Slot0.kD = HoodConstants.kD;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.FORWARD_LIMIT; 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = HoodConstants.FORWARD_LIMIT_ENABLED;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.REVERSE_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = HoodConstants.REVERSE_LIMIT_ENABLED;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
        return config;
    }

    public InterpolatingDoubleTreeMap configHubAngleMap(){
        InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

        angleMap.put(1.5, 28.0);
        angleMap.put(2.0, 32.5); 
        angleMap.put(3.0, 38.0);
        angleMap.put(4.0, 43.0);
        angleMap.put(5.5, 48.0);

        return angleMap;
    }

    public InterpolatingDoubleTreeMap configFeederAngelMap(){
        InterpolatingDoubleTreeMap angleMap2Feeder = new InterpolatingDoubleTreeMap();
 
        angleMap2Feeder.put(1.5, 18.0);
        angleMap2Feeder.put(2.0, 28.0);
        angleMap2Feeder.put(3.0, 35.0);
        angleMap2Feeder.put(4.0, 40.0);
        angleMap2Feeder.put(5.5, 45.0);

        return angleMap2Feeder;
    }
}