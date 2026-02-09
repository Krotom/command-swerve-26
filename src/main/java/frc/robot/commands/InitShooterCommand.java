// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InitShooterCommand extends WaitUntilCommand {
  /** Creates a new InitShooterCommand. */
  ShooterSubsystem m_ShooterSubsystem;
  public InitShooterCommand(ShooterSubsystem m_ShooterSubsystem) {
    super(() -> m_ShooterSubsystem.atSetpoint());
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.spinShooterToLife();
  }
}
