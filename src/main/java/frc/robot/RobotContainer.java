// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<String> autoChooser;

    public RobotContainer() {
        autoChooser = new SendableChooser<String>();

        boolean first = true;

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            if (first) {
                autoChooser.setDefaultOption(autoName, autoName);
                first = false;
            } else {
                autoChooser.addOption(autoName, autoName);
            }
        }

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        NamedCommands.registerCommand("ShootAll", new Command() {}); // FIXME implement shoot all when finished

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty(
                    "Front Left Angle",
                    () -> drivetrain.getState().ModuleStates[0].angle.getRadians(),
                    null
                );
                builder.addDoubleProperty(
                    "Front Left Velocity",
                    () -> drivetrain.getState().ModuleStates[0].speedMetersPerSecond,
                    null
                );

                builder.addDoubleProperty(
                    "Front Right Angle",
                    () -> drivetrain.getState().ModuleStates[1].angle.getRadians(),
                    null
                );
                builder.addDoubleProperty(
                    "Front Right Velocity",
                    () -> drivetrain.getState().ModuleStates[1].speedMetersPerSecond,
                    null
                );

                builder.addDoubleProperty(
                    "Back Left Angle",
                    () -> drivetrain.getState().ModuleStates[2].angle.getRadians(),
                    null
                );
                builder.addDoubleProperty(
                    "Back Left Velocity",
                    () -> drivetrain.getState().ModuleStates[2].speedMetersPerSecond,
                    null
                );

                builder.addDoubleProperty(
                    "Back Right Angle",
                    () -> drivetrain.getState().ModuleStates[3].angle.getRadians(),
                    null
                );
                builder.addDoubleProperty(
                    "Back Right Velocity",
                    () -> drivetrain.getState().ModuleStates[3].speedMetersPerSecond,
                    null
                );

                builder.addDoubleProperty(
                    "Robot Angle",
                    () -> drivetrain.getState().Pose.getRotation().getRadians(),
                    null
                );
            }
        });

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));

        driverJoystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverJoystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        new EventTrigger("EnableIntake").onTrue(new PrintCommand("Intake Deployed")).whileTrue(new Command() {}).onFalse(new PrintCommand("Intake Retracted"));

        // FIXME add the intake deployment, retraction and enabling commands here when finished
    }

    public Command getAutonomousCommand() {
        try {
            System.out.println("Fetching auto: " + autoChooser.getSelected());
            PathPlannerPath path = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected()).get(0);
            PathConstraints constraint = new PathConstraints(3.0, 3.0, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0));
            System.out.println("Got path to fetch starting position. Position: " + path.getStartingHolonomicPose().orElseThrow());
            Command comm;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                comm = AutoBuilder.pathfindToPoseFlipped(path.getStartingHolonomicPose().orElseThrow(), constraint, 3.0);
            } else {
                comm = AutoBuilder.pathfindToPose(path.getStartingHolonomicPose().orElseThrow(), constraint, 3.0);
            }
            return comm.andThen(new PathPlannerAuto(autoChooser.getSelected()));
        } catch (Exception e) {
            System.out.println("Failed to load start position from auto: " + e.getMessage());
            e.printStackTrace();
            return new PathPlannerAuto(autoChooser.getSelected());
        }
    }
}
