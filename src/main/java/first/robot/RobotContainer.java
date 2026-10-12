// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.*;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.robot.generated.TunerConstants;
import first.robot.subsystems.CommandSwerveDrivetrain;
import first.robot.subsystems.CommandSwerveDrivetrain.SysIdDirection;
import first.robot.subsystems.ElevatorSubsystem;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.LimelightSubsystem;
import first.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private static final double ELEVATOR_MANUAL_DUTY_CYCLE = 0.3;
    private static final double SHOOTER_TARGET_RPM = 3000.0; // TODO: Set the desired shooter speed.

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.ApplyFieldVelocity autoDrive = new SwerveRequest.ApplyFieldVelocity()
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandGamepad operatorController = new CommandGamepad(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final LimelightSubsystem limelight = new LimelightSubsystem(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back (view)/start (menu) and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.view().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(SysIdDirection.FORWARD));
        joystick.view().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(SysIdDirection.REVERSE));
        joystick.menu().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdDirection.FORWARD));
        joystick.menu().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdDirection.REVERSE));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(
            drivetrain.run(coroutine -> drivetrain.seedFieldCentric()).named("Seed Field Centric")
        );

        // Press once to run the elevator; press the same control again to stop it.
        operatorController.dpadUp().toggleOnTrue(elevator.set(ELEVATOR_MANUAL_DUTY_CYCLE));
        operatorController.dpadDown().toggleOnTrue(elevator.set(-ELEVATOR_MANUAL_DUTY_CYCLE));
        operatorController.faceRight().toggleOnTrue(elevator.set(ELEVATOR_MANUAL_DUTY_CYCLE));

        // Hold the right bumper to run the shooter; releasing it removes power and coasts.
        operatorController.rightBumper().whileTrue(
            shooter.run(RPM.of(SHOOTER_TARGET_RPM))
        );

        // Hold the left bumper to run the indexer; releasing it stops the motor.
        operatorController.leftBumper().whileTrue(indexer.run());

        // Press face-up to replace odometry with the next accepted Limelight pose.
        operatorController.faceUp().onTrue(
            limelight.run(coroutine -> limelight.requestPoseReset()).named("Request Vision Pose Reset")
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Command.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.run(coroutine -> drivetrain.seedFieldCentric(Rotation2d.ZERO))
                .named("Seed Autonomous Heading"),
            // Then slowly drive forward (away from us) for 2 seconds.
            drivetrain.applyRequest(() ->
                autoDrive.withVelocity(
                    new ChassisVelocities(1.0, 0.0, 0.0)
                )
            )
            .withTimeout(Seconds.of(2)),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        ).named("Simple Drive Forward Autonomous");
    }
}
