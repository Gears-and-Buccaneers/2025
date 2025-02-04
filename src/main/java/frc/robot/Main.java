// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.LimitMotorSystem;
import frc.robot.subsystems.MotorSystem;

public class Main extends TimedRobot {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Initialise robot systems */
  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  public final MotorSystem elevator = new LimitMotorSystem(9, (i, c) -> {
    c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    c.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    c.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    c.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5.0;

    c.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    c.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 180.0;

    c.Slot0.kP = 48;
    c.Slot0.kD = 3;
    c.Slot0.kS = 4.7;
    c.Slot0.kG = 5.625;

    c.MotionMagic.MotionMagicAcceleration = 9999;
    c.MotionMagic.MotionMagicAcceleration = 180;
  }, 0.3, 25);
  public final MotorSystem wrist = new MotorSystem((i, c) -> {
    c.MotorOutput.DutyCycleNeutralDeadband = 0.05;
    c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }, 0.3, 11);
  public final MotorSystem coral = new MotorSystem((i, c) -> {
  }, 0.3, 10);
  public final MotorSystem algae = new MotorSystem((i, c) -> {
    // Invert the first motor.
    if (i == 11)
      c.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }, 0.3, 30, 37);

  public final AprilTags tags = new AprilTags("camera", new Transform3d(), drivetrain);

  private Command m_autonomousCommand;
  private final SendableChooser<Command> autoChooser;

  public static void main(String... args) {
    RobotBase.startRobot(Main::new);
  }

  public Main() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous path", autoChooser);

    SlewRateLimiter xRate = new SlewRateLimiter(5.0);
    SlewRateLimiter yRate = new SlewRateLimiter(5.0);
    SlewRateLimiter rRate = new SlewRateLimiter(5.0);

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
        // Drive forward with negative Y (forward)
        drive.withVelocityX(xRate.calculate(-driver.getLeftY() * MaxSpeed))
            // Drive left with negative X (left)
            .withVelocityY(yRate.calculate(-driver.getLeftX() * MaxSpeed))
            // Drive counterclockwise with negative X (left)
            .withRotationalRate(rRate.calculate(-driver.getRightX() * MaxAngularRate))));

    elevator.setDefaultCommand(elevator.runWithLimit(() -> operator.getRightY(), 0.01));
    wrist.setDefaultCommand(wrist.runWith(() -> operator.getLeftY()));
    coral.setDefaultCommand(coral.brake());
    algae.setDefaultCommand(algae.brake());

    operator.rightTrigger(0.5).whileTrue(coral.runAt(1.0));
    operator.leftTrigger(0.5).whileTrue(algae.runAt(1.0));

    operator.rightBumper().whileTrue(coral.runAt(-1.0));
    operator.leftBumper().whileTrue(algae.runAt(-1.0));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // Run SysId routines when holding back button.
    driver.back().onTrue(drivetrain.characterise());
    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
