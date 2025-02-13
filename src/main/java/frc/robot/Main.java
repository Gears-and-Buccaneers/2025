// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimitMotorSystem;
import frc.robot.subsystems.MotorSystem;

public class Main extends TimedRobot {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Initialise robot systems */
  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  public final LimitMotorSystem elevator = new LimitMotorSystem(9, (i, c) -> {
    c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    c.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // c.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // c.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5.0;

    // c.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // c.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 180.0;

    c.Slot0.kP = 30;
    c.Slot0.kD = 8;
    c.Slot0.kS = 5;
    c.Slot0.kG = 5;

    c.MotionMagic.MotionMagicCruiseVelocity = 9999;
    c.MotionMagic.MotionMagicAcceleration = 250;
  }, 1.0, 0.3, 25);
  public final MotorSystem wrist = new MotorSystem((i, c) -> {
    c.MotorOutput.DutyCycleNeutralDeadband = 0.25;
    c.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    c.Feedback.SensorToMechanismRatio = 125;

    c.Slot0.kP = 1000;
    c.Slot0.kD = 40;
    c.Slot0.kS = 40;
    c.Slot0.kG = 40;

    c.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    c.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
  }, 1.0, 0.5, 11);
  public final MotorSystem coral = new MotorSystem((i, c) -> {
    c.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
  }, 1.0, 0.15, 10);
  public final MotorSystem algae = new MotorSystem((i, c) -> {
    // Invert the first motor.
    if (i == 11)
      c.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }, 1.0, 0.3, 14);

  public final AprilTags tags = new AprilTags("camera", new Transform3d(
      new Translation3d(Inches.of(15.0), Inches.zero(), Inches.of(4.25)),
      new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.zero())),
      drivetrain);

  public final LEDs leds = new LEDs(1);

  private Command m_autonomousCommand;
  private final SendableChooser<Command> autoChooser;

  public static void main(String... args) {
    RobotBase.startRobot(Main::new);
  }

  public Main() {
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous path", autoChooser);

    SlewRateLimiter xRate = new SlewRateLimiter(5.0);
    SlewRateLimiter yRate = new SlewRateLimiter(5.0);
    SlewRateLimiter rRate = new SlewRateLimiter(15.0);

    NamedCommands.registerCommand("Elevator To Low", elevator.goTo(0.0));
    NamedCommands.registerCommand("Elevator At Low", elevator.atPoint(0.0));

    NamedCommands.registerCommand("Elevator To Mid", elevator.goTo(50.0));
    NamedCommands.registerCommand("Elevator At Mid", elevator.atPoint(50.0));

    NamedCommands.registerCommand("Eject Coral", coral.runAt(-1.0));

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

    // Brake mode.
    driver.x().whileTrue(drivetrain.applyRequest(() -> brake));

    // Run SysId routines when holding the back button.
    driver.back().onTrue(drivetrain.characterise());
    // Reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Snap to the nearest point-of-interest while holding POV buttons.
    driver.povUp().whileTrue(drivetrain.snapTo(Locations.cage));
    driver.povLeft().whileTrue(drivetrain.snapTo(Locations.withPredicate(Locations.reef, Locations.reefIsLeft)));
    driver.povRight().whileTrue(drivetrain.snapTo(Locations.withPredicate(Locations.reef, Locations.reefIsLeft.negate())));
    driver.povDown().whileTrue(drivetrain.snapTo(Locations.station));

    // Switch to coast-mode once we're within deadband of the zero position.
    elevator.setDefaultCommand(elevator.goTo(0.0).raceWith(elevator.atPoint(0.0)).andThen(elevator.coast()));
    // Manual, voltage-based override for the elevator.
    operator.povDown().whileTrue(elevator.runWith(() -> -operator.getRightY()));

    wrist.setDefaultCommand(wrist.runWith(() -> -operator.getLeftY()));
    coral.setDefaultCommand(coral.brake());
    algae.setDefaultCommand(algae.brake());

    // Control the coral receptacle with the right triggle & bumper.
    operator.rightTrigger(0.5).whileTrue(coral.runAt(1.0));
    operator.rightBumper().whileTrue(coral.runAt(-1.0));
    
    // Control the algae receptacle with the right triggle & bumper.
    operator.leftTrigger(0.5).whileTrue(algae.runAt(1.0));
    operator.leftBumper().whileTrue(algae.runAt(-1.0));

    drivetrain.registerTelemetry(logger::telemeterize);
    Locations.publish();
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
