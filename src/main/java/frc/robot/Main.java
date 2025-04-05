// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import au.grapplerobotics.CanBridge;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CoralSensor;
import frc.robot.util.Level;
import frc.robot.util.RateLimiter;
import frc.robot.util.Toggle;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.MotorSystem;

public class Main extends TimedRobot {
  private Level target = Level.L4;

  private double intakePosition = .138;
  private double maxWristRotation = 0.115;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Initialise robot systems */
  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  public final MotorSystem elevator = new MotorSystem((i, c) -> {
    c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    c.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    c.Slot0.kP = 80;
    c.Slot0.kD = 4;
    c.Slot0.kS = 3;
    c.Slot0.kG = 17;

    c.MotionMagic.MotionMagicCruiseVelocity = 9999;
    c.MotionMagic.MotionMagicAcceleration = 80;
    c.MotionMagic.MotionMagicJerk = 600;
  }, 1.0, 0.3, 25, 26);
  public final MotorSystem wrist = new MotorSystem((i, c) -> {
    c.MotorOutput.DutyCycleNeutralDeadband = 0.25;
    c.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    c.Feedback.RotorToSensorRatio = 125;
    c.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    c.Feedback.FeedbackRemoteSensorID = 0;

    c.Slot0.kP = 2000;
    c.Slot0.kD = 200;
    c.Slot0.kS = 20;
    c.Slot0.kG = 10;

    c.MotionMagic.MotionMagicCruiseVelocity = 9999;
    c.MotionMagic.MotionMagicAcceleration = 4;

    c.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
  }, 0.01, 0.5, 12);
  public final MotorSystem coral = new MotorSystem((i, c) -> {
    c.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
  }, 1.0, 0.15, 10);
  public final MotorSystem algae = new MotorSystem((i, c) -> {
    // Invert the first motor.
    if (i == 11)
      c.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }, 1.0, 0.3, 14);
  public final MotorSystem climber = new MotorSystem((i, c) -> {

  }, 0.0, 0.5, 11);

  public final CoralSensor coralSensor = new CoralSensor(0);

  public final AprilTags lower = new AprilTags("camera", new Transform3d(
      new Translation3d(Inches.of(15.0 - 4.75), Inches.of(-15.0 + 2.375), Inches.zero()), Rotation3d.kZero),
      drivetrain);

  public final AprilTags upper = new AprilTags("camera1", new Transform3d(
    new Translation3d(Inches.of(15.0 - 8.5 + 1.75), Inches.of(-15.0 + 2 + 0.875), Inches.of(40.125)), new Rotation3d(Degrees.of(180), Degrees.of(36), Degrees.zero())),
    drivetrain);

  public final Lidar lidar = new Lidar(drivetrain, new Transform2d(0.211, 0.165, Rotation2d.kZero));

  public final LEDs leds = new LEDs(1);

  private Command m_autonomousCommand;
  private final SendableChooser<Command> autoChooser;

  public void updateLED() {
    leds.setColor(target.color);
  }

  public Command setTarget(Supplier<Level> level) {
    return new InstantCommand(() -> {
      target = level.get();
      updateLED();
    });
  }

  /** Elevator and Wrist without Drive Train **/
  public Command toTargetHeight() {
    return Commands.deferredProxy(
        () -> elevator.goTo(target.height).alongWith(elevator.atPoint(target.height)
            .andThen(
                wrist.goToStop(-0.086).alongWith(coral.runAt(1.0).asProxy()))));
    /*
     * ,
     * coral.runAt(-1).withTimeout(0.7),
     * wrist.goToStop(.1)
     */
  }

  Toggle elevatorToggle = new Toggle(
      elevator.goTo(0.0).raceWith(elevator.atPoint(0.0)).andThen(elevator.coast()),
      elevator.runWith(() -> -operator.getRightY()),
      operator.rightStick());
  Toggle wristToggle = new Toggle(
      wrist.goToStop(0.11).andThen(wrist.brake()),
      wrist.runWith(() -> -operator.getLeftY()),
      operator.leftStick());

  public static void main(String... args) {
    RobotBase.startRobot(Main::new);
  }

  public Main() {
    DriverStation.silenceJoystickConnectionWarning(true);

    CanBridge.runTCP();

    Command l4 = Commands.defer(
        () -> wrist.goToStop(0.11)
          .andThen(
            elevator.goTo(target.height)
              .raceWith(
                drivetrain.snapTo(Locations.branches)
                .andThen(
                    drivetrain.applyRequest(() -> brake)
                      .raceWith(
                        elevator.atPoint(target.height)
                          .andThen(
                            wrist.goToStop(-0.086)
                              .raceWith(coral.runAt(1.0)),
                            coral.runAt(-1)
                              .raceWith(
                                wrist.brake().withTimeout(1)
                                  .andThen(wrist.goToStop(.1))
                              )
                          )
                      )
                  )
              )
          ).andThen(new InstantCommand(() -> leds.setColor(0xFF0000))),
          Set.of(elevator, wrist, drivetrain, coral)
      ).andThen(coral.brake().raceWith(elevator.goToStop(0)));

    Command intake = drivetrain.snapTo(Locations.station).andThen(coral.runAt(1.0)
        .alongWith(wrist.goToStop(intakePosition)).until(coralSensor::hasCoral));

    NamedCommands.registerCommand("L4", l4);
    NamedCommands.registerCommand("Intake", intake);

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Leave", drivetrain.applyRequest(() -> robotCentric.withVelocityX(0.3)));
    SmartDashboard.putData("Autonomous path", autoChooser);

    StructPublisher<Pose2d> targetPose = NetworkTableInstance.getDefault()
        .getStructTopic("Lidar Target Pose", Pose2d.struct).publish();

    /* Default commands */
    // Switch to coast-mode once we're within deadband of the zero position.+
    drivetrain.registerTelemetry(logger::telemeterize);
    Locations.publish();

    /* Driver controls */
    RateLimiter xRate = new RateLimiter();
    RateLimiter yRate = new RateLimiter();
    SlewRateLimiter rRate = new SlewRateLimiter(30.0);

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          double limit = MathUtil.clamp(10.0 - 0.1 * elevator.position(), 3.0, 5.0);

          if (driver.rightTrigger().getAsBoolean()) {
            drive.VelocityX = -driver.getLeftY() * MaxSpeed;
            drive.VelocityY = -driver.getLeftX() * MaxSpeed;
            drive.RotationalRate = -driver.getRightX() * MaxAngularRate;
          } else {
            drive.VelocityX = xRate.calculate(-driver.getLeftY() * MaxSpeed, drive.VelocityX, limit);
            drive.VelocityY = yRate.calculate(-driver.getLeftX() * MaxSpeed, drive.VelocityY, limit);
            drive.RotationalRate = rRate.calculate(-driver.getRightX() * MaxAngularRate);
          }

          return drive;
        }));

    // Lock directions to only forwards and backwards while the right stick is held.
    driver.rightStick().whileTrue(
        drivetrain.applyRequest(() -> robotCentric.withVelocityX(-driver.getLeftY() * MaxSpeed)));

    // Brake mode.
    driver.x().whileTrue(drivetrain.applyRequest(() -> brake));
    // Reset the field-centric heading on left bumper press
    driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Climber controls.
    driver.rightBumper().whileTrue(climber.runAt(1));
    driver.rightTrigger(0.5).whileTrue(climber.runAt(-1));

    // driver.a().whileTrue(
    // new DeferredCommand(() -> {
    // Pose2d target = drivetrain.nearest(Locations.reef);
    // DriveTo driveTo = drivetrain.new DriveTo(target, Pose2d.kZero);

    // return lidar.new Subscription(d -> {
    // var curr = drivetrain.getState().Pose;
    // var diff = target.minus(curr);
    // driveTo.setDestination(curr.plus(new Transform2d(d +
    // lidar.robotToLidar.getX(), diff.getY(), diff.getRotation())).plus(new
    // Transform2d(-0.64, -(Locations.branchOffset - Locations.coralOffset),
    // Rotation2d.kZero)), Pose2d.kZero);
    // driveTo.schedule();
    // });
    // }, Set.of())
    // );

    // driver.a().whileTrue(new ParallelCommandGroup(
    // drivetrain.snapTo(Locations.branches)
    // ));

    driver.a().whileTrue(l4);
    driver.b().whileTrue(intake);

    // Swerve.DriveTo cmd = drivetrain.new DriveTo(drivetrain.getState().Pose,
    // Pose2d.kZero);

    // WristAngle wristAngleCalc = new WristAngle(elevator, Locations.L4);

    // driver.start().whileTrue(cmd);

    /* Operator controls */
    elevator.setDefaultCommand(elevatorToggle);
    wrist.setDefaultCommand(wristToggle);
    elevator.setPosition(0);
    // Lock elevator, but manual wrist
    operator.a().whileTrue(new DeferredCommand(() -> elevator.goTo(elevator.position()), Set.of(elevator))
        .alongWith(wrist.runWith(() -> -operator.getLeftY())));

    operator.b().whileTrue(toTargetHeight());

    operator.x().whileTrue(climber.runAt(-1));

    // Level Shifter
    operator.povUp().or(operator.povUpLeft()).or(operator.povUpRight()).debounce(0.1, DebounceType.kFalling)
        .onTrue(setTarget(() -> target.next()).ignoringDisable(true));
    operator.povDown().or(operator.povDownLeft()).or(operator.povDownRight()).debounce(0.1, DebounceType.kFalling)
        .onTrue(setTarget(() -> target.prev()).ignoringDisable(true));

    // Control the coral receptacle with the right trigger & bumper.
    operator.rightTrigger(0.5).whileTrue(coral.runAt(-1.0));
    operator.rightBumper().whileTrue(coral.runAt(1.0).alongWith(wrist.goToStop(intakePosition)));

    // Control the algae receptacle with the right trigger & bumper.
    operator.leftTrigger(0.5).whileTrue(algae.runAt(1.0));
    operator.leftBumper().whileTrue(algae.runAt(-1.0));

    // operator.a().whileTrue(elevator.goTo(L2));
  }

  @Override
  public void robotInit() {
    leds.setColor(0x000000);
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
    elevatorToggle.reset();
    wristToggle.reset();

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

    lidar.startMotor();
  }

  NetworkTableInstance nt = NetworkTableInstance.getDefault();

  DoublePublisher elevatorHeight = nt.getDoubleTopic("Elevator Position").publish();
  DoublePublisher wristHeight = nt.getDoubleTopic("Wrist Position").publish();

  @Override
  public void teleopPeriodic() {
    elevatorHeight.set(elevator.position());
    wristHeight.set(wrist.position());
  }

  @Override
  public void teleopExit() {
    lidar.stopMotor();
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
