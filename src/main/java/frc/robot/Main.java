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

import java.security.PrivateKey;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.util.WristAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CoralSensor;
import frc.robot.util.RateLimiter;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.LimitMotorSystem;
import frc.robot.subsystems.MotorSystem;

public class Main extends TimedRobot {



  private double L1 = 0;
  private double L2 = 15.6; //wrist position -0.086, 8.375 in from the reef
  private double L3 = 28;
  private double L4 = 43; 

  
  private double maxWristRotation = 0.13;
  

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
    c.MotionMagic.MotionMagicAcceleration = 150;
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

  public final AprilTags tags = new AprilTags("camera", new Transform3d(
      new Translation3d(Inches.of(15.0), Inches.zero(), Inches.of(4.25)),
      new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.zero())),
      drivetrain);

  public final Lidar lidar = new Lidar(drivetrain, new Transform2d(0.211, 0.165, Rotation2d.fromDegrees(-22.5)), null,
      0);

  public final LEDs leds = new LEDs(1);

  private Command m_autonomousCommand;
  private final SendableChooser<Command> autoChooser;

  public static void main(String... args) {
    RobotBase.startRobot(Main::new);
  }

  public Main() {
    DriverStation.silenceJoystickConnectionWarning(true);

    CanBridge.runTCP();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous path", autoChooser);

    NamedCommands.registerCommand("Elevator To Low", elevator.goTo(0.0));
    NamedCommands.registerCommand("Elevator At Low", elevator.atPoint(0.0));

    NamedCommands.registerCommand("Elevator To Mid", elevator.goTo(50.0));
    NamedCommands.registerCommand("Elevator At Mid", elevator.atPoint(50.0));

    NamedCommands.registerCommand("Eject Coral", coral.runAt(-1.0));

    StructPublisher<Pose2d> targetPose = NetworkTableInstance.getDefault()
        .getStructTopic("Lidar Target Pose", Pose2d.struct).publish();

    /* Default commands */
    // Switch to coast-mode once we're within deadband of the zero position.
    // elevator.setDefaultCommand(elevator.goTo(0.0).raceWith(elevator.atPoint(0.0)).andThen(elevator.coast()));
    wrist.setPosition(0);
    elevator.setPosition(0);
    // wrist.setDefaultCommand(wrist.goToStop(0.1577).andThen(wrist.brake()));


    drivetrain.registerTelemetry(logger::telemeterize);
    Locations.publish();


    /* Driver controls */
    RateLimiter xRate = new RateLimiter();
    RateLimiter yRate = new RateLimiter();
    SlewRateLimiter rRate = new SlewRateLimiter(30.0);

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          double limit = MathUtil.clamp(5.0 - 0.1 * elevator.position(), 3.0, 5.0);

          drive.VelocityX = xRate.calculate(-driver.getLeftY() * MaxSpeed, drive.VelocityX, limit);
          drive.VelocityY = yRate.calculate(-driver.getLeftX() * MaxSpeed, drive.VelocityY, limit);
          drive.RotationalRate = rRate.calculate(-driver.getRightX() * MaxAngularRate);

          return drive;
        }));

    // Lock directions to only forwards and backwards while the right stick is held.
    driver.rightStick().whileTrue(
        drivetrain.applyRequest(() -> robotCentric.withVelocityX(-driver.getLeftY() * MaxSpeed)));
    // PID tuning control while holding the left stick.
    driver.start().whileTrue(drivetrain.new DriveTo(Pose2d.kZero, Pose2d.kZero));

    // Brake mode.
    driver.x().whileTrue(drivetrain.applyRequest(() -> brake));
    // Reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driver.back().onTrue(drivetrain.runOnce(drivetrain::tareEverything));

    // Climber controls.
    driver.leftTrigger(0.5).whileTrue(climber.runAt(1));
    driver.rightTrigger(0.5).whileTrue(climber.runAt(-1));

    // Swerve.DriveTo cmd = drivetrain.new DriveTo(drivetrain.getState().Pose, Pose2d.kZero);

    WristAngle wristAngleCalc = new WristAngle(elevator, Locations.L4);

    // driver.start().whileTrue(cmd);

    /* Operator controls */
    // Command lidarTracking = lidar.subscribe(xform -> {
    //   wrist.setTarget(wristAngleCalc.calculate(xform.getTranslation()));
    // });

    // lidarTracking.addRequirements(wrist);

    // operator.x().whi   leTrue(lidarTracking);

    operator.x().whileTrue(
      wrist.goTo(() -> MathUtil.clamp(wristAngleCalc.calculate(Translation2d.kZero), -0.1125, 0.1372))
    );

    operator.y().whileTrue(
          elevator.goTo(43).alongWith(elevator.atPoint(43).andThen(wrist.goToStop(-0.086).alongWith(coral.runAt(1)).andThen(coral.runAt(-1).withTimeout(0.3))))        
    );

    // Wrist & elevator override controls while holding a & b.
    operator.a().whileTrue(wrist.runWith(() -> -operator.getLeftY()));
    operator.b().whileTrue(elevator.runWith(() -> -operator.getRightY()));

    // Control the coral receptacle with the right trigger & bumper.
    operator.rightTrigger(0.5).whileTrue(coral.runAt(1.0));
    operator.rightBumper().whileTrue(coral.runAt(-1.0));

    // Control the algae receptacle with the right trigger & bumper.
    operator.leftTrigger(
      0.5).whileTrue(algae.runAt(1.0));
    operator.leftBumper().whileTrue(algae.runAt(-1.0));

    //operator.a().whileTrue(elevator.goTo(L2));

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
