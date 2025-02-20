package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    public final PathConstraints pathConstraints = new PathConstraints(3.0, 3.0, Math.toRadians(540.0),
            Math.toRadians(720.0));

    // PID constants for translation
    private final PIDConstants translation = new PIDConstants(10, 0, 0);
    private final Constraints translationConstraints = new Constraints(pathConstraints.maxVelocityMPS(),
            pathConstraints.maxAccelerationMPSSq());
    // PID constants for rotation
    private final PIDConstants rotation = new PIDConstants(7, 0, 0);
    private final Constraints rotationConstraints = new Constraints(pathConstraints.maxAngularVelocityRadPerSec(),
            pathConstraints.maxAngularAccelerationRadPerSecSq());

    // Profiled PID controllers for direct control
    private final ProfiledPIDController xPID = new ProfiledPIDController(translation.kP, translation.kI, translation.kD,
            translationConstraints);
    private final ProfiledPIDController yPID = new ProfiledPIDController(translation.kP, translation.kI, translation.kD,
            translationConstraints);
    private final ProfiledPIDController rPID = new ProfiledPIDController(rotation.kP, rotation.kI, rotation.kD,
            rotationConstraints);

    private double translationDeadband = 0.02;
    private double rotationDeadband = Math.toRadians(5);

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        rPID.enableContinuousInput(-Math.PI, Math.PI);

        try {
            var config = RobotConfig.fromGUISettings();
            var applySpeeds = new SwerveRequest.ApplyRobotSpeeds();

            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            applySpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(translation, rotation),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /** Returns a Command which drives directly to the specified pose. */
    public Command driveTo(Pose2d destination, Pose2d velocity) {
        return new Command() {
            @Override
            public void initialize() {
                var state = getState();

                xPID.reset(state.Pose.getX(), state.Speeds.vxMetersPerSecond);
                yPID.reset(state.Pose.getY(), state.Speeds.vyMetersPerSecond);
                rPID.reset(MathUtil.angleModulus(state.Pose.getRotation().getRadians()),
                        state.Speeds.omegaRadiansPerSecond);

                xPID.setGoal(new TrapezoidProfile.State(destination.getX(), velocity.getX()));
                yPID.setGoal(new TrapezoidProfile.State(destination.getY(), velocity.getY()));
                rPID.setGoal(new TrapezoidProfile.State(
                        MathUtil.angleModulus(destination.getRotation().getRadians()),
                        MathUtil.angleModulus(velocity.getRotation().getRadians())));
            }

            @Override
            public void execute() {
                var pose = getState().Pose;

                if (Math.abs(pose.getX() - destination.getX()) > translationDeadband)
                    fieldCentric.VelocityX = xPID.calculate(pose.getX());
                if (Math.abs(pose.getY() - destination.getY()) > translationDeadband)
                    fieldCentric.VelocityY = yPID.calculate(pose.getY());
                if (Math.abs(
                        pose.getRotation().getRadians() - destination.getRotation().getRadians()) > rotationDeadband)
                    fieldCentric.RotationalRate = xPID.calculate(pose.getRotation().getRadians());

                setControl(fieldCentric);
            }

            @Override
            public boolean isFinished() {
                var pose = getState().Pose;

                return Math.abs(pose.getX() - destination.getX()) <= translationDeadband
                        && Math.abs(pose.getY() - destination.getY()) <= translationDeadband
                        && Math.abs(pose.getRotation().getRadians()
                                - destination.getRotation().getRadians()) <= rotationDeadband;
            }
        };
    }

    /**
     * Returns a command which snaps the drivetrain to the closest of the provided
     * points of interest
     */
    public Command snapTo(Pose2d[] poi) {
        return defer(() -> {
            Translation2d robot = getState().Pose.getTranslation();

            double minDist = Double.POSITIVE_INFINITY;
            Pose2d dest = null;

            for (Pose2d p : poi) {
                double dist = p.getTranslation().getDistance(robot);

                if (dist < minDist) {
                    minDist = dist;
                    dest = p;
                }
            }

            if (dest == null)
                return new Command() {
                };

            return driveTo(dest, Pose2d.kZero);
        });
    }

    /** Runs SysId characterisations for all axes in all directions. */
    public Command characterise() {
        /* Swerve requests to apply during SysId characterization */
        SwerveRequest.SysIdSwerveTranslation translation = new SwerveRequest.SysIdSwerveTranslation();
        SwerveRequest.SysIdSwerveSteerGains steer = new SwerveRequest.SysIdSwerveSteerGains();
        SwerveRequest.SysIdSwerveRotation rotation = new SwerveRequest.SysIdSwerveRotation();

        SysIdRoutine[] routines = {
                /*
                 * SysId routine for characterizing translation. This is used to find PID gains
                 * for the drive motors.
                 */
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null, // Use default ramp rate (1 V/s)
                                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                                Seconds.of(5), // Use default timeout (10 s)
                                // Log state with SignalLogger class
                                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
                        new SysIdRoutine.Mechanism(
                                output -> setControl(translation.withVolts(output)),
                                null,
                                this)),

                /*
                 * SysId routine for characterizing steer. This is used to find PID gains for
                 * the steer motors.
                 */
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null, // Use default] ramp rate (1 V/s)
                                Volts.of(7), // Use dynamic voltage of 7 V
                                null, // Use default timeout (10 s)
                                // Log state with SignalLogger class
                                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts -> setControl(steer.withVolts(volts)),
                                null,
                                this)),

                /*
                 * SysId routine for characterizing rotation.
                 * This is used to find PID gains for the FieldCentricFacingAngle
                 * HeadingController.
                 * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
                 * importing the log to SysId.
                 */
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                /* This is in radians per second², but SysId only supports "volts per second" */
                                Volts.of(Math.PI / 6).per(Second),
                                /* This is in radians per second, but SysId only supports "volts" */
                                Volts.of(Math.PI),
                                null, // Use default timeout (10 s)
                                // Log state with SignalLogger class
                                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
                        new SysIdRoutine.Mechanism(
                                output -> {
                                    /* output is actually radians per second, but SysId only supports "volts" */
                                    setControl(rotation.withRotationalRate(output.in(Volts)));
                                    /* also log the requested output for SysId */
                                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                                },
                                null,
                                this))
        };

        SequentialCommandGroup executeAll = new SequentialCommandGroup();

        for (SysIdRoutine routine : routines) {
            executeAll.addCommands(
                    routine.dynamic(Direction.kForward),
                    routine.dynamic(Direction.kReverse),
                    routine.quasistatic(Direction.kForward),
                    routine.quasistatic(Direction.kReverse));
        }

        return executeAll;
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
