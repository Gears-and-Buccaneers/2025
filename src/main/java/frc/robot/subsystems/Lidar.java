package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Swerve.DriveTo;

public class Lidar implements Subsystem {
    // The drivetrain, which provides pose data.
    private final Swerve drivetrain;
    // A native pointer to the driver state.
    private final long statePtr;

    static {
        System.out.println("Loading lidar native library...");

        try {
            System.load(Filesystem.getDeployDirectory() + "/liblidar.so");
            System.out.println("Successfully loaded lidar native library!");
        } catch (Exception e) {
            DriverStation.reportError("Could not load lidar native library: " + e, false);
        }
    }

    public Lidar(Swerve drivetrain, String path, int baudRate) {
        this.drivetrain = drivetrain;
        // Construct the rplidar instance.
        this.statePtr = construct();

        System.out.println("statePtr: " + statePtr);
    }

    // Construct a new lidar driver.
    private native long construct();
    // Starts the lidar motor.
    public native void startMotor();
    // Starts the lidar scanning operation.
    public native void startScan();
    // Polls the lidar sensor for scan data.
    public native void poll(Consumer<Transform2d> consumer);
    // Stops the lidar scanning operation.
    public native void stopScan();
    // Stops the lidar motor.
    public native void stopMotor();

    public class FeedPose extends Command {
        private final Consumer<Transform2d> consumer;

        public FeedPose(Consumer<Transform2d> consumer) {
            addRequirements(Lidar.this);
            this.consumer = consumer;
        }

        @Override
        public void initialize() {
            startScan();
        }

        @Override
        public void execute() {
            poll(consumer);
        }

        @Override
        public void end(boolean interrupted) {
            stopScan();
        }
    }
}
