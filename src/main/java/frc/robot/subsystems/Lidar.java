package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Lidar extends Command {
    // The drivetrain, which provides pose data.
    private final Swerve drivetrain;
    // The subscribers to the lidar data feed.
    private final List<Consumer<Transform2d>> subscribers;
    // The transformation from the robot origin to the lidar coordinate origin.
    private final Transform2d robotToLidar;
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

    public Lidar(Swerve drivetrain, Transform2d robotToLidar, String path, int baudRate) {
        this.drivetrain = drivetrain;
        this.robotToLidar = robotToLidar;
        this.subscribers = new ArrayList<>();
        // Construct the rplidar instance.
        this.statePtr = construct();
    }

    public Command subscribe(Consumer<Transform2d> subscriber) {
        return new Command() {
            @Override
            public void initialize() {
                if (subscribers.contains(subscriber)) return;

                if (subscribers.isEmpty()) Lidar.this.schedule();
                subscribers.add(subscriber);
            }

            @Override
            public void end(boolean interrupted) {
                subscribers.remove(subscriber);
                if (subscribers.isEmpty()) Lidar.this.cancel();
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    // Construct a new lidar driver.
    private native long construct();
    // Starts the lidar motor.
    public native void startMotor();
    // Starts the lidar scanning operation.
    @Override
    public native void initialize();
    // Polls the lidar sensor for scan data.
    @Override
    public native void execute();
    // Stops the lidar scanning operation.
    @Override
    public native void end(boolean interrupted);
    // Stops the lidar motor.
    public native void stopMotor();

    // The data callback for line detections.
    private void callback(Transform2d lidarToCenter) {
        Transform2d robotToCenter = robotToLidar.plus(lidarToCenter);
        for (var subscriber : subscribers) subscriber.accept(robotToCenter);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
