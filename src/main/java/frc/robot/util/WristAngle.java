package frc.robot.util;

import frc.robot.subsystems.MotorSystem;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class WristAngle implements DoubleSupplier {
    // The number of meters that the elevator rises per rotation of the motor.
    private static final double elevatorRatio = 20.0 / 58.15;
    // The number of meters high that the wrist is at when the elevator is at its lowest position.
    private static final double elevatorBase = 23.0;
    // The transformation from the robot origin to the wrist.
    private static final Translation2d robotToWrist = new Translation2d();

    // The systems whose positions are read to calculate angle.
    private final MotorSystem elevator;
    private final Swerve drivetrain;

    // The wrist target position.
    private Translation2d target;
    private double targetHeight;

    public WristAngle(MotorSystem elevator, Swerve drivetrain, Translation2d target, double targetHeight) {
        this.elevator = elevator;
        this.drivetrain = drivetrain;
        this.target = target;
        this.targetHeight = targetHeight;
    }

    @Override
    public double getAsDouble() {
        // Find the height of the wrist in meters.
        var wristHeight = elevator.position() * elevatorRatio + elevatorBase;
        // Take the difference to find the vertical distance to the target.
        var vdist = targetHeight - wristHeight;

        // Find the position of the wrist as a 2D field-relative position.
        var wrist = drivetrain.getState().Pose.getTranslation().plus(robotToWrist);
        // Take the difference to find the horizontal distance to the target.
        var hdist = target.getDistance(wrist);

        // Find the angle to point the wrist at.
        return Units.radiansToRotations(Math.atan2(vdist, hdist));
    }
}
