package frc.robot.util;

import frc.robot.subsystems.MotorSystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class WristAngle {
    // The number of inches that the elevator rises per rotation of the motor.
    private static final double elevatorRatio = 1.328;
    // The number of inches high that the wrist is at when the elevator is at its lowest position.
    private static final double elevatorBase = 19.17;
    // The transformation from the robot origin to the wrist.
    private static final Translation2d robotToWrist = new Translation2d();

    // The systems whose positions are read to calculate angle.
    private final MotorSystem elevator;

    // The wrist target position.
    private double targetHeight;

    public WristAngle(MotorSystem elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
    }

    public double calculate(Translation2d target) {
        // Find the height of the wrist in meters.
        var wristHeight = elevator.position() * elevatorRatio + elevatorBase;
        // Take the difference to find the vertical distance to the target.
        var vdist = targetHeight - wristHeight;
        // Take the difference to find the horizontal distance to the target.
        var hdist = 20.0;
        // Find the angle to point the wrist at.
        return Units.radiansToRotations(Math.atan2(vdist, hdist) - Math.asin(3.125 / Math.hypot(hdist, vdist)));
    }
}
