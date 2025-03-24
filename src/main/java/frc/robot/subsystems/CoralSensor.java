package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CoralSensor implements Subsystem {
    private final LaserCan sensor;
    private final DoublePublisher publisher = NetworkTableInstance.getDefault().getDoubleTopic("").publish();

    public CoralSensor(int id) {
        sensor = new LaserCan(id);
        register();
    }

    @Override
    public void periodic() {
        var measurement = sensor.getMeasurement();

        if (measurement != null)
            publisher.accept(measurement.distance_mm);
    }
}
