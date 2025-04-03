package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.core.CoreCANrange;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CoralSensor implements Subsystem {
    private final CoreCANrange sensor;
    private final StatusSignal<Distance> distance;
    private final DoublePublisher publisher = NetworkTableInstance.getDefault().getDoubleTopic("").publish();

    public CoralSensor(int id) {
        sensor = new CoreCANrange(id);
        distance = sensor.getDistance();
        register();
    }

    @Override
    public void periodic() {
        var measurement = distance.refresh().getValueAsDouble();
        publisher.accept(measurement);
    }
}
