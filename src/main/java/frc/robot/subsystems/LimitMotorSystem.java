package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitMotorSystem extends MotorSystem {
    private final DigitalInput limitSwitch;

    public LimitMotorSystem(int switchChannel, BiConsumer<Integer, TalonFXConfiguration> configure, double maxOutPercent, int leader, int... followers) {
        super(configure, maxOutPercent, leader, followers);
        limitSwitch = new DigitalInput(switchChannel);
    }

    @Override
    public void periodic() {
        super.periodic();

        // Each tick, check whether the limit switch has been hit.
        if (limitSwitch.get()) {
            // If the switch is hit while the system is running backwards, stop the motors.
            if (leader.getVelocity().getValueAsDouble() < 0.0) leader.setControl(cachedBrake);
            // TODO: reset the motor position to the switch position.
        }
    }
}
