package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitMotorSystem extends MotorSystem {
    private final DigitalInput limitSwitch;

    public LimitMotorSystem(int switchChannel, BiConsumer<Integer, TalonFXConfiguration> configure,
            double maxOutPercent, int... ids) {
        super(configure, maxOutPercent, ids);
        limitSwitch = new DigitalInput(switchChannel);
    }

    @Override
    public void periodic() {
        super.periodic();

        // Each tick, check whether the limit switch has been hit.
        if (limitSwitch.get()) {
            // TODO: reset the motor positions to the switch position.

            for (TalonFX motor : motors)
                // If the switch is hit while any motors are running backwards, stop all of the motors.
                if (motor.getVelocity().getValueAsDouble() < 0.0) {
                    motors[0].setControl(cachedBrake);
                    break;
                }
        }
    }
}
