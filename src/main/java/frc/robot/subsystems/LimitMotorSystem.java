package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitMotorSystem extends MotorSystem {
    private final DigitalInput limitSwitch;

    public LimitMotorSystem(int switchChannel, BiConsumer<Integer, TalonFXConfiguration> configure,
            double deadband, double maxOutPercent, int... ids) {
        super(configure, deadband, maxOutPercent, ids);
        limitSwitch = new DigitalInput(switchChannel);
    }

    @Override
    public void periodic() {
        super.periodic();

        // Each tick, check whether the limit switch has been hit.
        if (limitSwitch.get())
            for (TalonFX motor : motors)
                motor.setPosition(0.0);
    }
}
