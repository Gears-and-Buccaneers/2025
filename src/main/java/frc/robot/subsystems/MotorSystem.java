package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorSystem implements Subsystem {
    protected final TalonFX leader;
    protected final double voltageMax;

    protected final  VoltageOut cachedVout  = new VoltageOut(0);
    protected final StaticBrake cachedBrake = new StaticBrake();

    public MotorSystem(BiConsumer<Integer, TalonFXConfiguration> configure, double maxOutPercent, int leader, int... followers) {
        this.leader = new TalonFX(leader);
        this.voltageMax = maxOutPercent * 12.0;

        TalonFXConfiguration config = new TalonFXConfiguration();
        configure.accept(leader, config);
        this.leader.getConfigurator().apply(config);

        for (int id : followers) {
            config = new TalonFXConfiguration();
            configure.accept(id, config);

            TalonFX motor = new TalonFX(id);

            motor.getConfigurator().apply(config);
            motor.setControl(new StrictFollower(leader));

            motor.close();
        }
    }

    public Command runWith(DoubleSupplier rate) {
        return run(() -> leader.setControl(cachedVout.withOutput(rate.getAsDouble() * voltageMax)));
    }

    public Command runWithLimit(DoubleSupplier rate, double limit) {
        SlewRateLimiter accelLimiter = new SlewRateLimiter(limit);

        return run(() -> leader.setControl(cachedVout.withOutput(accelLimiter.calculate(rate.getAsDouble()) * voltageMax)));
    }

    public Command runAt(double rate) {
        return startEnd(() -> leader.setControl(cachedVout.withOutput(rate * voltageMax)), () -> {});
    }

    public Command brake() {
        return startEnd(() -> leader.setControl(cachedBrake), () -> {});
    }
}
