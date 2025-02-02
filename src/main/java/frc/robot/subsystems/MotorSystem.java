package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class MotorSystem implements Subsystem {
    protected final TalonFX[] motors;
    protected final double voltageMax;

    protected final VoltageOut cachedVout = new VoltageOut(0);
    protected final StaticBrake cachedBrake = new StaticBrake();

    public MotorSystem(BiConsumer<Integer, TalonFXConfiguration> configure, double maxOutPercent, int... ids) {
        if (ids.length == 0)
            DriverStation.reportError("Constructed MotorSystem without any motors", true);

        this.voltageMax = maxOutPercent * 12.0;
        motors = new TalonFX[ids.length];

        for (int i = 0; i < ids.length; i++) {
            motors[i] = new TalonFX(ids[i]);

            TalonFXConfiguration config = new TalonFXConfiguration();
            configure.accept(ids[i], config);

            motors[i].getConfigurator().apply(config);
        }

        for (int i = 1; i < motors.length; i++)
            motors[i].setControl(new StrictFollower(ids[0]));
    }

    public Command runWith(DoubleSupplier rate) {
        return run(() -> motors[0].setControl(cachedVout.withOutput(rate.getAsDouble() * voltageMax)));
    }

    public Command runWithLimit(DoubleSupplier rate, double limit) {
        SlewRateLimiter accelLimiter = new SlewRateLimiter(limit);

        return run(() -> motors[0]
                .setControl(cachedVout.withOutput(accelLimiter.calculate(rate.getAsDouble()) * voltageMax)));
    }

    public Command runAt(double rate) {
        return startEnd(() -> motors[0].setControl(cachedVout.withOutput(rate * voltageMax)), () -> {
        });
    }

    public Command brake() {
        return startEnd(() -> motors[0].setControl(cachedBrake), () -> {
        });
    }

    public Command characterise(boolean linear) {
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(v -> motors[0].setControl(cachedVout.withOutput(v)), l -> {
                    for (var motor : motors)
                        l.motor(motor.getDescription()).voltage(motor.getMotorVoltage().getValue())
                                .angularPosition(motor.getPosition().getValue())
                                .angularVelocity(motor.getVelocity().getValue());
                }, this));

        return new SequentialCommandGroup(
                routine.dynamic(Direction.kForward),
                routine.dynamic(Direction.kReverse),
                routine.quasistatic(Direction.kForward),
                routine.quasistatic(Direction.kReverse));
    }
}
