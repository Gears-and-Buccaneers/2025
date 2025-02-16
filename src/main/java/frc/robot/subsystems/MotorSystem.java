package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class MotorSystem implements Subsystem {
    protected final TalonFX[] motors;
    protected final double voltageMax;
    protected final double deadband;

    protected final StatusSignal<Angle> positionSignal;

    protected final VoltageOut cachedVout = new VoltageOut(0);
    protected final StaticBrake cachedBrake = new StaticBrake();
    protected final CoastOut cachedCoast = new CoastOut();
    protected final TorqueCurrentFOC cachedTorque = new TorqueCurrentFOC(0.0);
    protected final MotionMagicTorqueCurrentFOC cachedPosition = new MotionMagicTorqueCurrentFOC(0.0);

    public MotorSystem(BiConsumer<Integer, TalonFXConfiguration> configure, double deadband, double maxOutPercent,
            int... ids) {
        if (ids.length == 0)
            DriverStation.reportError("Constructed MotorSystem without any motors", true);

        this.voltageMax = maxOutPercent * 12.0;
        this.deadband = deadband;

        motors = new TalonFX[ids.length];

        for (int i = 0; i < ids.length; i++) {
            motors[i] = new TalonFX(ids[i]);

            TalonFXConfiguration config = new TalonFXConfiguration();
            configure.accept(ids[i], config);

            motors[i].getConfigurator().apply(config);
        }

        for (int i = 1; i < motors.length; i++)
            motors[i].setControl(new StrictFollower(ids[0]));

            positionSignal = motors[0].getPosition();
    }

    public double position() {
        return positionSignal.refresh().getValueAsDouble();
    }

    public Command runWith(DoubleSupplier rate) {
        return run(() -> motors[0].setControl(cachedVout.withOutput(rate.getAsDouble() * voltageMax)));
    }

    public Command runAt(double rate) {
        return startEnd(() -> motors[0].setControl(cachedVout.withOutput(rate * voltageMax)), () -> {
        });
    }

    public Command brake() {
        return startEnd(() -> motors[0].setControl(cachedBrake), () -> {
        });
    }

    public Command coast() {
        return startEnd(() -> motors[0].setControl(cachedBrake), () -> {
        });
    }

    public Command goTo(double position) {
        return startEnd(() -> motors[0].setControl(cachedPosition.withPosition(position)), () -> {
        });
    }

    public Command atPoint(double position) {
        return new WaitUntilCommand(() -> Math.abs(positionSignal.refresh().getValueAsDouble() - position) < deadband);
    }

    public Command characterise() {
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(10.0).per(Second), Voltage.ofBaseUnits(9.0, Volts), null),
                new SysIdRoutine.Mechanism(v -> motors[0].setControl(cachedTorque.withOutput(v.in(Volts))), l -> {
                    for (var motor : motors)
                        l.motor(motor.getDescription())
                                .voltage(Voltage.ofBaseUnits(motor.getTorqueCurrent().getValueAsDouble(), Volts))
                                .angularPosition(positionSignal.refresh().getValue())
                                .angularVelocity(motor.getVelocity().getValue());
                }, this));

        return new SequentialCommandGroup(
                routine.dynamic(Direction.kForward),
                routine.dynamic(Direction.kReverse),
                routine.quasistatic(Direction.kForward),
                routine.quasistatic(Direction.kReverse));
    }
}
