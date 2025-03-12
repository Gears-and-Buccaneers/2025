package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MotorSystem implements Subsystem {
    protected final TalonFX[] motors;
    protected final double voltageMax;
    public final double deadband;

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

        StrictFollower follow = new StrictFollower(ids[0]);

        for (int i = 1; i < motors.length; i++)
            motors[i].setControl(follow);

        positionSignal = motors[0].getPosition();

        setDefaultCommand(brake());
    }

    /* Methods for setting motor controls and reading sensors */
    public double position() {
        return positionSignal.refresh().getValueAsDouble();
    }

    public void setPosition(double position) {
        motors[0].setPosition(position);
    }

    public void setRate(double rate) {
        motors[0].setControl(cachedVout.withOutput(rate * voltageMax));
    }

    public void setTarget(double position) {
        motors[0].setControl(cachedPosition.withPosition(position));
    }

    /* Commands for those behaviours */
    public Command runWith(DoubleSupplier rate) {
        return run(() -> setRate(rate.getAsDouble()));
    }

    public Command runAt(double rate) {
        return startEnd(() -> setRate(rate), () -> {
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
        return startEnd(() -> setTarget(position), () -> {
        });
    }

    public Command goTo(DoubleSupplier position) {
        return run(() -> setTarget(position.getAsDouble()));
    }

    public Command atPoint(double position) {
        return new WaitUntilCommand(() -> Math.abs(position() - position) < deadband);
    }

    public Command goToStop(double position) {
        return goTo(position).raceWith(atPoint(position));
    }
}
