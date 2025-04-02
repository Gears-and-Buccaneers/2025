package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class Toggle extends Command {
    private final Command cmdA;
    private final Command cmdB;
    private final BooleanSupplier toggle;
    private boolean useA = true;
    private boolean previous = false;

    public Toggle(Command cmdA, Command cmdB, BooleanSupplier toggle) {
        this.cmdA = cmdA;
        this.cmdB = cmdB;
        this.toggle = toggle;
    }

    @Override
    public void initialize() {
        if (useA)
            cmdA.schedule();
        else
            cmdB.schedule();
    }

    @Override
    public void execute() {
        var current = toggle.getAsBoolean();

        if (current && !previous) {
            end(false);
            useA = !useA;
            initialize();
        }

        previous = current;
    }

    @Override
    public void end(boolean interrupted) {
        if (useA)
            cmdA.cancel();
        else
            cmdB.cancel();
    }
}
