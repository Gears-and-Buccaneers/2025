package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

public class LEDs {
    private final CANdle candle;

    public LEDs(int channel) {
        candle = new CANdle(channel, "drivetrain");

        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
    }
}
