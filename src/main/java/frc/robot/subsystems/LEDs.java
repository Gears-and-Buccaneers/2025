package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

public class LEDs {
    private final CANdle candle;

    public LEDs(int channel) {
        this.candle = new CANdle(channel, "drivetrain");
        this.candle.setLEDs(100, 100, 100);
    }
}
