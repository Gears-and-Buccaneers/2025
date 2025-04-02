package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

public class LEDs {
    private final CANdle candle;

    public LEDs(int channel) {
        candle = new CANdle(channel, "drivetrain");

        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
    }

    public void setColor(int hex) {
        candle.setLEDs((hex >> 16) & 0xff, (hex >> 8) & 0xff, hex & 0xff);
    }
}
