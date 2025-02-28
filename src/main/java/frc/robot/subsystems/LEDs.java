package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;


public class LEDs {
    private final CANdle candle;
    Animation b = new RainbowAnimation(1, .3, -1, false, 8);

    public LEDs(int channel) {
        this.candle = new CANdle(channel, "drivetrain");

        candle.animate(b);
    }
}
