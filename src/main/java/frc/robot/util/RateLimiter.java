package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class RateLimiter {
  private double m_prevTime;

  public RateLimiter() {
    m_prevTime = MathSharedStore.getTimestamp();
  }

  public double calculate(double input, double output, double limit) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    output += MathUtil.clamp(
            input - output,
            -limit * elapsedTime,
            limit * elapsedTime);
    m_prevTime = currentTime;
    return output;
  }
}
