package frc.robot.subsystems.shooter.FeederIO;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public class FeederIOInputs {
    // Feeder
    public double feedAppliedVoltage = 0.0;
    public double feedStatorCurrent = 0.0;
    public double feedSupplyCurrent = 0.0;
    public double feedTemp = 0.0;
    public double feedVelocity = 0.0;
    public boolean isJammed = false;
  }

  public default void setFeedVoltage(double volts) {}

  public default void updateInputs(FeederIOInputs feederIOInputsAutoLogged) {}
}
