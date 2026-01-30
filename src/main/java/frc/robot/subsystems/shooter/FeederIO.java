package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public class FeederIOInputs {
        // Feeder
        public double feedAppliedVoltage = 0.0;
        public double feedStatorCurrent = 0.0;
        public double feedSupplyCurrent = 0.0;
        public double feedTemp = 0.0;
    }
    public default void setFeedVoltage(double volts) {}
    public default void updateInputs(FeederIOInputs inputs) {}

}
