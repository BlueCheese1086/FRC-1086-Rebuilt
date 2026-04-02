package frc.robot.subsystems.shooter.FeederIO;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;

public class FeederIOTalonFX implements FeederIO {
  private TalonFX feeder;
  private TalonFXConfiguration feedConfig;

  private StatusSignal<Voltage> volts;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Temperature> tempreature;

  public FeederIOTalonFX(int feedID) {
    feeder = new TalonFX(feedID, RobotMap.systemBus);
    new VoltageOut(0.0).withEnableFOC(true);
    feedConfig = new TalonFXConfiguration();

    feedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feedConfig.CurrentLimits.SupplyCurrentLimit = 40.0; // arbitury

    // go)
    feedConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> feeder.getConfigurator().apply(feedConfig));

    volts = feeder.getMotorVoltage();
    statorCurrent = feeder.getStatorCurrent();
    supplyCurrent = feeder.getSupplyCurrent();
    tempreature = feeder.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, volts, statorCurrent, supplyCurrent, tempreature);

    feeder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(volts, statorCurrent, supplyCurrent, tempreature);

    inputs.feedAppliedVoltage = volts.getValueAsDouble();
    inputs.feedStatorCurrent = statorCurrent.getValueAsDouble();
    inputs.feedSupplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.feedTemp = tempreature.getValueAsDouble();
  }

  @Override
  public void setFeedVoltage(double volts) {
    // feeder.setControl(voltageRequest.withOutput(volts));
    feeder.setVoltage(volts);

    if (volts == 0) {
      feeder.stopMotor();
    }
  }
}
