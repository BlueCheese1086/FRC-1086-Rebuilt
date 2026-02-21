package frc.robot.subsystems.shooter.FeederIO;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOTalonFX implements FeederIO {
  private TalonFX feeder;
  private VoltageOut voltageRequest;
  private TalonFXConfiguration feedConfig;

  private StatusSignal<Voltage> volts;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Temperature> tempreature;

  public FeederIOTalonFX(int feedID) {
    feeder = new TalonFX(feedID);
    voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
    feedConfig = new TalonFXConfiguration();

    feedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feedConfig.CurrentLimits.SupplyCurrentLimit = 80.0; // arbitury
    feedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feedConfig.CurrentLimits.StatorCurrentLimit = 60.0; // arbitury

    feedConfig.Voltage.PeakForwardVoltage = 10.0;
    feedConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        200.0; // In amps (estimated 200 amps is max it will ever
    // go)

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
    feeder.setControl(voltageRequest.withOutput(volts));
  }
}
