package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.RobotMap;

public class HoodIOSim implements HoodIO {

  private static final Distance kServoLength = HoodConstants.Mechanical.kServoLength;
  private static final LinearVelocity kMaxServoSpeed = HoodConstants.Mechanical.kMaxServoSpeed;
  private static final double kMinPosition = HoodConstants.Mechanical.minPosition;
  private static final double kMaxPosition = HoodConstants.Mechanical.maxPosition;
  private static final double kPositionTolerance = 0.01;

  private double currentPosition = 0.5;
  private double targetPosition = 0.5;
  private Time lastUpdateTime;
  private boolean first = true;

  private final PWMSim leftSim;
  private final PWMSim rightSim;

  public HoodIOSim() {
    leftSim = new PWMSim(RobotMap.HoodMap.left);
    rightSim = new PWMSim(RobotMap.HoodMap.right);

    leftSim.setInitialized(true);
    rightSim.setInitialized(true);
  }

  private void updateCurrentPosition() {
    final Time now = Seconds.of(Timer.getFPGATimestamp());

    if (first) {
      lastUpdateTime = now;
      first = false;
      return;
    }

    final Time dt = now.minus(lastUpdateTime);
    lastUpdateTime = now;

    if (MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance)) {
      currentPosition = targetPosition;
      return;
    }

    final Distance maxDistance = kMaxServoSpeed.times(dt);
    final double maxPercent = maxDistance.div(kServoLength).in(Value);

    currentPosition =
        targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercent)
            : Math.max(targetPosition, currentPosition - maxPercent);
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
    updateCurrentPosition();

    leftSim.setPosition(currentPosition);
    rightSim.setPosition(currentPosition);

    inputs.leftPosition = currentPosition;
    inputs.rightPosition = currentPosition;
    inputs.targetPosition = targetPosition;
  }

  @Override
  public void setPosition(double position) {
    targetPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
  }
}
