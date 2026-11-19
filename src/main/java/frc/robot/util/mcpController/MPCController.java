package frc.robot.util.mcpController;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Acceleration MPC with separate measured and commanded velocity states.
 *
 * <p>
 * The optimizer slews a persistent FIELD-relative velocity command. Its rollout
 * models measured
 * velocity approaching that command using VelocityResponse. Measurements
 * initialize each predicted
 * robot state, but do not overwrite the saved command on every cycle. This
 * avoids feeding normal
 * module overspeed back into a higher velocity command.
 *
 * <p>
 * Velocity tracking costs have a configurable deadband; position and heading
 * costs do not.
 * Acceleration limits constrain COMMAND slew, not guaranteed physical tire
 * acceleration. Predicted
 * speed limits are soft; the returned command is hard saturated.
 * Drive.runVelocity already
 * discretizes/desaturates, so the returned robot-relative speeds are
 * continuous.
 *
 * <p>
 * Call once per configured period on one thread and reset at each independent
 * drive section. The
 * follower uses measured elapsed time for progress/timeout bookkeeping. A
 * best-effort solve budget
 * limits optimizer work, but is not a hard real-time guarantee on the roboRIO
 * JVM.
 */
public final class MPCController {
  /**
   * Fraction of command-minus-measured velocity error corrected per configured
   * period.
   */
  public record VelocityResponse(double parallel, double perpendicular, double angular) {
    public VelocityResponse {
      for (double value : new double[] { parallel, perpendicular, angular }) {
        if (!Double.isFinite(value) || value <= 0 || value > 1) {
          throw new IllegalArgumentException("Velocity response fractions must be in (0, 1]");
        }
      }
    }

    public static VelocityResponse ideal() {
      return new VelocityResponse(1.0, 1.0, 1.0);
    }
  }

  /** Soft velocity-error allowance. Pose errors are always penalized. */
  public record TrackingTolerance(double linearMps, double angularRadPerSec) {
    public TrackingTolerance {
      if (!Double.isFinite(linearMps)
          || linearMps < 0
          || !Double.isFinite(angularRadPerSec)
          || angularRadPerSec < 0) {
        throw new IllegalArgumentException("Tracking tolerances must be finite and nonnegative");
      }
    }

    public static TrackingTolerance defaults() {
      return new TrackingTolerance(0.15, 0.25);
    }
  }

  private static final long SOLVE_BUDGET_NANOS = 6_000_000L;

  public record Acceleration(double fieldAxMps2, double fieldAyMps2, double alphaRadPerSec2) {
  }

  public record Diagnostics(
      int iterations,
      boolean converged,
      double cost,
      double solveTimeMillis,
      boolean outputLimited,
      double maxPredictedSpeedViolationMps) {
  }

  private final MPCConfig config;
  private final VelocityResponse velocityResponse;
  private final TrackingTolerance trackingTolerance;
  // The last command actually returned, in field coordinates; independent of
  // encoder noise.
  private final double[] commandVelocity = new double[3];
  private boolean commandInitialized;
  // Fixed response matrix during one solve, aligned with the measured direction
  // of travel.
  private double responseXX, responseXY, responseYY, responseAngular;
  private final int n;
  private double dt;
  private final double[][] inputs;
  private final double[][] candidate;
  private final double[][] gradient;
  private final double[][] states;
  private final double[][] references;
  private final double[] previousAcceleration = new double[3];
  private boolean warm;
  private double stepSize = 0.1;
  private Diagnostics diagnostics = new Diagnostics(0, false, 0, 0, false, 0);

  public MPCController(MPCConfig config) {
    this(config, VelocityResponse.ideal());
  }

  public MPCController(MPCConfig config, VelocityResponse velocityResponse) {
    this(config, velocityResponse, TrackingTolerance.defaults());
  }

  public MPCController(
      MPCConfig config, VelocityResponse velocityResponse, TrackingTolerance trackingTolerance) {
    this.trackingTolerance = Objects.requireNonNull(trackingTolerance, "trackingTolerance");
    this.velocityResponse = Objects.requireNonNull(velocityResponse, "velocityResponse");
    this.config = Objects.requireNonNull(config, "config");
    n = config.horizonSteps();
    dt = config.periodSeconds();
    inputs = new double[n][3];
    candidate = new double[n][3];
    gradient = new double[n][3];
    states = new double[n + 1][9];
    references = new double[n][6];
  }

  public MPCConfig getConfig() {
    return config;
  }

  /**
   * Clear warm start after interruption, odometry reset, or a new independent
   * driving section.
   */
  public void reset() {
    for (double[] row : inputs)
      Arrays.fill(row, 0.0);
    for (double[] row : states)
      Arrays.fill(row, 0.0);
    Arrays.fill(previousAcceleration, 0.0);
    Arrays.fill(commandVelocity, 0.0);
    commandInitialized = false;
    warm = false;
    stepSize = 0.1;
    diagnostics = new Diagnostics(0, false, 0, 0, false, 0);
  }

  /**
   * Returns ROBOT-relative continuous ChassisSpeeds. Supply exactly horizonSteps
   * references: index
   * 0 is the target at now+dt, index 1 at now+2*dt, ..., NOT a sample at now.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      ChassisSpeeds measuredRobotRelativeSpeeds,
      List<MPCReference> futureReferences) {
    return calculate(
        MPCState.fromRobotRelative(currentPose, measuredRobotRelativeSpeeds), futureReferences);
  }

  public ChassisSpeeds calculate(MPCState measured, List<MPCReference> futureReferences) {
    return calculate(measured, futureReferences, config.periodSeconds());
  }

  /**
   * References must be spaced by controlPeriodSeconds; supports a slower measured
   * loop.
   */
  public ChassisSpeeds calculate(
      MPCState measured, List<MPCReference> futureReferences, double controlPeriodSeconds) {
    if (!Double.isFinite(controlPeriodSeconds) || controlPeriodSeconds <= 0) {
      throw new IllegalArgumentException("controlPeriodSeconds must be finite and positive");
    }
    dt = controlPeriodSeconds;
    Objects.requireNonNull(measured, "measured");
    Objects.requireNonNull(futureReferences, "futureReferences");
    if (futureReferences.size() != n) {
      throw new IllegalArgumentException("Expected exactly " + n + " future samples");
    }
    long start = System.nanoTime();
    loadReferences(measured.headingRadians(), futureReferences);
    if (!commandInitialized) {
      commandVelocity[0] = measured.fieldVxMps();
      commandVelocity[1] = measured.fieldVyMps();
      commandVelocity[2] = measured.omegaRadPerSec();
      commandInitialized = true;
    }
    double speed = Math.hypot(measured.fieldVxMps(), measured.fieldVyMps());
    double ex = speed > 0.1 ? measured.fieldVxMps() / speed : 1.0;
    double ey = speed > 0.1 ? measured.fieldVyMps() / speed : 0.0;
    double periodScale = dt / config.periodSeconds();
    double parallel = 1.0 - Math.pow(1.0 - velocityResponse.parallel(), periodScale);
    double perpendicular = speed > 0.1
        ? 1.0 - Math.pow(1.0 - velocityResponse.perpendicular(), periodScale)
        : parallel;
    responseAngular = 1.0 - Math.pow(1.0 - velocityResponse.angular(), periodScale);
    responseXX = perpendicular + (parallel - perpendicular) * ex * ex;
    responseXY = (parallel - perpendicular) * ex * ey;
    responseYY = perpendicular + (parallel - perpendicular) * ey * ey;
    if (warm) {
      for (int i = 0; i < n - 1; i++)
        System.arraycopy(inputs[i + 1], 0, inputs[i], 0, 3);
      // Hold the terminal acceleration as a warm start only; the optimizer may change
      // it.
    }

    int iterations = 0;
    boolean converged = false;
    double cost = evaluate(measured, inputs, true);
    if (!Double.isFinite(cost))
      throw new IllegalArgumentException("State/reference magnitude overflow");

    for (int iteration = 0; iteration < config.maxIterations(); iteration++) {
      if (iterations > 0 && System.nanoTime() - start >= SOLVE_BUDGET_NANOS)
        break;
      iterations++;
      // evaluate(...,true) has refreshed the gradient at inputs.
      double trialStep = Math.min(stepSize * 1.2, 1.0);
      boolean accepted = false;
      double candidateCost = cost;
      double movement = 0.0;
      double maxMovement = 0.0;
      for (int search = 0; search < 24; search++) {
        double directionalDerivative = 0.0;
        movement = 0.0;
        maxMovement = 0.0;
        for (int k = 0; k < n; k++) {
          for (int j = 0; j < 3; j++)
            candidate[k][j] = inputs[k][j] - trialStep * gradient[k][j];
          projectAcceleration(candidate[k]);
          for (int j = 0; j < 3; j++) {
            double delta = candidate[k][j] - inputs[k][j];
            directionalDerivative += gradient[k][j] * delta;
            movement += delta * delta;
            maxMovement = Math.max(maxMovement, Math.abs(delta));
          }
        }
        candidateCost = evaluate(measured, candidate, false);
        // Quadratic upper-bound test gives a descent step for projected gradient.
        if (Double.isFinite(candidateCost)
            && candidateCost <= cost
                + directionalDerivative
                + movement / (2.0 * trialStep)
                + 1e-10 * Math.max(1.0, Math.abs(cost))) {
          accepted = true;
          break;
        }
        trialStep *= 0.5;
      }
      if (!accepted)
        break; // Keep last accepted feasible inputs; not reported as converged.
      for (int k = 0; k < n; k++)
        System.arraycopy(candidate[k], 0, inputs[k], 0, 3);
      stepSize = trialStep;
      cost = candidateCost;
      // Projected gradient mapping, rather than a tiny step alone, tests convergence.
      if (maxMovement / trialStep <= config.convergenceTolerance()) {
        converged = true;
        break;
      }
      cost = evaluate(measured, inputs, true);
    }

    cost = evaluate(measured, inputs, false); // Restore states for the accepted plan.
    // Integrate the optimized COMMAND, not the latest measured wheel speed.
    double vx = commandVelocity[0] + dt * inputs[0][0];
    double vy = commandVelocity[1] + dt * inputs[0][1];
    double omega = commandVelocity[2] + dt * inputs[0][2];
    MPCConfig.Limits limits = config.limits();
    double linearSpeed = Math.hypot(vx, vy);
    double moduleBudget = linearSpeed + limits.driveBaseRadiusMeters() * Math.abs(omega);
    double scale = 1.0;
    if (linearSpeed > limits.maxLinearSpeedMps())
      scale = Math.min(scale, limits.maxLinearSpeedMps() / linearSpeed);
    if (Math.abs(omega) > limits.maxAngularSpeedRadPerSec())
      scale = Math.min(scale, limits.maxAngularSpeedRadPerSec() / Math.abs(omega));
    if (moduleBudget > limits.maxModuleSpeedMps())
      scale = Math.min(scale, limits.maxModuleSpeedMps() / moduleBudget);
    boolean outputLimited = scale < 1.0;
    vx *= scale;
    vy *= scale;
    omega *= scale;
    // Saturation is fed back into command memory, preventing command integrator
    // windup.
    previousAcceleration[0] = (vx - commandVelocity[0]) / dt;
    previousAcceleration[1] = (vy - commandVelocity[1]) / dt;
    previousAcceleration[2] = (omega - commandVelocity[2]) / dt;
    projectAcceleration(previousAcceleration);
    commandVelocity[0] = vx;
    commandVelocity[1] = vy;
    commandVelocity[2] = omega;
    warm = true;
    diagnostics = new Diagnostics(
        iterations,
        converged,
        cost,
        (System.nanoTime() - start) / 1e6,
        outputLimited,
        maximumSpeedViolation());
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        vx, vy, omega, Rotation2d.fromRadians(measured.headingRadians()));
  }

  public Diagnostics getDiagnostics() {
    return diagnostics;
  }

  /**
   * Planned states before output saturation. Allocates; use on demand for
   * logging.
   */
  public Pose2d[] getPredictedPoses() {
    Pose2d[] result = new Pose2d[n + 1];
    for (int k = 0; k <= n; k++) {
      result[k] = new Pose2d(states[k][0], states[k][1], Rotation2d.fromRadians(states[k][2]));
    }
    return result;
  }

  public Acceleration[] getPlannedAccelerations() {
    Acceleration[] result = new Acceleration[n];
    for (int k = 0; k < n; k++)
      result[k] = new Acceleration(inputs[k][0], inputs[k][1], inputs[k][2]);
    return result;
  }

  public TrackingTolerance getTrackingTolerance() {
    return trackingTolerance;
  }

  /** Last continuous command in field coordinates; returns a fresh object. */
  public ChassisSpeeds getCommandedFieldSpeeds() {
    return new ChassisSpeeds(commandVelocity[0], commandVelocity[1], commandVelocity[2]);
  }

  private void loadReferences(double currentHeading, List<MPCReference> samples) {
    double previousWrapped = wrap(currentHeading);
    double continuous = currentHeading;
    for (int k = 0; k < n; k++) {
      MPCReference sample = Objects.requireNonNull(samples.get(k), "reference " + k);
      double heading = sample.pose().getRotation().getRadians();
      continuous += wrap(heading - previousWrapped);
      previousWrapped = heading;
      references[k][0] = sample.pose().getX();
      references[k][1] = sample.pose().getY();
      references[k][2] = continuous;
      references[k][3] = sample.fieldVxMps();
      references[k][4] = sample.fieldVyMps();
      references[k][5] = sample.omegaRadPerSec();
    }
  }

  private void projectAcceleration(double[] acceleration) {
    MPCConfig.Limits l = config.limits();
    double norm = Math.hypot(acceleration[0], acceleration[1]);
    if (norm > l.maxLinearAccelerationMps2()) {
      double scale = l.maxLinearAccelerationMps2() / norm;
      acceleration[0] *= scale;
      acceleration[1] *= scale;
    }
    acceleration[2] = Math.max(
        -l.maxAngularAccelerationRadPerSec2(),
        Math.min(l.maxAngularAccelerationRadPerSec2(), acceleration[2]));
  }

  /**
   * Roll out [pose, measured velocity, command velocity] and differentiate by
   * adjoints.
   */
  private double evaluate(MPCState measured, double[][] accelerations, boolean differentiate) {
    MPCConfig.Weights w = config.weights();
    states[0][0] = measured.xMeters();
    states[0][1] = measured.yMeters();
    states[0][2] = measured.headingRadians();
    states[0][3] = measured.fieldVxMps();
    states[0][4] = measured.fieldVyMps();
    states[0][5] = measured.omegaRadPerSec();
    System.arraycopy(commandVelocity, 0, states[0], 6, 3);
    double cost = 0.0;
    for (int k = 0; k < n; k++) {
      double[] before = states[k], after = states[k + 1];
      for (int j = 0; j < 3; j++)
        after[j + 6] = before[j + 6] + dt * accelerations[k][j];
      double dx = after[6] - before[3], dy = after[7] - before[4];
      after[3] = before[3] + responseXX * dx + responseXY * dy;
      after[4] = before[4] + responseXY * dx + responseYY * dy;
      after[5] = before[5] + responseAngular * (after[8] - before[5]);
      for (int j = 0; j < 3; j++) {
        after[j] = before[j] + 0.5 * dt * (before[j + 3] + after[j + 3]);
      }
      double terminal = k == n - 1 ? w.terminalMultiplier() : 1.0;
      double positionErrorX = after[0] - references[k][0];
      double positionErrorY = after[1] - references[k][1];
      double headingError = after[2] - references[k][2];
      double vxError = after[3] - references[k][3];
      double vyError = after[4] - references[k][4];
      double speedError = Math.hypot(vxError, vyError);
      double excess = Math.max(0, speedError - trackingTolerance.linearMps());
      double omegaError = deadband(after[5] - references[k][5], trackingTolerance.angularRadPerSec());
      cost += terminal
          * (w.position() * (positionErrorX * positionErrorX + positionErrorY * positionErrorY)
              + w.heading() * headingError * headingError
              + w.velocity() * excess * excess
              + w.angularVelocity() * omegaError * omegaError);
      // Apply soft speed bounds to both physical predictions and commanded
      // velocities.
      cost += speedPenalty(after, 3, null) + speedPenalty(after, 6, null);
      for (int j = 0; j < 3; j++) {
        double a = accelerations[k][j];
        double previous = k == 0 ? previousAcceleration[j] : accelerations[k - 1][j];
        double change = a - previous;
        cost += accelerationWeight(j) * a * a + changeWeight(j) * change * change;
      }
    }
    if (!differentiate)
      return cost;

    double lpX = 0, lpY = 0, lpTheta = 0;
    double lvX = 0, lvY = 0, lvOmega = 0;
    double lcX = 0, lcY = 0, lcOmega = 0;
    for (int k = n - 1; k >= 0; k--) {
      double[] state = states[k + 1], ref = references[k];
      double terminal = k == n - 1 ? w.terminalMultiplier() : 1.0;
      lpX += 2 * terminal * w.position() * (state[0] - ref[0]);
      lpY += 2 * terminal * w.position() * (state[1] - ref[1]);
      lpTheta += 2 * terminal * w.heading() * (state[2] - ref[2]);
      double dx = state[3] - ref[3], dy = state[4] - ref[4];
      double speedError = Math.hypot(dx, dy);
      if (speedError > trackingTolerance.linearMps()) {
        double g = 2 * terminal * w.velocity() * (speedError - trackingTolerance.linearMps()) / speedError;
        lvX += g * dx;
        lvY += g * dy;
      }
      lvOmega += 2
          * terminal
          * w.angularVelocity()
          * deadband(state[5] - ref[5], trackingTolerance.angularRadPerSec());
      // Reuse preallocated scratch vectors for speed-penalty derivatives.
      speedPenalty(state, 3, speedGradient);
      lvX += speedGradient[0];
      lvY += speedGradient[1];
      lvOmega += speedGradient[2];
      speedPenalty(state, 6, speedGradient);
      lcX += speedGradient[0];
      lcY += speedGradient[1];
      lcOmega += speedGradient[2];

      double qx = lvX + 0.5 * dt * lpX;
      double qy = lvY + 0.5 * dt * lpY;
      double qo = lvOmega + 0.5 * dt * lpTheta;
      double bx = responseXX * qx + responseXY * qy;
      double by = responseXY * qx + responseYY * qy;
      double bo = responseAngular * qo;
      lcX += bx;
      lcY += by;
      lcOmega += bo;
      gradient[k][0] = dt * lcX;
      gradient[k][1] = dt * lcY;
      gradient[k][2] = dt * lcOmega;
      for (int j = 0; j < 3; j++) {
        double a = accelerations[k][j];
        double previous = k == 0 ? previousAcceleration[j] : accelerations[k - 1][j];
        double smooth = a - previous;
        if (k + 1 < n)
          smooth += a - accelerations[k + 1][j];
        gradient[k][j] += 2 * accelerationWeight(j) * a + 2 * changeWeight(j) * smooth;
      }
      lvX = 0.5 * dt * lpX + qx - bx;
      lvY = 0.5 * dt * lpY + qy - by;
      lvOmega = 0.5 * dt * lpTheta + qo - bo;
    }
    return cost;
  }

  private final double[] speedGradient = new double[3];

  private static double deadband(double value, double tolerance) {
    return Math.copySign(Math.max(0.0, Math.abs(value) - tolerance), value);
  }

  private double speedPenalty(double[] state, int offset, double[] derivative) {
    var l = config.limits();
    double vx = state[offset], vy = state[offset + 1], omega = state[offset + 2];
    double linear = Math.hypot(vx, vy), radius = l.driveBaseRadiusMeters();
    double moduleOver = Math.max(0, linear + radius * Math.abs(omega) - l.maxModuleSpeedMps());
    double linearOver = Math.max(0, linear - l.maxLinearSpeedMps());
    double angularOver = Math.max(0, Math.abs(omega) - l.maxAngularSpeedRadPerSec());
    double weight = config.weights().speedViolation();
    if (derivative != null) {
      double g = linear > 1e-12 ? 2 * weight * (moduleOver + linearOver) / linear : 0;
      derivative[0] = g * vx;
      derivative[1] = g * vy;
      derivative[2] = 2 * weight * Math.signum(omega) * (radius * moduleOver + radius * radius * angularOver);
    }
    return weight
        * (moduleOver * moduleOver
            + linearOver * linearOver
            + radius * radius * angularOver * angularOver);
  }

  private double accelerationWeight(int j) {
    return j == 2 ? config.weights().angularAcceleration() : config.weights().linearAcceleration();
  }

  private double changeWeight(int j) {
    return j == 2
        ? config.weights().angularAccelerationChange()
        : config.weights().linearAccelerationChange();
  }

  private double maximumSpeedViolation() {
    double max = 0.0;
    MPCConfig.Limits l = config.limits();
    for (int k = 1; k <= n; k++) {
      for (int offset = 3; offset <= 6; offset += 3) {
        double linear = Math.hypot(states[k][offset], states[k][offset + 1]);
        double omega = Math.abs(states[k][offset + 2]);
        max = Math.max(max, linear - l.maxLinearSpeedMps());
        max = Math.max(max, l.driveBaseRadiusMeters() * (omega - l.maxAngularSpeedRadPerSec()));
        max = Math.max(max, linear + l.driveBaseRadiusMeters() * omega - l.maxModuleSpeedMps());
      }
    }
    return max;
  }

  private static double wrap(double radians) {
    return Math.atan2(Math.sin(radians), Math.cos(radians));
  }

  /**
   * SI units throughout. Defaults are starting values, not identified robot
   * parameters.
   */
  public record MPCConfig(
      double periodSeconds,
      int horizonSteps,
      int maxIterations,
      double convergenceTolerance,
      Weights weights,
      Limits limits) {

    public MPCConfig {
      positive(periodSeconds, "periodSeconds");
      if (horizonSteps < 2 || horizonSteps > 250) {
        throw new IllegalArgumentException("horizonSteps must be between 2 and 250");
      }
      if (maxIterations < 1 || maxIterations > 1000) {
        throw new IllegalArgumentException("maxIterations must be between 1 and 1000");
      }
      positive(convergenceTolerance, "convergenceTolerance");
      Objects.requireNonNull(weights, "weights");
      Objects.requireNonNull(limits, "limits");
    }

    /**
     * A 1 second nominal horizon at 20 ms. Tune command slew limits for the
     * drivetrain.
     * maxModuleSpeedMps is the wheel speed limit, not a trajectory's preferred
     * cruising speed.
     */
    public static MPCConfig startingPoint(double maxModuleSpeedMps, double driveBaseRadiusMeters) {
      positive(maxModuleSpeedMps, "maxModuleSpeedMps");
      positive(driveBaseRadiusMeters, "driveBaseRadiusMeters");
      return new MPCConfig(
          0.02,
          50,
          30,
          1e-3,
          new Weights(40.0, 4.0, 25.0, 2.0, 0.08, 0.04, 0.08, 0.04, 3.0, 100.0),
          new Limits(
              maxModuleSpeedMps,
              maxModuleSpeedMps / driveBaseRadiusMeters,
              5.0,
              600.0,
              maxModuleSpeedMps,
              driveBaseRadiusMeters));
    }

    /** Raw squared-error weights; meters and radians have separate weights. */
    public record Weights(
        double position,
        double velocity,
        double heading,
        double angularVelocity,
        double linearAcceleration,
        double angularAcceleration,
        double linearAccelerationChange,
        double angularAccelerationChange,
        double terminalMultiplier,
        double speedViolation) {
      public Weights {
        positive(position, "position");
        positive(velocity, "velocity");
        positive(heading, "heading");
        positive(angularVelocity, "angularVelocity");
        positive(linearAcceleration, "linearAcceleration");
        positive(angularAcceleration, "angularAcceleration");
        nonnegative(linearAccelerationChange, "linearAccelerationChange");
        nonnegative(angularAccelerationChange, "angularAccelerationChange");
        positive(terminalMultiplier, "terminalMultiplier");
        positive(speedViolation, "speedViolation");
      }
    }

    /**
     * Command acceleration bounds are hard optimizer constraints. Predicted speed
     * bounds are SOFT
     * costs; the returned command is hard limited. See MPCController for the
     * conservative module
     * budget.
     */
    public record Limits(
        double maxLinearSpeedMps,
        double maxAngularSpeedRadPerSec,
        double maxLinearAccelerationMps2,
        double maxAngularAccelerationRadPerSec2,
        double maxModuleSpeedMps,
        double driveBaseRadiusMeters) {
      public Limits {
        positive(maxLinearSpeedMps, "maxLinearSpeedMps");
        positive(maxAngularSpeedRadPerSec, "maxAngularSpeedRadPerSec");
        positive(maxLinearAccelerationMps2, "maxLinearAccelerationMps2");
        positive(maxAngularAccelerationRadPerSec2, "maxAngularAccelerationRadPerSec2");
        positive(maxModuleSpeedMps, "maxModuleSpeedMps");
        positive(driveBaseRadiusMeters, "driveBaseRadiusMeters");
      }
    }

    static void finite(double value, String name) {
      if (!Double.isFinite(value))
        throw new IllegalArgumentException(name + " must be finite");
    }

    private static void positive(double value, String name) {
      finite(value, name);
      if (value <= 0)
        throw new IllegalArgumentException(name + " must be positive");
    }

    private static void nonnegative(double value, String name) {
      finite(value, name);
      if (value < 0)
        throw new IllegalArgumentException(name + " must be nonnegative");
    }
  }

  /**
   * Measured chassis state. Velocities are FIELD relative; heading is in radians.
   */
  public record MPCState(
      double xMeters,
      double yMeters,
      double headingRadians,
      double fieldVxMps,
      double fieldVyMps,
      double omegaRadPerSec) {
    public MPCState {
      MPCConfig.finite(xMeters, "xMeters");
      MPCConfig.finite(yMeters, "yMeters");
      MPCConfig.finite(headingRadians, "headingRadians");
      MPCConfig.finite(fieldVxMps, "fieldVxMps");
      MPCConfig.finite(fieldVyMps, "fieldVyMps");
      MPCConfig.finite(omegaRadPerSec, "omegaRadPerSec");
    }

    public static MPCState fromRobotRelative(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
      Objects.requireNonNull(pose, "pose");
      Objects.requireNonNull(robotRelativeSpeeds, "robotRelativeSpeeds");
      double c = pose.getRotation().getCos();
      double s = pose.getRotation().getSin();
      return new MPCState(
          pose.getX(),
          pose.getY(),
          pose.getRotation().getRadians(),
          c * robotRelativeSpeeds.vxMetersPerSecond - s * robotRelativeSpeeds.vyMetersPerSecond,
          s * robotRelativeSpeeds.vxMetersPerSecond + c * robotRelativeSpeeds.vyMetersPerSecond,
          robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    public Pose2d pose() {
      return new Pose2d(xMeters, yMeters, Rotation2d.fromRadians(headingRadians));
    }
  }

  /**
   * One future reference. Pose uses the field frame and robot heading (not travel
   * heading).
   * Velocity is FIELD relative in m/s; omega is rad/s. Immutable primitive
   * velocity values avoid
   * retaining a mutable ChassisSpeeds supplied by another subsystem.
   */
  public record MPCReference(
      Pose2d pose, double fieldVxMps, double fieldVyMps, double omegaRadPerSec) {
    public MPCReference {
      Objects.requireNonNull(pose, "pose");
      MPCConfig.finite(pose.getX(), "pose.x");
      MPCConfig.finite(pose.getY(), "pose.y");
      MPCConfig.finite(pose.getRotation().getRadians(), "pose.heading");
      MPCConfig.finite(fieldVxMps, "fieldVxMps");
      MPCConfig.finite(fieldVyMps, "fieldVyMps");
      MPCConfig.finite(omegaRadPerSec, "omegaRadPerSec");
    }

    public static MPCReference stationary(Pose2d pose) {
      return new MPCReference(pose, 0.0, 0.0, 0.0);
    }
  }

  /**
   * Time-based preview over an already generated, already alliance-transformed
   * driving section. Can
   * span multiple trajectories, but does not smooth their geometry, regenerate
   * them, execute named
   * commands/events, or construct an auto. End the section at a planned stop,
   * e.g. Paths1-3.
   */
  public static final class PathPlannerMPCReference {
    private final List<PathPlannerTrajectory> trajectories;
    private final double[] endTimes;
    private final MPCReference stoppedEnd;

    public PathPlannerMPCReference(List<PathPlannerTrajectory> trajectories) {
      this.trajectories = List.copyOf(trajectories);
      if (this.trajectories.isEmpty())
        throw new IllegalArgumentException("No trajectories");
      endTimes = new double[this.trajectories.size()];
      double time = 0;
      for (int i = 0; i < endTimes.length; i++) {
        PathPlannerTrajectory trajectory = Objects.requireNonNull(this.trajectories.get(i));
        double duration = trajectory.getTotalTimeSeconds();
        if (!Double.isFinite(duration) || duration <= 0) {
          throw new IllegalArgumentException(
              "Trajectory " + i + " must have positive finite duration");
        }
        if (Math.abs(trajectory.getInitialState().timeSeconds) > 1e-6) {
          throw new IllegalArgumentException("Each trajectory must start at time zero");
        }
        double previousTime = -1.0;
        for (PathPlannerTrajectoryState state : trajectory.getStates()) {
          if (!Double.isFinite(state.timeSeconds) || state.timeSeconds <= previousTime) {
            throw new IllegalArgumentException(
                "Trajectory states must have increasing finite times");
          }
          fromState(state); // Validate pose and speeds before control starts.
          previousTime = state.timeSeconds;
        }
        if (i > 0) {
          var previous = this.trajectories.get(i - 1).getEndState();
          var next = trajectory.getInitialState();
          double gap = Math.hypot(
              previous.pose.getX() - next.pose.getX(), previous.pose.getY() - next.pose.getY());
          double headingGap = Math.abs(previous.pose.getRotation().minus(next.pose.getRotation()).getRadians());
          if (gap > 0.05 || headingGap > Math.toRadians(5)) {
            throw new IllegalArgumentException(
                "Trajectory join " + i + " has a pose discontinuity");
          }
        }
        time += duration;
        if (!Double.isFinite(time))
          throw new IllegalArgumentException("Total duration overflow");
        endTimes[i] = time;
      }
      PathPlannerTrajectoryState end = this.trajectories.get(endTimes.length - 1).getEndState();
      if (Math.hypot(end.fieldSpeeds.vxMetersPerSecond, end.fieldSpeeds.vyMetersPerSecond) > 0.1
          || Math.abs(end.fieldSpeeds.omegaRadiansPerSecond) > 0.1) {
        throw new IllegalArgumentException(
            "Preview section must end at a stop; include the next path");
      }
      stoppedEnd = MPCReference.stationary(end.pose);
    }

    public double getTotalTimeSeconds() {
      return endTimes[endTimes.length - 1];
    }

    /**
     * Samples at elapsed+dt ... elapsed+N*dt. Beyond the section, hold its final
     * stopped pose.
     */
    public List<MPCReference> sampleHorizon(double elapsedSeconds, MPCConfig config) {
      MPCConfig.finite(elapsedSeconds, "elapsedSeconds");
      Objects.requireNonNull(config, "config");
      List<MPCReference> result = new ArrayList<>(config.horizonSteps());
      for (int k = 1; k <= config.horizonSteps(); k++) {
        result.add(sample(elapsedSeconds + k * config.periodSeconds()));
      }
      return result;
    }

    public MPCReference sample(double seconds) {
      MPCConfig.finite(seconds, "seconds");
      if (seconds >= getTotalTimeSeconds())
        return stoppedEnd;
      seconds = Math.max(0, seconds);
      int index = 0;
      while (index + 1 < endTimes.length && seconds >= endTimes[index])
        index++;
      double localTime = seconds - (index == 0 ? 0 : endTimes[index - 1]);
      // Interpolate the dense generated states directly. This avoids relying on the
      // version-specific numerical integration inside
      // PathPlannerTrajectoryState.interpolate().
      List<PathPlannerTrajectoryState> states = trajectories.get(index).getStates();
      int low = 0;
      int high = states.size() - 1;
      if (localTime <= states.get(low).timeSeconds)
        return fromState(states.get(low));
      if (localTime >= states.get(high).timeSeconds)
        return fromState(states.get(high));
      while (low + 1 < high) {
        int mid = (low + high) >>> 1;
        if (states.get(mid).timeSeconds <= localTime)
          low = mid;
        else
          high = mid;
      }
      PathPlannerTrajectoryState a = states.get(low), b = states.get(high);
      double fraction = (localTime - a.timeSeconds) / (b.timeSeconds - a.timeSeconds);
      return new MPCReference(
          new Pose2d(
              lerp(a.pose.getX(), b.pose.getX(), fraction),
              lerp(a.pose.getY(), b.pose.getY(), fraction),
              a.pose.getRotation().interpolate(b.pose.getRotation(), fraction)),
          lerp(a.fieldSpeeds.vxMetersPerSecond, b.fieldSpeeds.vxMetersPerSecond, fraction),
          lerp(a.fieldSpeeds.vyMetersPerSecond, b.fieldSpeeds.vyMetersPerSecond, fraction),
          lerp(a.fieldSpeeds.omegaRadiansPerSecond, b.fieldSpeeds.omegaRadiansPerSecond, fraction));
    }

    private static MPCReference fromState(PathPlannerTrajectoryState state) {
      return new MPCReference(
          state.pose,
          state.fieldSpeeds.vxMetersPerSecond,
          state.fieldSpeeds.vyMetersPerSecond,
          state.fieldSpeeds.omegaRadiansPerSecond);
    }

    private static double lerp(double a, double b, double fraction) {
      return a + fraction * (b - a);
    }
  }

  /**
   * Error-governed trajectory clock with barriers at planned stops. This class
   * owns no motors,
   * commands, or wall clock. Call update exactly once per MPC period using
   * current measurements. It
   * preserves input path geometry and saved moving joins; it does not smooth
   * sharp joins.
   */
  public static final class MPCPathProgress {
    public record Settings(
        double slowErrorMeters,
        double pauseErrorMeters,
        double slowHeadingRadians,
        double pauseHeadingRadians,
        double positionToleranceMeters,
        double headingToleranceRadians,
        double speedToleranceMps,
        double omegaToleranceRadPerSec,
        double settleSeconds,
        double rateIncreasePerSecond,
        double timeoutMultiplier,
        double timeoutExtraSeconds) {
      public Settings {
        double[] values = {
            slowErrorMeters,
            pauseErrorMeters,
            slowHeadingRadians,
            pauseHeadingRadians,
            positionToleranceMeters,
            headingToleranceRadians,
            speedToleranceMps,
            omegaToleranceRadPerSec,
            settleSeconds,
            rateIncreasePerSecond,
            timeoutMultiplier,
            timeoutExtraSeconds
        };
        for (double v : values) {
          if (!Double.isFinite(v) || v <= 0)
            throw new IllegalArgumentException("Settings must be finite and positive");
        }
        if (pauseErrorMeters <= slowErrorMeters
            || pauseHeadingRadians <= slowHeadingRadians
            || timeoutMultiplier < 1)
          throw new IllegalArgumentException("Invalid error thresholds/timeout");
      }

      public static Settings defaults() {
        return new Settings(
            0.10, // Position error where slowing begins: meters
            0.45, // Position error where reference pauses: meters
            Math.toRadians(10), // Heading error where slowing begins
            Math.toRadians(45), // Heading error where reference pauses
            0.10, // Allowed position error at a stopped endpoint: meters
            Math.toRadians(5), // Allowed heading error at a stopped endpoint
            0.15, // Maximum translation speed to count as stopped: m/s
            0.25, // Maximum rotation speed to count as stopped: rad/s
            0.10, // Time all four conditions must stay satisfied: seconds
            2.0, // How quickly reference playback speeds back up: rate/second
            4.0, // Section timeout multiplier
            3.0); // Additional section timeout allowance: seconds
      }
    }

    public record Status(
        int pathIndex,
        String pathName,
        int sectionIndex,
        double referenceTimeSeconds,
        double referenceRate,
        double positionErrorMeters,
        double headingErrorRadians,
        boolean settling,
        boolean finished,
        boolean timedOut,
        Pose2d targetPose) {
    }

    private record Section(
        PathPlannerMPCReference preview, int firstPath, int lastPath, double[] pathEndTimes) {
    }

    private final List<String> names;
    private final List<Section> sections;
    private final MPCConfig config;
    private final Settings settings;
    private int sectionIndex;
    private double referenceTime;
    private double referenceRate;
    private double sectionWallTime;
    private double stableTime;
    private boolean finished;
    private boolean timedOut;
    private double positionError;
    private double headingError;

    public MPCPathProgress(
        List<PathPlannerTrajectory> trajectories,
        List<String> names,
        MPCConfig config,
        Settings settings) {
      this.config = Objects.requireNonNull(config);
      this.settings = Objects.requireNonNull(settings);
      this.names = List.copyOf(names);
      List<PathPlannerTrajectory> copy = List.copyOf(trajectories);
      if (copy.isEmpty() || copy.size() != this.names.size()) {
        throw new IllegalArgumentException("Provide one name per trajectory");
      }
      List<Section> result = new ArrayList<>();
      int first = 0;
      // A stopped endpoint separates preview sections. The next departure stays
      // hidden until
      // settled.
      for (int i = 0; i < copy.size(); i++) {
        var end = copy.get(i).getEndState().fieldSpeeds;
        boolean stop = Math.hypot(end.vxMetersPerSecond, end.vyMetersPerSecond) < 0.1
            && Math.abs(end.omegaRadiansPerSecond) < 0.1;
        if (stop || i == copy.size() - 1) {
          var group = copy.subList(first, i + 1);
          var preview = new PathPlannerMPCReference(group); // Validates final stop and internal joins.
          double[] boundaries = new double[group.size()];
          double sum = 0;
          for (int j = 0; j < group.size(); j++) {
            sum += group.get(j).getTotalTimeSeconds();
            boundaries[j] = sum;
          }
          result.add(new Section(preview, first, i, boundaries));
          first = i + 1;
        }
      }
      // The preview validator only sees within-section joins; check joins across
      // stops as well.
      for (int i = 1; i < copy.size(); i++) {
        Pose2d a = copy.get(i - 1).getEndState().pose;
        Pose2d b = copy.get(i).getInitialPose();
        if (distance(a, b) > 0.05 || Math.abs(angleDifference(a, b)) > Math.toRadians(5)) {
          throw new IllegalArgumentException("Pose discontinuity before " + this.names.get(i));
        }
      }
      sections = List.copyOf(result);
    }

    /** Fixed-period compatibility overload. */
    public boolean update(MPCState measured) {
      return update(measured, config.periodSeconds());
    }

    /**
     * Returns true when advancing to a new section. elapsedSeconds is real time
     * since last update.
     */
    public boolean update(MPCState measured, double elapsedSeconds) {
      Objects.requireNonNull(measured);
      if (!Double.isFinite(elapsedSeconds) || elapsedSeconds <= 0) {
        throw new IllegalArgumentException("elapsedSeconds must be finite and positive");
      }
      if (finished)
        return false;
      double dt = elapsedSeconds;
      sectionWallTime += dt;
      Section section = sections.get(sectionIndex);
      double duration = section.preview().getTotalTimeSeconds();
      Pose2d pose = measured.pose();
      MPCReference target = section.preview().sample(referenceTime);
      positionError = distance(pose, target.pose());
      headingError = Math.abs(angleDifference(pose, target.pose()));

      if (referenceTime >= duration) {
        referenceRate = 0;
        boolean settled = positionError <= settings.positionToleranceMeters()
            && headingError <= settings.headingToleranceRadians()
            && Math.hypot(measured.fieldVxMps(), measured.fieldVyMps()) <= settings.speedToleranceMps()
            && Math.abs(measured.omegaRadPerSec()) <= settings.omegaToleranceRadPerSec();
        stableTime = settled ? stableTime + Math.min(dt, config.periodSeconds()) : 0;
        if (stableTime + 1e-9 >= settings.settleSeconds()) {
          if (sectionIndex + 1 == sections.size())
            finished = true;
          else {
            sectionIndex++;
            referenceTime = referenceRate = sectionWallTime = stableTime = 0;
            positionError = distance(pose, sections.get(sectionIndex).preview().sample(0).pose());
            headingError = Math.abs(
                angleDifference(pose, sections.get(sectionIndex).preview().sample(0).pose()));
            return true;
          }
        }
      } else {
        double requestedRate = Math.min(
            rateForError(
                positionError, settings.slowErrorMeters(), settings.pauseErrorMeters()),
            rateForError(
                headingError, settings.slowHeadingRadians(), settings.pauseHeadingRadians()));
        // Drop the reference rate immediately when behind; resume smoothly after
        // recovery.
        referenceRate = Math.min(requestedRate, referenceRate + settings.rateIncreasePerSecond() * dt);
        referenceTime = Math.min(duration, referenceTime + dt * referenceRate);
        stableTime = 0;
      }
      if (!finished
          && sectionWallTime > duration * settings.timeoutMultiplier() + settings.timeoutExtraSeconds()) {
        timedOut = finished = true; // Abort the entire sequence; never skip an unfinished stop.
        referenceRate = 0;
      }
      return false;
    }

    /**
     * Samples ahead on the governed clock. Reference velocities are scaled with
     * that same clock.
     * After the section endpoint all future samples hold its stopped pose, even if
     * more paths
     * exist.
     */
    public List<MPCReference> sampleHorizon() {
      return sampleHorizon(config.periodSeconds());
    }

    public List<MPCReference> sampleHorizon(double predictionPeriodSeconds) {
      if (!Double.isFinite(predictionPeriodSeconds) || predictionPeriodSeconds <= 0) {
        throw new IllegalArgumentException("predictionPeriodSeconds must be finite and positive");
      }
      List<MPCReference> horizon = new ArrayList<>(config.horizonSteps());
      var preview = sections.get(sectionIndex).preview();
      for (int k = 1; k <= config.horizonSteps(); k++) {
        var s = preview.sample(referenceTime + k * predictionPeriodSeconds * referenceRate);
        horizon.add(
            new MPCReference(
                s.pose(),
                s.fieldVxMps() * referenceRate,
                s.fieldVyMps() * referenceRate,
                s.omegaRadPerSec() * referenceRate));
      }
      return horizon;
    }

    public boolean isFinished() {
      return finished;
    }

    public boolean hasTimedOut() {
      return timedOut;
    }

    public Status getStatus() {
      Section section = sections.get(sectionIndex);
      int local = 0;
      while (local + 1 < section.pathEndTimes().length
          && referenceTime >= section.pathEndTimes()[local])
        local++;
      int pathIndex = section.firstPath() + local;
      return new Status(
          pathIndex,
          names.get(pathIndex),
          sectionIndex,
          referenceTime,
          referenceRate,
          positionError,
          headingError,
          referenceTime >= section.preview().getTotalTimeSeconds(),
          finished,
          timedOut,
          section.preview().sample(referenceTime).pose());
    }

    private static double rateForError(double error, double slow, double pause) {
      return Math.max(0, Math.min(1, (pause - error) / (pause - slow)));
    }

    private static double distance(Pose2d a, Pose2d b) {
      return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private static double angleDifference(Pose2d a, Pose2d b) {
      return Math.atan2(
          Math.sin(b.getRotation().getRadians() - a.getRotation().getRadians()),
          Math.cos(b.getRotation().getRadians() - a.getRotation().getRadians()));
    }
  }

  /**
   * Regenerates reference timing without changing path geometry or files. Moving
   * joins are retained
   * only when travel tangents align; sharp joins and saved zero-speed joins
   * become real stops.
   * Apply alliance/mirror transforms BEFORE calling this method.
   */
  public static final class MPCPathPreparation {
    public record Limits(
        double maxSpeedMps,
        double maxAccelerationMps2,
        double maxOmegaRadPerSec,
        double maxAlphaRadPerSec2,
        double maxMovingJoinAngleRadians) {
      public Limits {
        for (double x : new double[] {
            maxSpeedMps,
            maxAccelerationMps2,
            maxOmegaRadPerSec,
            maxAlphaRadPerSec2,
            maxMovingJoinAngleRadians
        }) {
          if (!Double.isFinite(x) || x <= 0)
            throw new IllegalArgumentException("Limits must be finite and positive");
        }
      }

      /**
       * Conservative reference limits leave authority for the controller to correct
       * tracking error.
       */
      public static Limits conservative(MPCConfig config) {
        var limits = config.limits();
        return new Limits(
            limits.maxLinearSpeedMps(),
            limits.maxLinearAccelerationMps2(),
            limits.maxAngularSpeedRadPerSec(),
            limits.maxAngularAccelerationRadPerSec2(),
            Math.toRadians(5));
      }
    }

    private MPCPathPreparation() {
    }

    public static List<PathPlannerTrajectory> prepare(
        List<PathPlannerPath> paths, RobotConfig robot, Limits limits) {
      paths = List.copyOf(paths);
      Objects.requireNonNull(robot);
      Objects.requireNonNull(limits);
      if (paths.isEmpty())
        throw new IllegalArgumentException("No paths");
      int size = paths.size();
      double[] boundarySpeeds = new double[size + 1]; // Initial/final stops remain zero.
      for (int i = 0; i < size; i++) {
        if (paths.get(i).isChoreoPath() || paths.get(i).getWaypoints().size() < 2) {
          throw new IllegalArgumentException(
              "Only geometric PathPlanner paths with at least two waypoints are supported");
        }
        if (paths.get(i).getIdealStartingState() == null) {
          throw new IllegalArgumentException("Missing ideal starting state: " + paths.get(i).name);
        }
        if (i == 0)
          continue;
        var previous = paths.get(i - 1);
        var next = paths.get(i);
        double before = previous.getGoalEndState().velocityMPS();
        double after = next.getIdealStartingState().velocityMPS();
        Translation2d a = endTangent(previous);
        Translation2d b = startTangent(next);
        double angle = Math.abs(
            Math.atan2(
                a.getX() * b.getY() - a.getY() * b.getX(),
                a.getX() * b.getX() + a.getY() * b.getY()));
        if (before >= 0.1
            && after >= 0.1
            && a.getNorm() > 1e-9
            && b.getNorm() > 1e-9
            && angle <= limits.maxMovingJoinAngleRadians()) {
          boundarySpeeds[i] = Math.min(
              Math.min(before, after),
              Math.min(
                  endpointSpeedCap(previous, true, limits),
                  endpointSpeedCap(next, false, limits)));
        }
      }
      List<PathPlannerTrajectory> result = new ArrayList<>();
      for (int i = 0; i < size; i++) {
        PathPlannerPath original = paths.get(i);
        List<ConstraintsZone> zones = new ArrayList<>();
        for (ConstraintsZone z : original.getConstraintZones()) {
          zones.add(
              new ConstraintsZone(z.minPosition(), z.maxPosition(), cap(z.constraints(), limits)));
        }
        var path = new PathPlannerPath(
            original.getWaypoints(),
            original.getRotationTargets(),
            original.getPointTowardsZones(),
            zones,
            original.getEventMarkers(),
            cap(original.getGlobalConstraints(), limits),
            new IdealStartingState(
                boundarySpeeds[i], original.getIdealStartingState().rotation()),
            new GoalEndState(boundarySpeeds[i + 1], original.getGoalEndState().rotation()),
            original.isReversed());
        path.name = original.name;
        path.preventFlipping = true; // These are already transformed references.
        result.add(path.getIdealTrajectory(robot).orElseThrow());
      }
      return List.copyOf(result);
    }

    private static PathConstraints cap(PathConstraints c, Limits l) {
      return new PathConstraints(
          Math.min(c.maxVelocityMPS(), l.maxSpeedMps()),
          Math.min(c.maxAccelerationMPSSq(), l.maxAccelerationMps2()),
          Math.min(c.maxAngularVelocityRadPerSec(), l.maxOmegaRadPerSec()),
          Math.min(c.maxAngularAccelerationRadPerSecSq(), l.maxAlphaRadPerSec2()),
          c.nominalVoltageVolts(),
          false);
    }

    private static double endpointSpeedCap(PathPlannerPath path, boolean end, Limits limits) {
      double position = end ? path.getWaypoints().size() - 1 : 0;
      double speed = Math.min(path.getGlobalConstraints().maxVelocityMPS(), limits.maxSpeedMps());
      for (ConstraintsZone zone : path.getConstraintZones()) {
        if (position >= zone.minPosition() && position <= zone.maxPosition()) {
          speed = Math.min(speed, zone.constraints().maxVelocityMPS());
        }
      }
      return speed;
    }

    private static Translation2d startTangent(PathPlannerPath path) {
      var waypoint = path.getWaypoints().get(0);
      return waypoint.nextControl() == null
          ? new Translation2d()
          : waypoint.nextControl().minus(waypoint.anchor());
    }

    private static Translation2d endTangent(PathPlannerPath path) {
      var points = path.getWaypoints();
      var waypoint = points.get(points.size() - 1);
      return waypoint.prevControl() == null
          ? new Translation2d()
          : waypoint.anchor().minus(waypoint.prevControl());
    }
  }

  /**
   * Follows path files in order using the existing MPCController, a governed
   * reference clock, and
   * completion barriers at stopped endpoints. Does not run .auto named commands
   * or reset odometry.
   * Call at the configured robot loop period. Constructing does not load or move
   * anything.
   */
  public static final class MPCPathFollower extends Command {
    public record Telemetry(
        MPCPathProgress.Status progress,
        MPCController.Diagnostics optimizer,
        Pose2d[] predictedPoses,
        ChassisSpeeds commandedRobotSpeeds) {
    }

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedSupplier;
    private final Consumer<ChassisSpeeds> output;
    private final RobotConfig robotConfig;
    private final MPCConfig config;
    private final MPCPathProgress.Settings settings;
    private final MPCController controller;
    private final MPCPathPreparation.Limits referenceLimits;
    private final List<String> names;
    private final boolean mirror;
    private final Consumer<Telemetry> telemetry;
    private MPCPathProgress progress;
    private boolean failed;
    private double lastUpdateSeconds;
    private List<PathPlannerPath> loadedPaths = List.of();
    private int displayedPathIndex;

    public MPCPathFollower(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedSupplier,
        Consumer<ChassisSpeeds> output,
        RobotConfig robotConfig,
        MPCConfig config,
        Subsystem drive,
        String... pathNames) {
      this(
          poseSupplier,
          speedSupplier,
          output,
          robotConfig,
          config,
          MPCPathProgress.Settings.defaults(),
          false,
          value -> {
          },
          drive,
          pathNames);
    }

    public MPCPathFollower(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedSupplier,
        Consumer<ChassisSpeeds> output,
        RobotConfig robotConfig,
        MPCConfig config,
        MPCPathProgress.Settings settings,
        boolean mirror,
        Consumer<Telemetry> telemetry,
        Subsystem drive,
        String... pathNames) {
      this(
          poseSupplier,
          speedSupplier,
          output,
          robotConfig,
          config,
          settings,
          MPCController.VelocityResponse.ideal(),
          MPCPathPreparation.Limits.conservative(config),
          mirror,
          telemetry,
          drive,
          pathNames);
    }

    public MPCPathFollower(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedSupplier,
        Consumer<ChassisSpeeds> output,
        RobotConfig robotConfig,
        MPCConfig config,
        MPCPathProgress.Settings settings,
        MPCController.VelocityResponse response,
        boolean mirror,
        Consumer<Telemetry> telemetry,
        Subsystem drive,
        String... pathNames) {
      this(
          poseSupplier,
          speedSupplier,
          output,
          robotConfig,
          config,
          settings,
          response,
          MPCPathPreparation.Limits.conservative(config),
          mirror,
          telemetry,
          drive,
          pathNames);
    }

    public MPCPathFollower(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedSupplier,
        Consumer<ChassisSpeeds> output,
        RobotConfig robotConfig,
        MPCConfig config,
        MPCPathProgress.Settings settings,
        MPCController.VelocityResponse response,
        MPCPathPreparation.Limits referenceLimits,
        boolean mirror,
        Consumer<Telemetry> telemetry,
        Subsystem drive,
        String... pathNames) {
      this(
          poseSupplier,
          speedSupplier,
          output,
          robotConfig,
          config,
          settings,
          response,
          referenceLimits,
          TrackingTolerance.defaults(),
          mirror,
          telemetry,
          drive,
          pathNames);
    }

    public MPCPathFollower(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedSupplier,
        Consumer<ChassisSpeeds> output,
        RobotConfig robotConfig,
        MPCConfig config,
        MPCPathProgress.Settings settings,
        MPCController.VelocityResponse response,
        MPCPathPreparation.Limits referenceLimits,
        TrackingTolerance trackingTolerance,
        boolean mirror,
        Consumer<Telemetry> telemetry,
        Subsystem drive,
        String... pathNames) {
      this.poseSupplier = Objects.requireNonNull(poseSupplier);
      this.speedSupplier = Objects.requireNonNull(speedSupplier);
      this.output = Objects.requireNonNull(output);
      this.robotConfig = Objects.requireNonNull(robotConfig);
      this.config = Objects.requireNonNull(config);
      this.settings = Objects.requireNonNull(settings);
      this.mirror = mirror;
      this.telemetry = Objects.requireNonNull(telemetry);
      names = List.copyOf(Arrays.asList(pathNames));
      if (names.isEmpty())
        throw new IllegalArgumentException("No path names");
      controller = new MPCController(config, response, trackingTolerance);
      this.referenceLimits = Objects.requireNonNull(referenceLimits);
      addRequirements(Objects.requireNonNull(drive));
      setName("MPC Paths: " + String.join(", ", names));
    }

    @Override
    public void initialize() {
      failed = false;
      progress = null;
      controller.reset();
      try {
        boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        List<PathPlannerPath> paths = new ArrayList<>();
        List<Translation2d> rawEnds = new ArrayList<>();
        // Re-read deployed .path files for this run, even if another command loaded
        // them earlier.
        // This does not transfer local edits to the roboRIO; deployment must do that
        // first.
        PathPlannerPath.clearCache();
        for (String name : names) {
          PathPlannerPath path = PathPlannerPath.fromPathFile(name);
          Translation2d rawEnd = path.getWaypoints().get(path.getWaypoints().size() - 1).anchor();
          rawEnds.add(rawEnd);
          String diagnosticKey = "MPC/Loaded/" + name + "/";
          SmartDashboard.putNumber(diagnosticKey + "RawEndX", rawEnd.getX());
          SmartDashboard.putNumber(diagnosticKey + "RawEndY", rawEnd.getY());
          if (mirror)
            path = path.mirrorPath();
          if (red && !path.preventFlipping)
            path = path.flipPath();
          paths.add(path);
        }
        List<PathPlannerTrajectory> trajectories = MPCPathPreparation.prepare(paths, robotConfig, referenceLimits);
        // Log the exact trajectories supplied to this MPC, rather than an unrelated
        // AutoBuilder
        // path.
        loadedPaths = List.copyOf(paths);
        displayedPathIndex = 0;
        for (int i = 0; i < trajectories.size(); i++) {
          var trajectory = trajectories.get(i);
          Pose2d startPose = trajectory.getInitialPose();
          Pose2d endPose = trajectory.getEndState().pose;
          String key = "MPC/Loaded/" + names.get(i) + "/";
          SmartDashboard.putNumber(key + "EndX", endPose.getX());
          SmartDashboard.putNumber(key + "EndY", endPose.getY());
          SmartDashboard.putNumber(key + "EndHeadingDeg", endPose.getRotation().getDegrees());
          System.out.printf(
              java.util.Locale.ROOT,
              "[MPC FILE RELOAD] %s file=%s rawEnd=(%.3f, %.3f) "
                  + "start=(%.3f, %.3f) fieldEnd=(%.3f, %.3f, %.1f deg)%n",
              names.get(i),
              new java.io.File(
                  Filesystem.getDeployDirectory(),
                  "pathplanner/paths/" + names.get(i) + ".path")
                  .getAbsolutePath(),
              rawEnds.get(i).getX(),
              rawEnds.get(i).getY(),
              startPose.getX(),
              startPose.getY(),
              endPose.getX(),
              endPose.getY(),
              endPose.getRotation().getDegrees());
        }
        PathPlannerLogging.logActivePath(loadedPaths.get(0));
        progress = new MPCPathProgress(trajectories, names, config, settings);
        lastUpdateSeconds = Timer.getTimestamp();
      } catch (Exception e) {
        fail("Unable to prepare MPC paths", e);
      }
    }

    @Override
    public void execute() {
      if (failed || progress == null || progress.isFinished())
        return;
      try {
        Pose2d pose = poseSupplier.get();
        ChassisSpeeds speeds = speedSupplier.get();
        double now = Timer.getTimestamp();
        double elapsed = Math.max(1e-6, now - lastUpdateSeconds);
        lastUpdateSeconds = now;
        if (progress.update(MPCState.fromRobotRelative(pose, speeds), elapsed))
          controller.reset();
        if (progress.hasTimedOut()) {
          fail("MPC section timed out near " + progress.getStatus().pathName(), null);
          return;
        }
        // Estimate the next hold interval from the last cycle. Bound startup/stall
        // spikes;
        // progress/timeout bookkeeping above still uses the complete elapsed wall time.
        double predictionPeriod = Math.max(config.periodSeconds(), Math.min(0.06, elapsed));
        ChassisSpeeds command = progress.isFinished()
            ? new ChassisSpeeds()
            : controller.calculate(
                MPCState.fromRobotRelative(pose, speeds),
                progress.sampleHorizon(predictionPeriod),
                predictionPeriod);
        output.accept(command);
        var status = progress.getStatus();
        if (status.pathIndex() != displayedPathIndex) {
          displayedPathIndex = status.pathIndex();
          PathPlannerLogging.logActivePath(loadedPaths.get(displayedPathIndex));
        }
        PathPlannerLogging.logTargetPose(status.targetPose());
        telemetry.accept(
            new Telemetry(
                progress.getStatus(),
                controller.getDiagnostics(),
                controller.getPredictedPoses(),
                command));
      } catch (RuntimeException e) {
        fail("MPC execution failed", e);
      }
    }

    @Override
    public boolean isFinished() {
      return failed || (progress != null && progress.isFinished());
    }

    @Override
    public void end(boolean interrupted) {
      output.accept(new ChassisSpeeds());
      controller.reset();
    }

    private void fail(String message, Exception cause) {
      failed = true;
      output.accept(new ChassisSpeeds());
      if (cause == null)
        DriverStation.reportError(message, false);
      else
        DriverStation.reportError(message + ": " + cause.getMessage(), cause.getStackTrace());
    }
  }
}
