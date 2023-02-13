package frc.robot.subsystems.swervedrive2.swervelib.motors;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive2.swervelib.parser.PIDFConfig;

public class TalonFXSwerve extends SwerveMotor {

  /** SparkMAX Instance. */
  public WPI_TalonFX motor;
  public int m_id = 0;
  /** Factory default already occurred. */
  private boolean factoryDefaultOccurred = false;
  /**
   * PID wrapped or not, min and max pid output to use in degrees since Talon can't do it
   * automatically.
   */
  private boolean isPIDWrapped = false;

  private double minPIDInput;
  private double maxPIDInput;
  /** Conversion factors for pos and velocity since Talon can't do it automatically. */
  private double positionConversionFactor;

  private double velocityConversionFactor;

  /**
   * Initialize the swerve motor.
   *
   * @param id CAN ID of the SparkMax
   * @param isDriveMotor Is the motor being initialized a drive motor?
   */
  public TalonFXSwerve(int id, boolean isDriveMotor) {
    this.isDriveMotor = isDriveMotor;
    m_id = id;
    motor = new WPI_TalonFX(id);
    factoryDefaults();
    clearStickyFaults();

    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
  }

  /**
   * Put angle within a scope (ex. 22 deg put into 720-1080 with a scope of (0, 360)).
   *
   * <p>Assumes the general equation of some possible point of the interval is (interval*num)+offset
   * where offset is the positive offset from 0. First we solve for an initial number to scale down
   * the (scopeMin, scopeMax) to a standard interval that includes 0 (we solve for the offset here).
   * Then we formulate the interval including the reference r to be (interval*num+offset,
   * interval*(num+1)+offset) and r = interval*num+offset + a where a is an arbitary value. So, num
   * = (r-a-offset)/interval, which we can simply floor to compensate for the a (since -a/interval
   * is of (-1, 0)). Then we can find values to use with {@link MathUtil.inputModulus}.
   *
   * @param scopeReference Current Angle
   * @param input Target Angle
   * @param scopeMin minimum for defined scope
   * @param scopeMax maximum for defined scope
   * @return Closest angle within scope
   */
  private double placeInAppropriateScope(
      double scopeReference, double inputValue, double scopeMin, double scopeMax) {
    double interval = scopeMax - scopeMin;
    int numInitial = (int) Math.floor(scopeMax / (scopeMax - scopeMin));
    double offset = scopeMin - interval * numInitial;
    int numFinal = (int) Math.floor((scopeReference - offset) / interval);
    double output =
        MathUtil.inputModulus(
            inputValue, interval * numFinal + offset, interval * (numFinal + 1) + offset);
    return output;
  }

  
  private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    motor.configVoltageCompSaturation(nominalVoltage);
    motor.enableVoltageCompensation(true);
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in
   * conjunction with voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit) {
    var config = new SupplyCurrentLimitConfiguration();
    config.currentLimit = currentLimit;
    config.triggerThresholdCurrent = currentLimit;
    config.enable = true;
    motor.configSupplyCurrentLimit(config);
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate) {
    motor.configOpenloopRamp(rampRate);
    motor.configClosedloopRamp(rampRate);
  }

  /** Configure the factory defaults. */
  @Override
  public void factoryDefaults() {
    if (!factoryDefaultOccurred) {
      motor.configFactoryDefault();
      factoryDefaultOccurred = true;
    }
  }

  /** Clear the sticky faults on the motor controller. */
  @Override
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for
   * position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply.
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor) {
    this.positionConversionFactor = positionConversionFactor;
    this.velocityConversionFactor = positionConversionFactor / 600;
  }

  /**
   * Configure the PIDF values for the closed loop controller.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config) {
    int pidSlot = isDriveMotor ? TalonslotIdx.Velocity.ordinal() : TalonslotIdx.Position.ordinal();
    motor.config_kP(pidSlot, config.p);
    motor.config_kI(pidSlot, config.i);
    motor.config_kD(pidSlot, config.d);
    motor.config_kF(pidSlot, config.f);
    motor.config_IntegralZone(pidSlot, config.iz);
  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    isPIDWrapped = true;
    minPIDInput = minInput;
    maxPIDInput = maxInput;
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    motor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
  }

  /** Save the configurations from flash to EEPROM. */
  @Override
  public void burnFlash() {}

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput) {
    motor.set(percentOutput);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward) {
    setpoint =
        !isDriveMotor && isPIDWrapped
            ? placeInAppropriate0To360Scope(
                motor.getSelectedSensorPosition() * positionConversionFactor,
                setpoint)
            : setpoint;
    setpoint =
        isDriveMotor ? setpoint / velocityConversionFactor : setpoint / positionConversionFactor;
    motor.set(
        isDriveMotor ? TalonFXControlMode.Velocity : TalonFXControlMode.Position,
        setpoint,
        DemandType.ArbitraryFeedForward,
        feedforward);
  }

  /**
   * Get the velocity of the integrated encoder in RPM.
   *
   * @return velocity
   */
  @Override
  public double getVelocity() {
    return motor.getSelectedSensorVelocity() / velocityConversionFactor;
  }

  /**
   * Get the position of the integrated encoder. Put into 0-360
   *
   * @return Position
   */
  @Override
  public double getPosition() {
    var position = motor.getSelectedSensorPosition() % (360 / positionConversionFactor);
        // isDriveMotor
        //     ? motor.getSelectedSensorPosition()
        //     : MathUtil.wrap(
        //             motor.getSelectedSensorPosition(),
        //             -positionConversionFactor / 2,
        //             positionConversionFactor / 2);
    return position * positionConversionFactor;
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position.
   */
  @Override
  public void setPosition(double position) {
    motor.setSelectedSensorPosition(position);
  }

  /** CTRE Slots for PID configuration. */
  enum TalonslotIdx {
    Position,
    Velocity,
    Simulation
  }
}
