package swervelib.imu;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SwerveIMU interface for the Pigeon.
 */
public class PigeonTalonSwerve extends SwerveIMU
{

  /**
   * Pigeon v1 IMU device.
   */
  WPI_PigeonIMU imu;

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon, does not support CANBus.
   */
  public PigeonTalonSwerve(int canid)
  {
    TalonSRX talon = new TalonSRX(canid);
    imu = new WPI_PigeonIMU(talon);
    SmartDashboard.putData(imu);
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    imu.configFactoryDefault();
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    imu.clearStickyFaults();
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Angle in degrees.
   */
  @Override
  public void setYaw(double yaw)
  {
    imu.setYaw(yaw);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU, inverts them all if SwerveIMU is inverted.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {
    imu.getYawPitchRoll(yprArray);
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
