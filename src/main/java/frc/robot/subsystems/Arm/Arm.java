// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  WPI_TalonFX armMotor;
  TalonFXConfiguration config = new TalonFXConfiguration();
  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Stow position in Falcon units
   */
  static final double STOW = 2.0;

  /**
   * High position in Falcon units
   */
  static final double HIGH = 2.0;

  /**
   * Middle position in Falcon units
   */
  static final double MID = 2.0;

  /**
   * Low position in Falcon units
   */
  static final double LOW = 2.0;

  /**
   * Proportional term of PID for arm position control
   */
  static final double kP = 2.0;

  /** Creates a new Arm. */
  public Arm() {
    armMotor = new WPI_TalonFX(0);
    armMotor.configFactoryDefault();
    armMotor.setInverted(true);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configVoltageCompSaturation(12.0);
    armMotor.enableVoltageCompensation(true);
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, ARM_CURRENT_LIMIT_A, ARM_CURRENT_LIMIT_A + 10, .1);
    config.slot0.kP = kP;
    armMotor.configAllSettings(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stowArm() {
    setArmPosition(STOW);
  }

  public void extendArmHigh() {
    setArmPosition(HIGH);
  }

  public void extendArmMid() {
    setArmPosition(MID);
  }

  public void extendArmLow() {
    setArmPosition(LOW);
  }

  public void setArmPower(double percent) {
    armMotor.set(percent);
  }

  private void setArmPosition(double position) {
    armMotor.set(TalonFXControlMode.Position, position);
  }
}
