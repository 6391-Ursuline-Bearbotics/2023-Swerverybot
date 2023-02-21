// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;

  private enum gamepiece {
    cone(1),
    cube(-1),
    nothing(0);

    private int numVal;

    gamepiece(int numVal) {
      this.numVal = numVal;
    }

    public int getNumVal() {
      return numVal;
    }
  }

  private gamepiece piece = gamepiece.cone;

  /** How many amps the intake can use while picking up */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /** How many amps the intake can use while holding */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /** Percent output for intaking */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /** Percent output for holding */
  static final double INTAKE_HOLD_POWER = 0.07;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCone() {
    piece = gamepiece.cone;
  }

  public void setCube() {
    piece = gamepiece.cube;
  }

  public void intakeCube() {
    setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void intakeCone() {
    setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void outtakeCube() {
    setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void outtakeCone() {
    setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public void holdCube() {
    setIntakeMotor(INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
  }

  public void holdCone() {
    setIntakeMotor(-INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
  }

  public void pickup() {
    setIntakeMotor(INTAKE_OUTPUT_POWER * piece.getNumVal(), INTAKE_CURRENT_LIMIT_A);
  }

  public void place() {
    setIntakeMotor(-INTAKE_OUTPUT_POWER * piece.getNumVal(), INTAKE_CURRENT_LIMIT_A);
  }

  private void setIntakeMotor(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
  }
}
