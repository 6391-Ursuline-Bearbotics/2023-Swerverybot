package frc.robot.commands.swervedrive2.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.Intake;
import java.util.HashMap;
import java.util.function.Supplier;

public class AutoMap {
  private HashMap<String, Command> eventMap = new HashMap<>();
  private HashMap<String, Supplier<Command>> eventMapGetter = new HashMap<>();

  public AutoMap(Intake intake, Arm arm) {

    eventMapGetter.put(
        "ArmLow",
        () -> Commands.run(() -> arm.extendArmLow(), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmHigh",
        () -> Commands.run(() -> arm.extendArmHigh(), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmStow", () -> Commands.run(() -> arm.stowArm(), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmMid",
        () -> Commands.run(() -> arm.extendArmMid(), arm).until(() -> arm.isAtSetpoint()));

    eventMapGetter.put("CubeGrab", () -> Commands.run(() -> intake.intakeCube()).withTimeout(5));

    eventMapGetter.put("ConeGrab", () -> Commands.run(() -> intake.intakeCone()).withTimeout(5));

    eventMapGetter.put("OuttakeCube", () -> Commands.run(() -> intake.outtakeCube()).withTimeout(0.1));

    eventMapGetter.put("OuttakeCone", () -> Commands.run(() -> intake.intakeCube()).withTimeout(0.1));

    eventMapGetter.forEach(
        (key, val) -> {
          eventMap.put(key, val.get());
        });
  }

  public HashMap<String, Command> getMap() {
    return eventMap;
  }

  public Command getCommandInMap(String key) {
    return eventMapGetter.get(key).get();
  }
}
