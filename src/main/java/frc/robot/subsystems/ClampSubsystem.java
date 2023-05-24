package frc.robot.subsystems;


import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link edu.wpi.first.wpilibj.DoubleSolenoid}. */
public class ClampSubsystem extends SubsystemBase {
  
      DoubleSolenoid Gripper = new DoubleSolenoid(50,PneumaticsModuleType.REVPH, 1, 0);
          

  /** Grabs the hatch. */
  public CommandBase openGripper() {
    // implicitly require `this`
    System.out.println("CLAMPPER");
    System.out.println("CLAMPPER");
    System.out.println("CLAMPPER");
    System.out.println("CLAMPPER");
    Gripper.set(DoubleSolenoid.Value.kForward);
    // implicitly require `this`
    return runOnce(
                () -> {
                    /* one-time action goes here */
                });
  }

  /** Releases the hatch. */
  public CommandBase closeGripper() {
    Gripper.set(DoubleSolenoid.Value.kReverse);
    // implicitly require `this`
    return runOnce(
                () -> {
                    /* one-time action goes here */
                });
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish the solenoid state to telemetry.
    builder.addBooleanProperty("extended", () -> Gripper.get() == kForward, null);
  }
}
    
