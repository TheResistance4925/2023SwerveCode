package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class extendoSubsystem extends SubsystemBase {
    private TalonFX extendMotor;

    /** Creates a new ExampleSubsystem. */
    public extendoSubsystem() {
        extendMotor = new TalonFX(15);
        extendMotor.config_kP(0, 0.04);
        


    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase Extendcmd() {
        extendMotor.setSelectedSensorPosition(0);
        extendMotor.set(TalonFXControlMode.Position,(12*2048));
    

        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }
    public CommandBase Retractcmd() {
        extendMotor.setSelectedSensorPosition(0);
        extendMotor.set(TalonFXControlMode.Position,-(12*2048));
        System.out.println(" RETRACTING");
        System.out.println(" RETRACTING");
        System.out.println(" RETRACTING");
        System.out.println(" RETRACTING");
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
       //double motorOutput = armMotor.getMotorOutputPercent();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}