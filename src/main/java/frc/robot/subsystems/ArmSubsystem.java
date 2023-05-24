package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
    private TalonFX L_armMotor;
    //private TalonFX R_armMotor;

    /** Creates a new ExampleSubsystem. */
    public ArmSubsystem() {
        L_armMotor = new TalonFX(14);
        L_armMotor.setSelectedSensorPosition(0);
        L_armMotor.config_kP(0, 0.01);
        //R_armMotor = new TalonFX(16);
//double armPosition = armMotor.getSelectedSensorPosition(0);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase Raise(int degree) {
        L_armMotor.set(TalonFXControlMode.Position,((degree*((2048*(80*(60/18)))/360))));
        //L_armMotor.set(TalonFXControlMode.Position, (360*40));
        //L_armMotor.set(TalonFXControlMode.PercentOutput, 0.15);
        //L_armMotor.setSelectedSensorPosition(0);
        //L_armMotor.set(TalonFXControlMode.Position,(L_armMotor.getSelectedSensorPosition()+(2048*40)));

        
       // L_armMotor.setNeutralMode(NeutralMode.Brake);
        //L_armMotor.set(ControlMode.Position, (Swerve.RaiseTicksPerDegree*degree));
        //L_armMotor.set(TalonFXControlMode.Position, 2048*15);
    
        
        //R_armMotor.set(TalonFXControlMode., degree, null, degree);)

        
        //R_armMotor.set(ControlMode.Position, -(Swerve.RaiseTicksPerDegree*degree));
        

        System.out.println("ARM COMMAND IS WORKING GREAT");
        System.out.println("ARM COMMAND IS WORKING GREAT");
        System.out.println("ARM COMMAND IS WORKING GREAT");
        System.out.println("ARM COMMAND IS WORKING GREAT");
        System.out.println("ARM COMMAND IS WORKING GREAT");
        System.out.println("ARM COMMAND IS WORKING GREAT");
        System.out.println("ARM COMMAND IS WORKING GREAT");
        SmartDashboard.putString("TEST", "DEEZ");
        return runOnce(
            () -> {
                /* one-time action goes here */
            });
}

    public CommandBase Manual(int degree) {
        L_armMotor.set(TalonFXControlMode.Position,L_armMotor.getSelectedSensorPosition()-((degree*((2048*(80*(60/18)))/360))));

        

        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public CommandBase ManualUp(int degree) {
        L_armMotor.set(TalonFXControlMode.Position,L_armMotor.getSelectedSensorPosition()+((degree*((2048*(80*(60/18)))/360))));

        

        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("LEFTARM",L_armMotor.getSelectedSensorPosition());
        //SmartDashboard.putNumber("RIGHTARM",R_armMotor.getSelectedSensorPosition());
    
     
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}