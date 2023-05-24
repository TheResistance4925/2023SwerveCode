package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import java.lang.Math;




public class Lights extends SubsystemBase {
    private final AnalogInput ultraSonic = new AnalogInput(1);
    Spark blinkin = new Spark(4); //CNFIGURE THIS FOR PROPER LIGHTING
    private boolean OverR = false; 
  /** Creates a new ExampleSubsystem. */
  public Lights() {
    
    
    
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void set(double pwmVALUE){
    OverR = true;
    System.out.println("Color Change");
    //Timer.delay(1);
    blinkin.set(pwmVALUE);
    Timer.delay(2);
    OverR = false;
  }
  


  @Override
  public void periodic() {
    //System.out.println(OverR);
   double rawvalue = ultraSonic.getValue();
   SmartDashboard.putNumber("Distance", rawvalue);
   int GOALDISTANCE = 900; //DISTANCE WHEN IT TURNS 
   int MarginOfError = 100;
   // Distance you can be off for it to work WE NEED TO MEASURE THESE
   
        if(OverR==false){
         //System.out.println("FALSE");
          blinkin.set(-0.47);
   }}
      
 

  @Override
  public void simulationPeriodic() {
    ;
    // This method will be called once per scheduler run during simulation
  }
}