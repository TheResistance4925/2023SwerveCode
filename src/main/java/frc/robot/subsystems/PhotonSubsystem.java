package frc.robot.subsystems;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;


import java.security.DrbgParameters.Reseed;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PhotonSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("OV5647");
  
  /** Creates a new ExampleSubsystem. */
  public PhotonSubsystem() {
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
  public CommandBase MoveOnAprilTag(Swerve s_Swerve) {
    var result = camera.getLatestResult();
    //var Y_difference = ((result.getBestTarget().getBestCameraToTarget().getY()));
    if (result.hasTargets()){
      System.out.println("BUTTON AND SEES");
      while (camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY() > 0.1  ){
        
        System.out.println("SEES AND CHARGING TOWARDS COOPER");
        if (camera.getLatestResult().hasTargets()){
          System.out.println("COOPER IJN SIGHT");
          //Timer.delay(0.1);
          s_Swerve.drive(new Translation2d(0,0.5), 0, true, false);

        }
      }
      while(camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY() < (-0.1)){
        System.out.println("SEES AND CHARGING TOWARDS COOPER");
        if (camera.getLatestResult().hasTargets()){
          System.out.println("COOPER IJN SIGHT");
          //Timer.delay(0.1);
          s_Swerve.drive(new Translation2d(0,-0.5), 0, true, false);}}}


    
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          
          
          /* one-time action goes here */
        });
  }

  @Override 
  public void periodic() {
    var result = camera.getLatestResult();
    if (result.hasTargets()){
      //System.out.println("HAS A VERY COOL TARGET");
      //System.out.println(result.getBestTarget().getFiducialId());
      SmartDashboard.putNumber("April Tag", result.getBestTarget().getFiducialId());
      SmartDashboard.putNumber("Yaw", result.getBestTarget().getYaw());
      SmartDashboard.putNumber("Skew", result.getBestTarget().getSkew());
      SmartDashboard.putNumber("Area", result.getBestTarget().getArea());
      SmartDashboard.putNumber("Pitch", result.getBestTarget().getPitch());
      SmartDashboard.putNumber("X", result.getBestTarget().getBestCameraToTarget().getX());
      SmartDashboard.putNumber("Y", result.getBestTarget().getBestCameraToTarget().getY());
      SmartDashboard.putNumber("Z", result.getBestTarget().getBestCameraToTarget().getZ());}
      
 }

  @Override
  public void simulationPeriodic() {
    ;
    // This method will be called once per scheduler run during simulation
  }
}