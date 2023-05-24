package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class exampleAuto extends SequentialCommandGroup {
    
    public exampleAuto(Swerve s_Swerve){


        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Simple Score", new PathConstraints(4, 3));
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PPSwerveControllerCommand TestAutoPPCommand = new PPSwerveControllerCommand(
            examplePath,
             s_Swerve::getPose, 
             Constants.Swerve.swerveKinematics,
             new PIDController(Constants.AutoConstants.kPXController, 0, 0), 
             new PIDController(Constants.AutoConstants.kPYController, 0, 0), 
             new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
              s_Swerve::setModuleStates,
               s_Swerve);


            
        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(examplePath.getInitialPose())),
            TestAutoPPCommand
            );
    }
}