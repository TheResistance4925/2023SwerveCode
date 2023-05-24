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

public class TheCopy extends SequentialCommandGroup {
    public TheCopy(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-2,0)), //List.of(new Translation2d(0.4,0), new Translation2d(0.4,0.4)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-2, 0, new Rotation2d(0)),
                config);
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("TEST", new PathConstraints(0.2, 0.1),false);
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PPSwerveControllerCommand TestAutoPPCommand = new PPSwerveControllerCommand(
            examplePath,
             s_Swerve::getPose, 
             Constants.Swerve.swerveKinematics,
             new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), s_Swerve::setModuleStates, s_Swerve);
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                examplePath,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            TestAutoPPCommand
            );
    }
}