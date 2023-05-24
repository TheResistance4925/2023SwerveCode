package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */


 
public class RobotContainer {

 
  


    /* Controllers */
    private final Joystick driver = new Joystick(0);
    // private final Joystick ROTJOY = new Joystick(1);
    private final GenericHID ButtonBoard = new GenericHID(1);
    

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;//POTENTIAL ROTATION AXIS?

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton CubeLED =  new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton ConeLED =  new JoystickButton(driver, XboxController.Button.kA.value);

    /*Button Board Buttons */
    private final JoystickButton extend  = new JoystickButton(ButtonBoard,6 );
    private final JoystickButton retract = new JoystickButton(ButtonBoard, 10);
    private final JoystickButton closeGripper = new JoystickButton(ButtonBoard, 9);
    private final JoystickButton openGripper = new JoystickButton(ButtonBoard, 8);
    //arm positins
    private final JoystickButton TopPoleArm = new JoystickButton(ButtonBoard, 7);
    private final JoystickButton TopArm = new JoystickButton(ButtonBoard, 3);
    private final JoystickButton MiddleArm = new JoystickButton(ButtonBoard,5);
    private final JoystickButton FloorArm = new JoystickButton(ButtonBoard, 4);

    private final JoystickButton balance = new JoystickButton(ButtonBoard, 11);
    //private final JoystickButton scanforapril = new JoystickButton(ButtonBoard, 12);
    private final JoystickButton Up_Manuel_Override = new JoystickButton(ButtonBoard, 2);
    private final JoystickButton Down_Manuel_override = new JoystickButton(ButtonBoard, 1);
    /* Subsystems */
    final Swerve s_Swerve = new Swerve();
    private final PhotonSubsystem photon = new PhotonSubsystem();

    private final extendoSubsystem extendo = new extendoSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ClampSubsystem Grip = new ClampSubsystem();
    private final Lights LEDController = new Lights();
    
    //private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    

    




    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), //ROTATION AXIS?
                () -> robotCentric.getAsBoolean()
            )
        );
    
        // Configure the button bindings
        configureButtonBindings();
        
        final Command m_BALANCE = new BALANCE(s_Swerve);
        final Command m_complexAuto = new exampleAuto(s_Swerve);
        final Command m_FarAuto = new FarAuto(s_Swerve);
        final Command m_ShortAuto = new ShortAuto(s_Swerve);

        m_autoChooser.setDefaultOption("Default", m_complexAuto);
        m_autoChooser.addOption("No Auto", null);
        m_autoChooser.addOption("Balance Auto", m_BALANCE);
        m_autoChooser.addOption("Short Auto", m_ShortAuto);
        m_autoChooser.addOption("Far Auto", m_FarAuto);
        SmartDashboard.putData("Auto", m_autoChooser);
    }
       



    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        //StoreArm.onTrue(new InstantCommand(() -> armSubsystem.Raise(25)));

        //6666666666666666666666666699999999999999999999999999999999999999999999999999
        MiddleArm.onTrue(new InstantCommand(() -> armSubsystem.Raise(65)));
        TopArm.onTrue(new InstantCommand(() -> armSubsystem.Raise(80)));
        TopPoleArm.onTrue(new InstantCommand(() -> armSubsystem.Raise(85)));
        FloorArm.onTrue(new InstantCommand(() -> armSubsystem.Raise(21)));
        //open closey stuff
        extend.onTrue(new InstantCommand(() -> extendo.Extendcmd()));
        retract.onTrue(new InstantCommand(() -> extendo.Retractcmd()));
        closeGripper.onTrue(new InstantCommand(() -> Grip.closeGripper()));
        openGripper.onTrue(new InstantCommand(() -> Grip.openGripper()));
        //StoreToggle.onTrue(new InstantCommand(() -> armSubsystem.Raise(10)));
        CubeLED.onTrue(new InstantCommand(()-> LEDController.set(-0.09 )));
        Up_Manuel_Override.onTrue(new InstantCommand(()-> armSubsystem.ManualUp(8)));
        Down_Manuel_override.onTrue(new InstantCommand(()-> armSubsystem.Manual(1)));
        
        ConeLED.onTrue(new InstantCommand(()-> LEDController.set(-0.07)));
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     * 
     
     */
    
        


    
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return m_autoChooser.getSelected();
      }

}
