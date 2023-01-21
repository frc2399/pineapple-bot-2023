package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;;
/**
 here's the drive train class that operates the drive train
 *
 */
public class DriveTrain extends SubsystemBase {

    public static CANSparkMax leftFrontMotorController;
    private static CANSparkMax rightFrontMotorController;
    private static CANSparkMax leftBackMotorController;
    private static CANSparkMax rightBackMotorController;
    public static PIDController turnController;
    public final double kP = 0.06;
    public final double kPSim = 0.5;
    static final double kI = 0;
    static final double kD = 0;
    static final double kF = 0;
    public static double outputSpeed;

    
    //static double kToleranceDegrees = 2.0f;
    /**
    *
    */

    private DifferentialDriveOdometry odometry;
    private DifferentialDrivetrainSim driveSim;
    private Field2d field = new Field2d();
    final ShuffleboardTab tab = Shuffleboard.getTab("Motor Diag");
    public static final GenericEntry angleErrorTolerance = Shuffleboard.getTab("Params").addPersistent("Angle Err Tol", 2).getEntry();
    public static final GenericEntry distanceErrorTolerance = Shuffleboard.getTab("Params").addPersistent("Distance Err Tol", 5).getEntry();
    public static final GenericEntry robotAngle = Shuffleboard.getTab("Driver").add("Angle of Robot", 0).getEntry();
    public static final GenericEntry angleErrorPValue = Shuffleboard.getTab("Params").add("angle err p", 0.01).getEntry();
    public static final GenericEntry encoderTickLeft = Shuffleboard.getTab("Testing").add("tick left", 0).getEntry();
    public static final GenericEntry encoderTickRight = Shuffleboard.getTab("Testing").add("tick right", 0).getEntry();
    public static final GenericEntry leftSpeedNTE = Shuffleboard.getTab("Testing").add("left robot speed", 0).getEntry();
    public static final GenericEntry rightSpeedNTE = Shuffleboard.getTab("Testing").add("right robot speed", 0).getEntry();

    public DriveTrain() {

        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID,MotorType.kBrushless);
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID,MotorType.kBrushless);
        leftBackMotorController = new CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_ID,MotorType.kBrushless);
        rightBackMotorController = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID,MotorType.kBrushless);

        leftFrontMotorController.restoreFactoryDefaults();
        rightFrontMotorController.restoreFactoryDefaults();
        leftBackMotorController.restoreFactoryDefaults();
        rightBackMotorController.restoreFactoryDefaults();

        leftFrontMotorController.setInverted(false);
        rightFrontMotorController.setInverted(true);

        //sets motor controllers following leaders
        leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, 180.0f);



        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
       
            field = new Field2d();

            // Ethan is suspicious and thinks we need to re-enable this but it doesn't matter
            SmartDashboard.putData("Field", field);

            field.setRobotPose(new Pose2d(9, 6.5, new Rotation2d(3.14/2)));
        }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }



    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.

       
        field.setRobotPose(odometry.getPoseMeters());

       
    

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        driveSim.update(0.02);

        // Update all of our sensors.
        

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {

        leftFrontMotorController.set(leftSpeed*Constants.DriveConstants.SLOWING_SPEED);
        rightFrontMotorController.set(rightSpeed*Constants.DriveConstants.SLOWING_SPEED);
        //leftBackMotorController.set(leftSpeed);
       
        leftSpeedNTE.setDouble(leftSpeed);
            rightSpeedNTE.setDouble(rightSpeed);
        
    }

    public static void teleopInit()
    {
        System.out.print(leftBackMotorController);

        leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);        
        leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }



}
