package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;



public class SwerveSubsystem extends SubsystemBase {

    private final GenericEntry sb_gyro, sb_coord;
    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontLeftCANCoderId,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontRightCANCoderId,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackLeftCANCoderId,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackRightCANCoderId,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);  

    public final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics, getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            }, new Pose2d(0, 0, new Rotation2d()));


    public SwerveSubsystem() {

        sb_gyro = Shuffleboard.getTab("Driver")
            .add("Gyro", 0.0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 2)
            .withSize(3, 3)
            .getEntry();

        sb_coord = Shuffleboard.getTab("Driver")
            .add("Coordinates", "")
            .withPosition(8, 3)
            .withSize(4, 1)
            .getEntry();

        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.setAngleAdjustment(180);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    3.6, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        });
        sb_gyro.setDouble(getHeading());
        sb_coord.setString(getPose().toString());
    }


    public Command zeroHeading() {
        System.out.println("Gyro Reset");
        return Commands.runOnce(() -> gyro.reset()); // Returns a command to be used on button press
    }

    public Command zeroCoords() {
        System.out.println("Coords Reset");
        return Commands.runOnce(() -> resetPose(new Pose2d(0,0,getRotation2d())));
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          }, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            new SwerveModuleState[] {
                backLeft.getState(),
                backRight.getState(),
                frontLeft.getState(),
                frontRight.getState()
            }
        );
    }

    public void driveRobotRelative(ChassisSpeeds speed) {
        //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speed, 0.02); is this needed?
        SwerveModuleState states[] = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
        setModuleStates(states);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public double getHeading() {
        return Math.IEEEremainder(DriveConstants.kGyroReversed ? gyro.getAngle() * -1 : gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }



    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[3]);
        frontRight.setDesiredState(desiredStates[1]);
        // frontLeft.setDesiredState(desiredStates[2]);
        // frontRight.setDesiredState(desiredStates[1]);
        // backLeft.setDesiredState(desiredStates[0]);
        // backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] newModuleStates = {
            frontLeft.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            backRight.getModuleState()
        };
        return newModuleStates;
    }
}
