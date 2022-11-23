// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;



public class SwerveDriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //Put in correct ports later, 
  //Ports: drive, turn, driveEncoderA, driveEncoderB, turnEncoderA, turnEncoderB
  private final SwerveModule frontRightModule = new SwerveModule(1, 1, 1, 1, 1, 1);
  private final SwerveModule frontLeftModule = new SwerveModule(1, 1, 1, 1, 1, 1);
  private final SwerveModule backRightModule = new SwerveModule(1, 1, 1, 1, 1, 1);
  private final SwerveModule backLeftModule = new SwerveModule(1, 1, 1, 1, 1, 1);

  //Creating Kinimatics objects
  //Put in correct measurments in meters
  private final Translation2d frontLeftLocation = new Translation2d(1, 1);
  private final Translation2d frontRightLocation = new Translation2d(1, -1);
  private final Translation2d backLeftLocation = new Translation2d(-1, 1);
  private final Translation2d backRightLocation = new Translation2d(-1, -1);

  private final SwerveDriveKinematics kinimatics = new SwerveDriveKinematics(
    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
  );
//add correct port
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(1);


  
  public SwerveDriveSubsystem() {
    zeroSensors();
  }

  private void zeroSensors(){
    pigeon.reset();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double forward, double left, double rotation){
    SwerveModuleState[] modules = getModules(forward, left, rotation);

    frontLeftModule.setState(modules[0]);
    frontRightModule.setState(modules[1]);
    backLeftModule.setState(modules[2]);
    backRightModule.setState(modules[3]);


  }

  public SwerveModuleState[] getModules (double forward, double left, double rotation){
    //forward and left are in m/s
    //rotation is in rad/s
    Rotation2d angle = pigeon.getRotation2d();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, rotation, angle);
    SwerveModuleState[] modules = kinimatics.toSwerveModuleStates(speeds);
    return modules;
 
  }
}
