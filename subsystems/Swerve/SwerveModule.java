package frc.robot.subsystems.Swerve;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;


public class SwerveModule {
    //Actually tune PIDs
    private final PIDController drivePID = new PIDController(1, 1, 1);
    private final PIDController turnPID = new PIDController(1, 1, 1);

    private final WPI_TalonSRX driveMotor;
    private final WPI_TalonSRX turnMotor;
    private final Encoder driveEncoder;
    private final Encoder turnEncoder;
 
    public SwerveModule(
        int driveMotorPort, 
        int turnMotorPort, 
        int driveEncoderAPort, 
        int driveEncoderBPort, 
        int turnEncoderAPort,
        int turnEncoderBPort
        ){

        driveMotor = new WPI_TalonSRX(driveMotorPort);
        turnMotor = new WPI_TalonSRX(turnMotorPort);
        driveEncoder = new Encoder(driveEncoderAPort, driveEncoderBPort);
        turnEncoder = new Encoder(turnEncoderAPort, turnEncoderBPort);
        double wheelDiameter = Units.inchesToMeters(4.0);
        //Find this out later
        driveEncoder.setDistancePerPulse(wheelDiameter*Math.PI/4096);
        turnEncoder.setDistancePerPulse(Math.PI*2/4096);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setState(SwerveModuleState desiredState){
        //add feedfoward???

        SwerveModuleState newDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getDistance()));
   
        final double driveOutput = drivePID.calculate(driveEncoder.getRate(), newDesiredState.speedMetersPerSecond);
        final double turnOutput = turnPID.calculate(turnEncoder.getDistance(), newDesiredState.angle.getRadians());

        driveMotor.setVoltage(driveOutput);
        turnMotor.setVoltage(turnOutput);
   
   
   
    }

}
