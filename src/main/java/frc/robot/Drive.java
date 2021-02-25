package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;

public class Drive{
    double kP = 0;
    double kI = 0;
    double kD = 0;
    PigeonIMU gyro = new PigeonIMU(0);
    PIDController pid = new PIDController(kP, kI, kD, gyro, output;

    private static TalonSRX talon0;
    private static TalonSRX talon1;
    private static TalonSRX talon2;
    private static TalonSRX talon3;
    private static TalonSRX talon4;
    private static TalonSRX talon5;

    public Drive(int tal0, int tal1, int tal2, int tal3, int tal4, int tal5){
        talon0 = new TalonSRX(tal0);
        talon1 = new TalonSRX(tal1);
        talon2 = new TalonSRX(tal2);
        talon3 = new TalonSRX(tal3);
        talon4 = new TalonSRX(tal4);
        talon5 = new TalonSRX(tal5);

        talon0.configFactoryDefault();
        talon1.configFactoryDefault();
        talon2.configFactoryDefault();
        talon3.configFactoryDefault();
        talon4.configFactoryDefault();
        talon5.configFactoryDefault();

        talon0.setNeutralMode(NeutralMode.Brake);
        talon1.setNeutralMode(NeutralMode.Brake);
        talon2.setNeutralMode(NeutralMode.Brake);
        talon3.setNeutralMode(NeutralMode.Brake);
        talon4.setNeutralMode(NeutralMode.Brake);
        talon5.setNeutralMode(NeutralMode.Brake);

        talon0.follow(talon4);
        talon2.follow(talon4);
        talon1.follow(talon5);
        talon3.follow(talon5);

        talon1.setInverted(true);
        talon3.setInverted(true);
        talon5.setInverted(true);
    }
}

