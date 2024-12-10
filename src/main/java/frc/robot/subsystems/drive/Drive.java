package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase
{
    static final Lock odometryLock = new ReentrantLock();
    static final double ODOMETRY_FREQUENCY = new CANBus(DriveConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
}
