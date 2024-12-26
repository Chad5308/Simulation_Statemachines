package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class DriveOdometry extends Thread
{
    private final Lock signalsLock = new ReentrantLock();

    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();


    private final List<SparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
    private final List<Queue<Double>> sparkQueues = new ArrayList<>();


    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static boolean isCANFD = DriveConstants.kCANBus.isNetworkFD();
  
    private static DriveOdometry instance = null;
    // private Notifier notifier = new Notifier(this::run);
  
    public static DriveOdometry getInstance()
    {
        if (instance == null)
        {
        instance = new DriveOdometry();
        }
        return instance;
    }
  
    private DriveOdometry()
    {
    //   notifier.setName("OdometryThread");
        setName("OdometryThread");
        setDaemon(true);
    }
  
    public void start()
    {
        if (timestampQueues.size() > 0)
        {
        super.start();
        // notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency);
        }
    }
  
    /** Registers a Spark signal to be read from the thread. */
    public Queue<Double> registerSparkSignal(SparkBase spark, DoubleSupplier signal)
    {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
        sparks.add(spark);
        sparkSignals.add(signal);
        sparkQueues.add(queue);
        } finally {
        Drive.odometryLock.unlock();
        }
        return queue;
    }

    /** Registers a Phoenix signal to be read from the thread. */
    public Queue<Double> registerTalonSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try {
        BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
        System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
        newSignals[phoenixSignals.length] = signal;
        phoenixSignals = newSignals;
        phoenixQueues.add(queue);
        } finally {
        signalsLock.unlock();
        Drive.odometryLock.unlock();
        }
        return queue;
    }
  
    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal)
    {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try
        {
        genericSignals.add(signal);
        genericQueues.add(queue);
        } finally
        {
        signalsLock.unlock();
        Drive.odometryLock.unlock();
        }
        return queue;
    }
  
    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue()
    {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try
        {
        timestampQueues.add(queue);
        } finally
        {
        Drive.odometryLock.unlock();
        }
        return queue;
    }
  
    public void run()
    {
        while (true)
        {
            signalsLock.lock();
            try {
                if (isCANFD && phoenixSignals.length > 0) {
                BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
                } else {
                // "waitForAll" does not support blocking on multiple signals with a bus
                // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                // behavior is provided by the documentation.
                Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
                if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }


            // Save new data to queues
            Drive.odometryLock.lock();
            try
            {
                // Get sample timestamp
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;

                for (BaseStatusSignal signal : phoenixSignals)
                {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0)
                {
                    timestamp -= totalLatency / phoenixSignals.length;
                }


                // Read Spark values, mark invalid in case of error
                double[] sparkValues = new double[sparkSignals.size()];
                boolean isValid = true;
                for (int i = 0; i < sparkSignals.size(); i++) {
                sparkValues[i] = sparkSignals.get(i).getAsDouble();
                if (sparks.get(i).getLastError() != REVLibError.kOk) {
                    isValid = false;
                }
                }

                // If valid, add values to queues
                for (int i = 0; i < phoenixSignals.length; i++)
                {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }

                if (isValid) {
                for (int i = 0; i < sparkSignals.size(); i++) {
                    sparkQueues.get(i).offer(sparkValues[i]);
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
                }
            } finally
            {
                Drive.odometryLock.unlock();
            }
        }
    }


}
