// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file in
// the directory of this file.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.SwerveConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread {
  private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();
  private List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier;
  private static SparkMaxOdometryThread instance = null;

  public static SparkMaxOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkMaxOdometryThread();
    }
    return instance;
  }

  private SparkMaxOdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkMaxOdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / SwerveConstants.ODOMETRY_FREQUENCY);
    }
  }

  public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Swerve.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Swerve.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  private void periodic() {
    Swerve.odometryLock.lock();
    double timestamp = Logger.getRealTimestamp() / 1e6;
    try {
      double[] values = new double[signals.size()];
      boolean isValid = true;
      for (int i = 0; i < signals.size(); i++) {
        OptionalDouble value = signals.get(i).get();
        if (value.isPresent()) {
          values[i] = value.getAsDouble();
        } else {
          isValid = false;
          break;
        }
      }
      if (isValid) {
        for (int i = 0; i < queues.size(); i++) {
          queues.get(i).offer(values[i]);
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      }
    } finally {
      Swerve.odometryLock.unlock();
    }
  }
}