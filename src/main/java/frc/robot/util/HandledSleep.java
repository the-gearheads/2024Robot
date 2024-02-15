package frc.robot.util;

import frc.robot.Robot;

public class HandledSleep {
    public static void sleep(long delayMs) {
        if(Robot.isSimulation()) return;
        try {
            Thread.sleep(delayMs);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }
}
