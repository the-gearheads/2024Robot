package frc.robot.util;

import frc.robot.Robot;

public class HandledSleep {
    public static void sleep(long delayMs) {
        if(Robot.isSimulation()) return;
        if(Robot.isReal()) return;  // TODO: AGHHHHHHHHHHHHHHHhh - LAC ok 
        try {
            Thread.sleep(delayMs);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }
}
