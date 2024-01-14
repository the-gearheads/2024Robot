package frc.robot.util;

public class HandledSleep {
    public static void sleep(long delayMs) {
        try {
            Thread.sleep(delayMs);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }
}
