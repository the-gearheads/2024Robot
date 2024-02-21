package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class DisablePhotonVersionCheck {
  // please don't ever do anything i am about to do
  private static void disablePhotonVersionCheck_unchecked() throws NoSuchFieldException, IllegalAccessException{
    var field = PhotonCamera.class.getDeclaredField("VERSION_CHECK_ENABLED");
    field.setAccessible(true);
    field.set(null, false);
  }

  public static void disablePhotonVersionCheck() {
    try {
      disablePhotonVersionCheck_unchecked();
    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
    }
  }
}
