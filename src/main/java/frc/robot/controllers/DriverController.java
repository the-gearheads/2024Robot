public interface DriverController {
  public default double getTranslateXAxis() {
    return 0;
  }

  public default double getTranslateYAxis() {
    return 0;
  }

  public default double getRotateAxis() {
    return 0;
  }
}
