package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

public class CameraIntrinsics {
  public double fx;
  public double fy;
  public double cx;
  public double cy;
  public double[] distCoeffs;

  public CameraIntrinsics(double fx, double fy, double cx, double cy, double[] distCoeffs) {
    this.fx = fx;
    this.fy = fy;
    this.cx = cx;
    this.cy = cy;
    this.distCoeffs = distCoeffs;
  }

  public Matrix<N3, N3> getCameraMatrix() {
    return MatBuilder.fill(
      Nat.N3(), Nat.N3(),
      fx,  0.0, cx,
      0.0, fy,  cy,
      0.0, 0.0, 1.0
    );
  }

  public Vector<N5> getDistCoeffs() {
    return VecBuilder.fill(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]);
  }
}
