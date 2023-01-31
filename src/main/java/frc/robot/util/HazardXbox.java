package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

enum Side {
  LEFT,
  RIGHT
}

public class HazardXbox extends CommandXboxController {

  private float deadzone = 0;

  public HazardXbox(int port) {
    super(port);
  }

  public HazardXbox(int port, float deadzone) {
    super(port);

    this.deadzone = deadzone;
  }

  /**
   * Returns the value of the axis after accounting for the configured deadzone
   *
   * @param v The value of the axis
   * @return The value of the axis after accounting for deadzone
   */
  private double deadzone(float deadzone, double v) {
    return Math.abs(v) > deadzone ? v : 0;
  }

  @Override
  public double getLeftX() {
    return this.deadzone(this.deadzone, super.getLeftX());
  }

  public double getLeftX(float deadzone) {
    return this.deadzone(deadzone, super.getLeftX());
  }

  @Override
  public double getLeftY() {
    return this.deadzone(this.deadzone, super.getLeftY());
  }

  public double getLeftY(float deadzone) {
    return this.deadzone(deadzone, super.getLeftY());
  }

  @Override
  public double getRightX() {
    return this.deadzone(this.deadzone, super.getRightX());
  }

  public double getRightX(float deadzone) {
    return this.deadzone(deadzone, super.getRightX());
  }

  @Override
  public double getRightY() {
    return this.deadzone(this.deadzone, super.getRightY());
  }

  public double getRightY(float deadzone) {
    return this.deadzone(deadzone, super.getRightY());
  }
}
