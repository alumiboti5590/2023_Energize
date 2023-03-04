package com.alumiboti5590.util.properties;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * RobotProperty is used to fetch dynamic configuration values that change between robots. These use
 * the RobotProperties class to load in the configurations, and then values can be fetched using the
 * enumeration desired with the `get<Type>()` method matching the given type
 *
 * <p>RobotProperty.DRIVETRAIN_LEFT_LEADER_ID.getInteger()
 * RobotProperty.DRIVETRAIN_MOTOR_INVERTED.getBoolean()
 */
public enum RobotProperty {

  // ---------------------
  // Drivetrain properties
  // ---------------------
  DRIVETRAIN_LEFT_LEADER_ID(Integer.class),
  DRIVETRAIN_RIGHT_LEADER_ID(Integer.class),
  DRIVETRAIN_LEFT_FOLLOWER_ID(Integer.class),
  DRIVETRAIN_RIGHT_FOLLOWER_ID(Integer.class),

  DRIVETRAIN_ENCODER_LEFT_CHAN_A(Integer.class),
  DRIVETRAIN_ENCODER_LEFT_CHAN_B(Integer.class),
  DRIVETRAIN_ENCODER_RIGHT_CHAN_A(Integer.class),
  DRIVETRAIN_ENCODER_RIGHT_CHAN_B(Integer.class),

  DRIVETRAIN_INVERT_STEERING(Boolean.class),
  DRIVETRAIN_LEFT_LEADER_INVERT(Boolean.class),
  DRIVETRAIN_RIGHT_LEADER_INVERT(Boolean.class),
  DRIVETRAIN_LEFT_FOLLOWER_INVERT(Boolean.class),
  DRIVETRAIN_RIGHT_FOLLOWER_INVERT(Boolean.class),

  // Used to control the scaled maximum speed that the robot should have
  // under regular circumstances. This can be overriden to full by using
  // the "Turbo" button
  STANDARD_DRIVE_SPEED(Double.class),

  // Used to control the maximum speed that the robot should have when
  // the "Turbo" button is activated.
  TURBO_DRIVE_SPEED(Double.class),

  // -----------------
  // Arm properties
  // -----------------
  ARM_MOTOR_ID(Integer.class),
  ARM_MOTOR_BRUSHLESS(Boolean.class),
  ARM_MOTOR_INVERT(Integer.class),

  // -----------------
  // Intake properties
  // -----------------
  INTAKE_MOTOR_ID(Integer.class),
  INTAKE_MOTOR_INVERT(Boolean.class),

  // -------------------
  // Shoulder properties
  // -------------------
  SHOULDER_MOTOR_ID(Integer.class),
  SHOULDER_MOTOR_BRUSHLESS(Boolean.class),
  SHOULDER_MOTOR_INVERT(Integer.class);

  // ------------------
  // /END OF PROPERTIES
  // ------------------

  private final Class<?> propertyType;

  private RobotProperty(Class<? extends Object> propertyType) {
    this.propertyType = propertyType;
  }

  public String getString() {
    this.validatePropertyType(String.class);
    return this.get();
  }

  public int getInteger() {
    this.validatePropertyType(Integer.class);
    return Integer.parseInt(this.get());
  }

  public double getDouble() {
    if (this.propertyType != Double.class) {}

    return Double.parseDouble(this.get());
  }

  public boolean getBoolean() {
    if (this.propertyType != Boolean.class) {}

    return Boolean.parseBoolean(this.get());
  }

  public String getPropertyName() {
    return this.name().toLowerCase();
  }

  private String get() {
    return RobotProperties.getInstance().getProperty(this.getPropertyName());
  }

  /**
   * Dumps an error into the DriverStation if the type cast is not correct
   *
   * @param classType
   */
  private void validatePropertyType(Class<? extends Object> classType) {
    if (this.propertyType != classType) {
      String methodName = new Exception().getStackTrace()[1].getMethodName();
      String err =
          String.format(
              "Tried to call `%s.%s()` when it is a `%s`",
              this.name(), methodName, classType.getName());
      DriverStation.reportError(err, false);
      int v = 1 / 0; // Force an error without bubbling it up
    }
  }
}
