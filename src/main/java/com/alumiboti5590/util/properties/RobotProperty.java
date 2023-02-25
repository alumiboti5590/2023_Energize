package com.alumiboti5590.util.properties;

/**
 * RobotProperty is used to fetch dynamic configuration values that change between robots. These use
 * the RobotProperties class to load in the configurations, and then values can be fetched using the
 * enumeration desired with the `get<Type>()` method matching the given type
 *
 * <p>RobotProperty.DRIVETRAIN_LEFT_LEADER_ID.getInteger()
 * RobotProperty.DRIVETRAIN_MOTOR_INVERTED.getBoolean()
 */
public enum RobotProperty {

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

  // Intake properties
  // -----------------
  INTAKE_MOTOR_ID(Integer.class),
  INTAKE_MOTOR_INVERT(Boolean.class);

  // /END OF PROPERTIES
  // ------------------

  private final Class<?> propertyType;

  private RobotProperty(Class<? extends Object> propertyType) {
    this.propertyType = propertyType;
  }

  public String getString() {
    if (this.propertyType != String.class) {}

    return this.get();
  }

  public int getInteger() {
    if (this.propertyType != Integer.class) {}

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
}
