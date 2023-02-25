package com.alumiboti5590.util.properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Properties;
import java.util.Set;
import java.util.stream.Collectors;

/** Assists RobotProperty to load from the deploy directory properties file. */
class RobotProperties extends Properties {

  private static final String ROBOT_NAME_FILE = "/etc/robot-name";

  private static RobotProperties singleton;

  public static RobotProperties getInstance() {
    if (singleton == null) {
      singleton = new RobotProperties();
    }
    return singleton;
  }

  public RobotProperties() {
    this(robotName());
  }

  public RobotProperties(String propertyFilePrefix) {
    this.loadDeployFileContents(propertyFilePrefix);
    this.verifyRobotProperties();
  }

  public String getString(RobotProperty property) {
    return this.getProperty(property.name().toLowerCase());
  }

  public int getInteger(RobotProperty property) {
    return Integer.parseInt(this.getString(property));
  }

  public double getDouble(RobotProperty property) {
    return Double.parseDouble(this.getString(property));
  }

  public boolean getBoolean(RobotProperty property) {
    return Boolean.parseBoolean(this.getString(property));
  }

  /**
   * Load in the robot properties into the RobotProperties instance.
   *
   * @param propertyFilePrefix - The filename prefix (without .properties)
   */
  private void loadDeployFileContents(String propertyFilePrefix) {
    String deployDirPath = Filesystem.getDeployDirectory().getAbsolutePath();
    Path propertiesFilePath =
        Paths.get(deployDirPath, String.format("%s.properties", propertyFilePrefix));
    try {
      InputStream input = new FileInputStream(propertiesFilePath.toString());
      this.clear(); // Ensure no previous keys exist
      this.load(input);
    } catch (IOException ex) {
      ex.printStackTrace();
    }
  }

  /**
   * Verify that all of the keys in the RobotProperty enum exist in the configuration file. This
   * ensures that all important values have been defined for all important pieces.
   */
  private void verifyRobotProperties() {
    Set<String> requiredPropertyNames =
        Arrays.asList(RobotProperty.class.getEnumConstants()).stream()
            .map(prop -> prop.getPropertyName())
            .collect(Collectors.toSet());

    Set<String> currentPropertyNames = this.stringPropertyNames();
    if (!currentPropertyNames.containsAll(requiredPropertyNames)) {
      requiredPropertyNames.removeAll(currentPropertyNames);
      String stillMissing = requiredPropertyNames.toString();
      String error = String.format("Missing robot properties: %s", stillMissing);
      DriverStation.reportError(error, false);
      int v = 1 / 0; // Force an error without bubbling it up
    }
  }

  // Fetches the robot name from the `ROBOT_NAME_FILE`
  private static String robotName() {
    try {
      return Files.readString(Paths.get(ROBOT_NAME_FILE), StandardCharsets.UTF_8);
    } catch (IOException ex) {
      ex.printStackTrace();
      return "";
    }
  }

  /** Used primarily for testing */
  public static void UNSAFE_setSingleton(String robotName) {
    singleton = new RobotProperties(robotName);
  }
}
