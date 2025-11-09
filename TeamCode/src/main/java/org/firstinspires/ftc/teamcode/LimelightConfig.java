package org.firstinspires.ftc.teamcode;

public final class LimelightConfig {

    // Prevent instantiation
    private LimelightConfig() { }

    // --- Hardware / Pipeline Constants ---
    public static final String LIMELIGHT_NAME       = "limelight";          // Name of Limelight in hardware map
    // Note: pipeline may differ per OpMode, so not hard-coded here

    // --- Vision Mounting / Target Geometry ---
    public static final double CAMERA_HEIGHT_INCHES  = 11.711571;           // Height of camera off floor, in inches
    public static final double CAMERA_TILT_DEGREES   = 20.7499929;          // Tilt upward angle of camera, in degrees
    public static final double TAG_HEIGHT_INCHES     = 29.5;                 // Height of target AprilTag center from floor, in inches

    // --- Search & Aim Thresholds ---
    public static final double TURN_DEADZONE_DEGREES = 1.0;                  // Error threshold (degrees) within which we consider aligned
    public static final double MAX_TURN_POWER        = 0.4;                  // Maximum turn power during alignment
    public static final double SPIN_SEARCH_POWER     = 0.3;                  // Power used for spinning to search for target

    // --- Drive constants (for mecanum) ---
    public static final double KP_TURN              = 0.02;                 // Proportional constant for turning correction

    // --- Target IDs ---
    public static final int    TARGET_APRILTAG_ID   = 20;                   // ID of the AprilTag we track

    // Add additional constants as needed…

}
