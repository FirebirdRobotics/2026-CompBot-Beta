package frc.robot.constants;

public class IntakeConstants {
    // Need to set CAN IDs
    public static final int pivotMotorCANID = 42;   // this is correct
    public static final int rollerMotorCANID = 47;  // this is correct   

    public static final double deployDistance = 0;
    public static final double midPoint= 15.7;
    public static final double framePerimeterDistance = 2; // True value is -0.1933 but this should be fine and will help protect mechanism in case of overshoot or shenanagins

}
