package org.firstinspires.ftc.teamcode.components.live;

import static org.firstinspires.ftc.teamcode.util.MathUtil.angle_difference;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.systems.LocalCoordinateSystem;
import org.firstinspires.ftc.teamcode.util.qus.DcMotorQUS;

//@Config
class DriveTrainConfig {
    public static int GYRO_READ_INTERVAL = 2000;
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

}

public class DriveTrain extends Component {
    /**
     * Component that controls all of the drive motors, localization (odometers / gyro), and autonomous movement
     */

    //// MOTORS ////
    private DcMotorQUS drive_lf; // Left-Front drive motor
    private DcMotorQUS drive_rf; // Right-Front drive motor
    private DcMotorQUS drive_lb; // Left-Back drive motor
    private DcMotorQUS drive_rb; // Right-Back drive motor

    //// SENSORS ////
    private BHI260IMU imu; // For recalibrating odometry angle periodically

    // The cached last read IMU orientation
    private Orientation last_imu_orientation = new Orientation();

    private double imu_offset = 0; // Used to reset LCS angle correctly, doesn't work without

    // The odometry math system used for calculating position from encoder count updates from the odometers
    public LocalCoordinateSystem lcs = new LocalCoordinateSystem();

    //// VARIABLES ////

    // Drive train moves according to these. Update them, it moves
    private double drive_x = 0; // X-pos intent
    private double drive_y = 0; // Y-pos intent
    private double drive_a = 0; // Angle intent

    public double target_x = 0;
    public double target_y = 0;
    public double target_a = 0;

    private double speed;
    public boolean moving = false;
    public boolean auto = false;

    // The current coyote path the drive train is running
    public Path current_path;

    {
        name = "Drive Train";
    }

    public DriveTrain(Robot robot) {
        super(robot);
    }

    @Override
    public void registerHardware(HardwareMap hwmap) {
        super.registerHardware(hwmap);

        //// MOTORS ////
        drive_lf = new DcMotorQUS(hwmap.get(DcMotorEx.class, "drive_lf"));
        drive_rf = new DcMotorQUS(hwmap.get(DcMotorEx.class, "drive_rf"));
        drive_lb = new DcMotorQUS(hwmap.get(DcMotorEx.class, "drive_lb"));
        drive_rb = new DcMotorQUS(hwmap.get(DcMotorEx.class, "drive_rb"));

        //// SENSORS ////
        imu = hwmap.get(BHI260IMU.class, "imu"); // Expansion Hub
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        // Updating the localizer with the new odometer encoder counts
        lcs.update(
                drive_lf.motor.getCurrentPosition(),
                drive_rf.motor.getCurrentPosition(),
                drive_lb.motor.getCurrentPosition());

        // Allows the robot to move during auto
        if (moving && robot.opmode.opModeIsActive()) {
            double a = -target_a;

            double distance = Math.hypot(target_x - lcs.x, target_y - lcs.y);
            double drive_angle = Math.atan2(target_y - lcs.y, target_x - lcs.x);

            double mvmt_x = -cos(drive_angle - lcs.a) * (Range.clip(distance, 0, (5 * speed)) / (5 * speed)) * speed;
            double mvmt_y = Math.sin(drive_angle - lcs.a) * (Range.clip(distance, 0, (5 * speed)) / (5 * speed)) * speed;
            double mvmt_a = -Range.clip((angle_difference(lcs.a, a)) * 3, -1, 1) * speed;

            if (distance < 1 && drive_angle < 0.02) {
                moving = false;

                /*if (auto) {
                    read_from_imu();
                }*/
            } else {
                mecanum_drive(mvmt_x, mvmt_y, mvmt_a);
            }
        }

        // Finding new motors powers from the drive variables
        double[] motor_powers = mecanum_math(drive_x, drive_y, drive_a);
        // Set one motor power per cycle. We do this to maintain a good odometry update speed
        // We should be doing a full drive train update at about 40hz with this configuration, which is more than enough
        drive_lf.queue_power(motor_powers[0]);
        drive_rf.queue_power(motor_powers[1]);
        drive_lb.queue_power(motor_powers[2]);
        drive_rb.queue_power(motor_powers[3]);

        if (robot.cycle % 4 == 0) {
            drive_lf.update();
        } else if (robot.cycle % 4 == 1) {
            drive_rf.update();
        } else if (robot.cycle % 4 == 2) {
            drive_lb.update();
        } else if (robot.cycle % 4 == 3) {
            drive_rb.update();
        }

        // Periodically read from the IMU in order to realign the angle to counteract drift
        // IMU angle is generally more accurate than odometry angle near the end of the match
        if (robot.cycle % DriveTrainConfig.GYRO_READ_INTERVAL == 0 /* && !auto*/) {
            read_from_imu();
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("LE TURNS", drive_lf.motor.getCurrentPosition());
        telemetry.addData("RE TURNS", drive_rf.motor.getCurrentPosition());
        telemetry.addData("CE TURNS", drive_lb.motor.getCurrentPosition());

        telemetry.addData("X", lcs.x);
        telemetry.addData("Y", lcs.y);
        telemetry.addData("A", lcs.a);

        telemetry.addData("target X", target_x);
        telemetry.addData("target Y", target_y);
        telemetry.addData("target A", target_a);

        telemetry.addData("PID", drive_lf.motor.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        telemetry.addData("IMU", last_imu_orientation.firstAngle /*+" "+last_imu_orientation.secondAngle+" "+last_imu_orientation.thirdAngle*/);

        if (current_path != null) {
            Pose cfp = current_path.getFollowPose();
            telemetry.addData("PP FP", cfp.x+" "+cfp.y+" "+cfp.a);
        }
    }

    @Override
    public void startup() {
        super.startup();

        // IMU setup and parameters
        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(DriveTrainConfig.LOGO_DIRECTION, DriveTrainConfig.USB_DIRECTION);
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
        imu.resetYaw();

        // Set all the zero power behaviors to brake on startup, to prevent slippage as much as possible
        setZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the left motors, because they have a different orientation on the robot
        drive_rf.motor.setDirection(DcMotor.Direction.REVERSE);
        drive_rb.motor.setDirection(DcMotor.Direction.REVERSE);
        
        set_mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We run without encoder because we do not have motor encoders, we have odometry instead
        set_mode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void shutdown() {
        super.shutdown();

        stop();
    }


    //// Class Methods ////


    // Quick Use Methods //

    /**
     * Read the angular orientation from the IMU, takes about 6ms
     */
    public void read_from_imu() {
        last_imu_orientation = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        lcs.a = last_imu_orientation.firstAngle + imu_offset;
    }

    /**
     * Set the mode of all drive motors in bulk
     * @param mode new DcMotor.RunMode
     */
    private void set_mode(DcMotor.RunMode mode) {
        drive_lf.motor.setMode(mode);
        drive_rf.motor.setMode(mode);
        drive_lb.motor.setMode(mode);
        drive_rb.motor.setMode(mode);
    }

    /**
     * Set the zero power mode of all drive motors in bulk
     * @param mode new DcMotor.ZeroPowerBehavior
     */
    private void setZeroPower(DcMotor.ZeroPowerBehavior mode) {
        drive_lf.motor.setZeroPowerBehavior(mode);
        drive_rf.motor.setZeroPowerBehavior(mode);
        drive_lb.motor.setZeroPowerBehavior(mode);
        drive_rb.motor.setZeroPowerBehavior(mode);
    }


    // Drive Train TeleOp Methods //

    /**
     * Public setter for the drive variables, used for teleop drive
     * Can basically plug controller joystick inputs directly into it
     */
    public void mecanum_drive(double x, double y, double a) {
        drive_x = x;
        drive_y = y;
        drive_a = a;
    }


    // Drive Train Math Methods //

    /**
     * Return the motor powers needed to move in the given travel vector. Should give optimal speeds (not sqrt(2)/2 for 45% angles)
     * @param x analog x to move
     * @param y analog y to move
     * @param a analog angle to move
     * @return double[] list of  new motor powers
     */
    private double[] mecanum_math(double x, double y, double a) {

        double[] power = new double[]{
            - x + y - a,
            + x + y + a,
            + x + y - a,
            - x + y + a
        };

        double max = Math.max(Math.max(Math.abs(power[0]),Math.abs(power[1])),Math.max(Math.abs(power[2]),Math.abs(power[3])));

        if (max > 1) {
            power[0] /= max;
            power[1] /= max;
            power[2] /= max;
            power[3] /= max;
        }

        return power;
    }

    /**
     * Reset odometry to an arbitrary pose
     * @param x new x
     * @param y new y
     * @param a new angle
     */
    public void odo_reset(double x, double y, double a) {
        this.lcs.x = x;
        this.lcs.y = y;
        this.lcs.a = a;
        this.imu.resetYaw();
        this.imu_offset = a;
    }

    /**
     * Reset odometry to an arbitrary pose
     * @param pose new pose
     */
    public void odo_reset(@NonNull Pose pose) {
        odo_reset(pose.x, pose.y, pose.a);
    }

    /**
     * Sets current target for drive train
     * @param x target x
     * @param y target y
     * @param a target angle
     */
    private void target(double x, double y, double a) {
        target_x = x;
        target_y = y;
        target_a = -a;
    }

    /**
     * Stop all motors and reset all drive variables
     */
    public void stop() {
        mecanum_drive(0, 0, 0);

        setZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_lf.motor.setPower(0);
        drive_rf.motor.setPower(0);
        drive_lb.motor.setPower(0);
        drive_rb.motor.setPower(0);

        moving = false;
    }


    // Movement Methods //

    /**
     * Set drive variables to drive to a pose
     * - Note: Uses a local loop to update, will stop at target pose
     * @param x target x
     * @param y target x
     * @param a target a
     * @param speed robot drive speed
     */
    public void odo_move(double x, double y, double a, double speed) {
        odo_move(x, y, a, speed, 0);
    }

    /**
     * Set drive variables to drive to a pose until timeout
     * - Note: Uses a local loop to update, will stop at target pose
     * @param x target x
     * @param y target x
     * @param a target a
     * @param speed robot drive speed
     * @param timeout amount of time allowed to make the move
     */
    public void odo_move(double x, double y, double a, double speed, double timeout) {
        odo_move(x, y, a, speed, 1, 0.02, timeout, 0);
    }

    /**
     * Set drive variables to drive to a pose
     * - Note: Uses a local loop to update, will stop at target pose
     * @param pose target pose
     * @param speed robot drive speed
     */
    public void odo_move(Pose pose, double speed) {
        odo_move(pose, speed, 0);
    }

    /**
     * Set drive variables to drive to a pose until timeout
     * - Note: Uses a local loop to update, will stop at target pose
     * @param pose target pose
     * @param speed robot drive speed
     * @param timeout amount of time allowed to make the move
     */
    public void odo_move(Pose pose, double speed, double timeout) {
        odo_move(pose.x, pose.y, pose.a, speed, 1, 0.02, timeout, 0);
    }

    /**
     * Set drive variables to drive to a pose until timeout
     * - Note: Uses a local loop to update, will stop at target pose
     * @param x target x
     * @param y target x
     * @param a target a
     * @param speed robot drive speed
     * @param pos_acc target threshold for x & y
     * @param angle_acc target threshold for angle
     */
    public void odo_move(double x, double y, double a, double speed, double pos_acc, double angle_acc) {
        odo_move(x, y, a, speed, pos_acc, angle_acc, 0, 0);
    }

    /**
     * Set drive variables to drive to a pose until timeout
     * - Note: Uses a local loop to update, will stop at target pose
     * @param x target x
     * @param y target x
     * @param a target a
     * @param speed robot drive speed
     * @param pos_acc target threshold for x & y
     * @param angle_acc target threshold for angle
     * @param timeout amount of time allowed to make the move
     */
    public void odo_move(double x, double y, double a, double speed, double pos_acc, double angle_acc, double timeout) {
        odo_move(x, y, a, speed, pos_acc, angle_acc, timeout, 0);
    }

    /**
     * Set drive variables to drive to a pose until timeout
     * - Note: Uses a local loop to update, will stop at target pose
     * @param x target x
     * @param y target x
     * @param a target a
     * @param speed robot drive speed
     * @param pos_acc target threshold for x & y
     * @param angle_acc target threshold for angle
     * @param timeout amount of time allowed to make the move
     * @param time_at_target time to wait once at target
     */
    public void odo_move(double x, double y, double a, double speed, double pos_acc, double angle_acc, double timeout, double time_at_target) {
        target(x, y, a);
        a = -a;

        double original_distance = Math.hypot(x - lcs.x, y - lcs.y);
        double original_distance_a = Math.abs(a - lcs.a);

        double start_time = robot.opmode.getRuntime();
        double time_at_goal = 0;

        if (original_distance > 0 || original_distance_a > 0) {
            while (robot.opmode.opModeIsActive()) {
                double distance = Math.hypot(x - lcs.x, y - lcs.y);
                double distance_a = Math.abs(a - lcs.a);

                double drive_angle = Math.atan2(y-lcs.y, x-lcs.x);
                double mvmt_x = -cos(drive_angle - lcs.a) * ((Range.clip(distance, 0, (5*speed)))/(5*speed)) * speed;
                double mvmt_y = Math.sin(drive_angle - lcs.a) * ((Range.clip(distance, 0, (5*speed)))/(5*speed)) * speed;
                double mvmt_a = -Range.clip((angle_difference(lcs.a, a))*3, -1, 1) * speed;

                mecanum_drive(mvmt_x, mvmt_y, mvmt_a);

                if ((distance < pos_acc && distance_a < angle_acc) || (timeout > 0 && (robot.opmode.getRuntime() - start_time) > timeout)) {
                    if (robot.opmode.getRuntime()-time_at_goal >= time_at_target) {
                        stop();
                        break;
                    }
                } else {
                    time_at_goal = robot.opmode.getRuntime();
                }
            }
        }
    }


    /**
     * Set drive variables to drive to a pose
     * - Note: Uses the main loop to update, will stop at target pose
     * @param pose target pose
     * @param speed robot drive speed
     */
    public void odo_drive(@NonNull Pose pose, double speed) {
        odo_drive(pose.x, pose.y, pose.a, speed);
    }

    /**
     * Set drive variables to drive to a pose
     * - Note: Uses the main loop to update, will stop at target pose
     * @param x target x
     * @param y target y
     * @param a target angle
     * @param speed robot drive speed
     */
    public void odo_drive(double x, double y, double a, double speed) {
        target(x, y, a);
        this.speed = speed;
        this.moving = true;
    }

    /**
     * Set drive variables to slide
     * @param x
     * @param y
     * @param a
     * @param speed speed to slide
     */
    public void odo_slide(double x, double y, double a, double speed) {
        x = -(x * cos(a) - y * sin(a)); // cos(a) * Math.hypot(x, y);
        y = (y * cos(a) + x * sin(a));  // sin(a) * Math.hypot(x, y);
        a = -a;
        odo_drive(lcs.x + x, lcs.y + y, a, speed);
    }

    /**
     * Set drive variables to drive in a direction
     * @param direction direction to creep in radians
     * @param dist distance to travel
     * @param a robot heading (negative if no heading desired)
     * @param speed speed to creep
     */
    public void odo_creep(double direction, double dist, double a, double speed) {
        odo_creep_wiggle(direction, dist, a, 0, speed);
    }

    /**
     * Wiggle the robot while moving
     * @param wiggle wiggle amount (-0.2 - 0.2)
     */
    public void odo_wiggle(double wiggle) {
        target_a += wiggle;
        moving = true;
    }

    /**
     * Set drive variables to creep and wiggle
     * @param direction direction to creep in radians
     * @param dist distance to travel
     * @param a robot heading (negative if no heading desired)
     * @param wiggle wiggle amount (-0.2 - 0.2)
     * @param speed speed to creep
     */
    public  void odo_creep_wiggle(double direction, double dist, double a, double wiggle, double speed) {
        Pose start = lcs.get_pose();
        double x_creep = cos(direction + AutoConst.gridMod) * dist;
        double y_creep = sin(direction + AutoConst.gridMod) * dist;
        double angle = (a < 0 ? direction : a) + wiggle;

        odo_drive(start.x + x_creep, start.y + y_creep, angle, speed);
    }

    /**
     * Set drive variables to drive towards a pose
     * - Note: This will just drive towards pose, it wont stop once at pose
     * @param pose target pose
     * @param speed robot drive speed
     */
    public void odo_move_towards(@NonNull Pose pose, double speed) {
        odo_move_towards(pose.x, pose.y, pose.a, speed);
    }

    /**
     * Set drive variables to drive towards a pose
     * - Note: This will just drive towards pose, it wont stop once at pose
     * @param x target x
     * @param y target y
     * @param a target angle
     * @param speed robot drive speed
     */
    public void odo_move_towards(double x, double y, double a, double speed) {
        target(x, y, a);

        double distance = Math.hypot(x - lcs.x, y - lcs.y);

        double drive_angle = Math.atan2(y-lcs.y, x-lcs.x);
        double mvmt_x = -cos(drive_angle - lcs.a) * ((Range.clip(distance, 0, (5*speed)))/(5*speed)) * speed;
        double mvmt_y = Math.sin(drive_angle - lcs.a) * ((Range.clip(distance, 0, (5*speed)))/(5*speed)) * speed;
        double mvmt_a = -Range.clip((angle_difference(lcs.a, a))*3, -1, 1) * speed;

        mecanum_drive(mvmt_x, mvmt_y, mvmt_a);
    }


    /**
     * Follow a coyote curve path with odometry
     * @param path path to follow
     */
    public void follow_curve_path(Path path) {
        // Update our current path, for telemetry
        this.current_path = path;

        robot.opmode.resetRuntime();

        double time_at_goal = 0;

        while (robot.opmode.opModeIsActive()) {
            
            // Get our lookahead point
            path.update(lcs.get_pose());
            Pose lookahead_pose = path.getFollowPose();

            // Get the distance to our lookahead point
            double distance = Math.hypot(lookahead_pose.x-lcs.x, lookahead_pose.y-lcs.y);

            double translational_speed;
            // Find our drive speed based on distance
            if (distance < current_path.getFollowCircle().radius) {
                translational_speed = Range.clip((distance / (8 * path.getSpeed())), 0, 1) * path.getSpeed();
            } else {
                translational_speed = path.getSpeed();
            }

            // Find our turn speed based on angle difference
            double turn_speed = Range.clip(Math.abs((angle_difference(lcs.a, lookahead_pose.a -(Math.PI/2))) * 2), 0, 1) * path.getSpeed();

            // Drive towards the lookahead point
            drive_to_pose(lookahead_pose, translational_speed, turn_speed);

            if (path.isComplete() || (path.getTimeout() > 0 && robot.opmode.getRuntime() > path.getTimeout())) {
                if (robot.opmode.getRuntime()-time_at_goal >= 0 /* TODO: make this configurable */) {
                    stop();
                    break;
                }
            } else {
                time_at_goal = robot.opmode.getRuntime();
            }
        }
    }

    /**
     * Sets drive variables to drive towards a pose
     * @param pose target pose
     * @param drive_speed robot drive speed
     * @param turn_speed robot turn speed
     */
    public void drive_to_pose(@NonNull Pose pose, double drive_speed, double turn_speed) {

        // Find the angle to the pose
        double drive_angle = Math.atan2(pose.y-lcs.y, pose.x-lcs.x);

        // Find movement vector to drive towards that point
        double mvmt_x = cos(drive_angle - lcs.a) * drive_speed;
        double mvmt_y = -Math.sin(drive_angle - lcs.a) * drive_speed;
        // Find angle speed to turn towards the desired angle
        double mvmt_a = -((angle_difference(lcs.a, pose.a -(Math.PI/2) /* robot treats forward as 0deg*/) > 0 ? 1.0 : -1.0) * turn_speed);

        // Update actual motor powers with our movement vector
        mecanum_drive(mvmt_x, mvmt_y, mvmt_a);
    }
}
