package org.firstinspires.ftc.teamcode.components.live;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Config
class LiftConfig {
    public static int LIFT_OFFSET = 0;
    public static int MAX_LEVEL = 5; // highest level the virtual robot can extend
    public static int MIN_LEVEL = 0;

    public static int MAX_EXTENSION = 72000; // highest safe extension of physical robot
    public static int MIN_LIFT_OVERSHOOT = 4000;

    public static int THRESHOLD = 200;

    public static int LINEAR_MAX_SPEED = 16000;

    public static int TWEAK_MAX_ADD = 8000;

    public static int[] LIFT_LEVELS = {
            0,      // level 0
            70240,  // high bin
            30350,  // low bin
            24240,  // specimen
            57520,  // high rung
            68200   // hang
    };

    public static double PID_P = 0.08;
    public static double PID_I = 0.0016;
    public static double PID_D = 0.0002;
}

public class Lift extends Component {


    //// MOTORS ////
    public DcMotorEx lift_f;
    public DcMotorEx lift_b;

    //// SENSORS ////
    public DigitalChannel limit_switchL;

    private PIDFController pid_control;

    public int level;
    public int max_level = LiftConfig.MAX_LEVEL;
    public int lift_target = 0;
    public int lift_offset = 0;

    public boolean cur_limit_switch = true;
    private boolean last_limit_switch = true;

    public boolean at_thresh = false;

    private double tweak = 0;
    private double tweak_cache = 0;


    private double pid_speed = 0;
    public boolean starting_move = false;

    {
        name = "Lift";
    }

    public Lift(Robot robot)
    {
        super(robot);
    }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        //// MOTORS ////
        lift_f = hwmap.get(DcMotorEx.class, "liftF");
        lift_b = hwmap.get(DcMotorEx.class, "liftB");
        lift_b.setDirection(DcMotorSimple.Direction.REVERSE);

        limit_switchL = hwmap.get(DigitalChannel.class, "vLimSwitch");

        pid_control = new PIDFController(new PIDCoefficients(LiftConfig.PID_P, LiftConfig.PID_I, LiftConfig.PID_D));
        pid_control.setOutputBounds(-1000, 1000);
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        cur_limit_switch = !limit_switchL.getState();
        int cur_position = lift_f.getCurrentPosition();

        int new_target = 0;

        if (level == LiftConst.INIT) {
            if (cur_limit_switch && !last_limit_switch) {
                lift_offset = cur_position;
            }
            if (!cur_limit_switch) {
                new_target = (lift_offset - LiftConfig.MIN_LIFT_OVERSHOOT);
            }
            starting_move = true;
        } else if (level == LiftConst.RE_ZERO) {
            if (cur_limit_switch) {
                level = LiftConst.INIT;
                lift_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                set_power(-1);
                starting_move = false;
            }
        } else {
            new_target = (lift_offset + lift_target);
            if (tweak != tweak_cache) {
                tweak_cache = tweak;
                new_target = (Range.clip(
                        lift_target + lift_offset + (int) (tweak * LiftConfig.TWEAK_MAX_ADD),
                        lift_offset,
                        LiftConfig.MAX_EXTENSION
                ));
                starting_move = true;
            }
        }

        pid_control.setTargetPosition(new_target);
        starting_move = false;

        pid_speed = pid_control.update(cur_position) / 1000;

        if (lift_target <= 0 && cur_limit_switch) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            set_power(0);
        } else {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (level != LiftConst.RE_ZERO) {
                set_power(pid_speed);
            }
        }

        last_limit_switch = cur_limit_switch;
    }

    @Override
    public void startup() {
        super.startup();

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutdown() {
        super.shutdown();

        stop();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("TURNS",TELEMETRY_DECIMAL.format(lift_f.getCurrentPosition()));
        telemetry.addData("TARGET",TELEMETRY_DECIMAL.format(lift_target));
        telemetry.addData("OFFSET", TELEMETRY_DECIMAL.format(lift_offset));
        telemetry.addData("LEVEL", level);
        telemetry.addData("TWEAK", tweak);
        telemetry.addData("LIFT LIM", !limit_switchL.getState());
        telemetry.addData("PID VEL", pid_speed);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        lift_f.setZeroPowerBehavior(mode);
        lift_b.setZeroPowerBehavior(mode);
    }

    /**
     * Sets the motor power to speed
     * @param speed new motor power
     */
    public void set_power(double speed) {
        lift_f.setPower(speed);
        lift_b.setPower(speed);
    }

    public void stop() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        set_power(0);
    }

    /**
     * Sets the target position of PID to pos
     * @param pos new target position in cycles
     */
    private void set_target_position(int pos) {
        lift_target = pos;
    }

    /**
     * set PID target to LIFT_LEVELS[target_level]
     * @param target_level int
     */
    public void elevate_to(int target_level) {
        level = Math.max(Math.min(target_level, LiftConfig.MAX_LEVEL), LiftConfig.MIN_LEVEL);
        set_target_position((LiftConfig.LIFT_LEVELS[level]) + LiftConfig.LIFT_OFFSET);
        starting_move = true;
    }

    /**
     * elevates left to LIFT_LEVELS[MIN_LEVEL]
     */
    public void min_lift() {
        elevate_to(LiftConfig.MIN_LEVEL);
    }

    public void zero_lift() {
        level = LiftConst.RE_ZERO;
    }

    /**
     * elevates lift to LIFT_LEVELS[MAX_LEVEL]
     */
    public void max_lift() {
        elevate_to(LiftConfig.MAX_LEVEL);
    }

    /**
     * Returns speed if current position of lift is outside of the threshold
     * Returns 0 if current position of lift is inside of the threshold
     * @param speed new motor power
     * @return speed if current position is outside of the threshold
     */
    public double hold_threshold(double speed){
        if (Math.abs(lift_target - lift_f.getCurrentPosition()) <= LiftConfig.THRESHOLD) {
            at_thresh = true;
            return 0;
        }
        at_thresh = false;
        return speed;
    }

    /**
     * sets PID target to target_position +
     * @param speed analog input * max speed constant
     */
    public void linear(double speed) {
        set_target_position(lift_f.getCurrentPosition() + (int) (LiftConfig.LINEAR_MAX_SPEED * speed));
        starting_move = true;
    }

    /**
     * sets PID target to target_position +
     * @param tweak analog input
     */
    public void tweak(double tweak) {
        this.tweak = tweak;
    }
}