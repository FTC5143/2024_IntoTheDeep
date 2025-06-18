package org.firstinspires.ftc.teamcode.components.live;

import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.MIN_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.SECTION;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.SECT_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.TWEAK_MAX_ADD;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

@Config
class ReachConfig {
    public static final int MAX_LENGTH      = 670;  // mm point to point
    public static final int MIN_LENGTH      = 100;  // mm point to point
    public static final int TWEAK_MAX_ADD   = 100;  // mm
    public static final int SECTION         = 6;    // number of scissor sections
    public static final int SECT_LENGTH     = 112;  // length of section-arms in mm (from end to end)\

    public static final int SERVO_RANGE     = 355; // deg

    public static final int MAX_LINEAR_SPEED = 30; // mm
}

public class Reach extends Component {
    //// SERVOS ////
    public ServoQUS reach_l;
    public ServoQUS reach_r;

    //// SENSORS ////
    public DigitalChannel limit_switchR;

    public int reach_target;
    private int reach_l_target = MIN_LENGTH;
    private int reach_r_target = MIN_LENGTH;
    private double reach_l_angle = reach_angle(MIN_LENGTH);
    private double reach_r_angle = reach_angle(MIN_LENGTH);

    public boolean cur_limit_switch = true;

    double tweak = 0;
    double tweak_cache = 0;

    {
        name = "Reach";
    }

    public Reach(Robot robot) { super(robot); }

    @Override
    public void registerHardware (HardwareMap hwmap) {
        //// SERVOS ////
        reach_l     = new ServoQUS(hwmap.get(Servo.class, "reachL"));
        reach_r     = new ServoQUS(hwmap.get(Servo.class, "reachR"));

        limit_switchR = hwmap.get(DigitalChannel.class, "rLimSwitch");
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        cur_limit_switch = !limit_switchR.getState();

        if (tweak != tweak_cache) {
            tweak_cache = tweak;
            reach_l.queue_position(
                    Math.toDegrees(reach_angle(Range.clip(
                            reach_l_target + (int) (tweak * TWEAK_MAX_ADD),
                            MIN_LENGTH,
                            MAX_LENGTH
                    ))) / ReachConfig.SERVO_RANGE
            );

            reach_r.queue_position(
                    Math.toDegrees(reach_angle(Range.clip(
                            reach_r_target + (int) (tweak * TWEAK_MAX_ADD),
                            MIN_LENGTH,
                            MAX_LENGTH
                    ))) / ReachConfig.SERVO_RANGE
            );
        }

        update();
    }

    @Override
    public void startup() {
        super.startup();

        min_reach();

        update();
    }

    public void shutdown() {
        super.shutdown();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("REACH TARGET", TELEMETRY_DECIMAL.format(reach_l_target));

        telemetry.addData("REACH ANGLE", TELEMETRY_DECIMAL.format(Math.toDegrees(reach_l_angle) / ReachConfig.SERVO_RANGE));

        telemetry.addData("REACH LIM", !limit_switchR.getState());
    }

    public void update() {
        reach_l.update();
        reach_r.update();
    }

    private double reach_angle(double dis) {
        return Math.acos(dis / (SECTION * SECT_LENGTH));
    }

    public void extend_to(int target) {
        reach_target = Range.clip(target, MIN_LENGTH, MAX_LENGTH);

        reach_l_target = reach_target;
        reach_r_target = reach_target;

        reach_l_angle = reach_angle(reach_l_target);
        reach_r_angle = reach_angle(reach_r_target);
        reach_l.queue_position((Math.toDegrees(reach_l_angle)) / ReachConfig.SERVO_RANGE);
        reach_r.queue_position((Math.toDegrees(reach_r_angle)) / ReachConfig.SERVO_RANGE);
    }

    public void linear(double speed) {
        extend_to(reach_l_target + (int) (speed * ReachConfig.MAX_LINEAR_SPEED));
    }

    public void min_reach() {
        extend_to(MIN_LENGTH);
    }

    public void max_reach() {
        extend_to(MAX_LENGTH - TWEAK_MAX_ADD);
    }

    public void tweak(double tweak) {
        this.tweak = tweak;
    }
}