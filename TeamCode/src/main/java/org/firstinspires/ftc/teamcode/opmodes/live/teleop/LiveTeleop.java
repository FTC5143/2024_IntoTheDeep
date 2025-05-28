package org.firstinspires.ftc.teamcode.opmodes.live.teleop;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.opmodes.live.LiveTeleopBase;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@TeleOp(name="Teleop Live", group="driver control")
//@Disabled
public class LiveTeleop extends LiveTeleopBase {
    boolean back1_pressed = false;
    boolean a1_pressed = false;
    boolean b1_pressed = false;
    boolean y1_pressed = false;
    boolean x1_pressed = false;

    boolean y2_pressed = false;
    boolean dpad_up2_pressed = false;
    boolean dpad_down2_pressed = false;
    boolean lbump2_pressed = false;


    boolean intake = false;
    boolean SKIP = false;

    int prepared_level = 1;
    double drive_a;


    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {
        this.getRuntime();

        robot.arm.open_claw();
        robot.arm.waiting_position();
        robot.lift.min_lift();
    }

    @Override
    public void on_loop() {

        if(gamepad1.back && !back1_pressed) {
            robot.intake.toggle_wanted_color();
            robot.led_control.toggle_alliance_color();
            gamepad1.setLedColor(robot.intake.intake_color_wanted == IntakeConst.SAMPLE_RED ? 1 : 0, 0, robot.intake.intake_color_wanted == IntakeConst.SAMPLE_BLUE ? 1 : 0, LED_DURATION_CONTINUOUS);
            back1_pressed = true;
        } else if (!gamepad1.back) {
            back1_pressed = false;
        }

        // Reach
        if (gamepad2.dpad_right) {
            robot.reach.max_reach();
            run_in(() -> { robot.intake.intake_pitch(IntakeConst.INTAKE); }, 500);
        }
        else if (gamepad2.dpad_left){
            robot.reach.min_reach();
            robot.intake.intake_pitch(IntakeConst.TRANS);
        }

        // Sweeper
        if (gamepad1.a && !a1_pressed) {
            robot.intake.sweeper_kick();
            run_in(() -> { robot.intake.sweeper_rest();} , 400);
            a1_pressed = true;
        } else {
            a1_pressed = false;
        }

        // Claw Open
        if (gamepad2.left_bumper && !lbump2_pressed) {
            if (robot.lift.level == LiftConst.SPECIMEN || robot.lift.level == LiftConst.HIGH_BAR) {
                if (robot.arm.claw_state) {
                    robot.arm.open_claw();
                } else {
                    robot.arm.close_claw();
                }
            } else {
                robot.arm.open_claw();
            }
            lbump2_pressed = true;
        } else if (!gamepad2.left_bumper) {
            lbump2_pressed = false;
        }

        if(robot.lift.level != LiftConst.INIT) {
            // Lift tweak
            robot.lift.tweak(gamepad2.right_trigger - gamepad2.left_trigger);
        }
        else {
            // Reach tweak
            robot.reach.tweak(gamepad2.right_trigger - gamepad2.left_trigger);

            // Sets intake to intake
            if(gamepad2.right_bumper) {
                robot.intake.intake_pitch(IntakeConst.INTAKE);
                intake = true;
            }
        }
        // Runs intake spinners
        if (!gamepad2.start) {
            robot.intake.intake_run(gamepad2.a ? 1 : (gamepad2.b ? -1 : 0), gamepad1, gamepad2, robot);
        }
        // Slide retraction
        if(gamepad2.back || gamepad2.x || gamepad2.a) {
            if(gamepad2.y){
                robot.lift.zero_lift();
                y2_pressed = true;
            } else {
                y2_pressed = false;
                if (gamepad2.back && robot.lift.level != LiftConst.RE_ZERO) {
                    if (robot.lift.level == LiftConst.HIGH_BASKET || robot.lift.level == LiftConst.LOW_BASKET) {
                        robot.arm.open_claw();
                        run_in(() -> {
                            robot.arm.waiting_position();
                        }, 100);
                        run_in(() -> {
                            robot.lift.min_lift();
                        }, 200);
                    } else if (robot.lift.level == LiftConst.HANG) {
                        robot.lift.elevate_to(LiftConst.LOW_BASKET);
                        robot.arm.waiting_position();
                    } else if (robot.lift.level == LiftConst.HIGH_BAR) {
                        robot.lift.elevate_to(LiftConst.SPECIMEN);
                    } else if (robot.lift.level == LiftConst.SPECIMEN) {
                        robot.arm.waiting_position();
                        run_in(() -> {
                            robot.lift.elevate_to(LiftConst.INIT);
                        }, 500);
                    } else {
                        robot.lift.min_lift();
                    }
                }
            }

            if(gamepad2.x) {
                if (prepared_level == LiftConst.LOW_BASKET || prepared_level == LiftConst.HIGH_BASKET) {
                    SKIP = true;
                    robot.arm.transfer_position();
                    run_in(() -> {
                        robot.arm.close_claw();
                    }, 300);
                    run_in(() -> {
                        robot.lift.elevate_to(prepared_level);
                    }, 400);
                    run_in(() -> {
                        robot.arm.basket_position();
                    }, 600);
                }
                /*else if (gamepad2.a && robot.reach.cur_limit_switch && robot.intake.current_color != IntakeConst.SAMPLE_NONE) {
                    if (prepared_level == LiftConst.LOW_BASKET || prepared_level == LiftConst.HIGH_BASKET) {
                        run_in(() -> {
                            robot.arm.transfer_position();
                        }, 100);
                        run_in(() -> {
                            robot.arm.close_claw();
                        }, 300);
                        run_in(() -> {
                            robot.lift.elevate_to(prepared_level);
                        }, 400);
                        run_in(() -> {
                            robot.arm.basket_position();
                        }, 600);
                    }
                }*/
                else {
                    robot.lift.elevate_to(prepared_level);
                }
            } else {
                SKIP = false;
            }

            if (robot.lift.level == LiftConst.INIT && !SKIP) {
                robot.arm.waiting_position();
            }

            if (robot.lift.level == LiftConst.SPECIMEN) {
                run_in(() -> robot.arm.specimen_position(), 500);
            }
        }

        // Lift level selection
        if (gamepad2.dpad_up && !dpad_up2_pressed) {
            prepared_level = Range.clip(prepared_level + 1, 1, robot.lift.max_level);
            dpad_up2_pressed = true;
        } else if (!gamepad2.dpad_up) {
            dpad_up2_pressed = false;
        }
        if (gamepad2.dpad_down && !dpad_down2_pressed) {
            prepared_level = Range.clip(prepared_level - 1, 1, robot.lift.max_level);
            dpad_down2_pressed = true;
        } else if (!gamepad2.dpad_down) {
            dpad_down2_pressed = false;
        }

        /// Driver 1 ///
        double TURN_MOD = 0.85;
        double speed_mod = 1;
        if (gamepad1.left_bumper) {
            speed_mod = 0.5;
        } else if (gamepad1.right_bumper) {
            speed_mod = 0.25;
        }
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double a = gamepad1.right_stick_x;
        if (gamepad1.x) {
            if (!x1_pressed) {
                drive_a = (double) robot.drive_train.lcs.a;
                a1_pressed = true;
            }
            robot.drive_train.odo_slide(x * 10, y * 10, drive_a, speed_mod);

        } else if (gamepad1.y) {
            if (!y1_pressed) {
                drive_a = MathUtil.round_quarter(robot.drive_train.lcs.a);
            }
            robot.drive_train.odo_slide(x * 10, y * 10, drive_a, speed_mod);

        } else {
            robot.drive_train.moving = false;
            a1_pressed = false;
            b1_pressed = false;
            robot.drive_train.mecanum_drive(
                    x * speed_mod,
                    y * speed_mod,
                    a * speed_mod * TURN_MOD
            );
        }
    }

    @Override
    public void on_stop() {

    }
}