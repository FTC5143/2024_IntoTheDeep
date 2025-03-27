package org.firstinspires.ftc.teamcode.components.debug;

import static org.opencv.core.CvType.CV_8UC1;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

//@Config
class OCVPhoneCameraConfig {
    public static double center_rect_offset_y = 0.5;
    public static double rects_offset_x = 0.50;
    public static double lr_rects_offset_y = 0.50;
    public static double rect_separation = 0.25;
    public static double rect_size = 0.05;
}

public class OCVPhoneCamera extends Component {

    OpenCvCamera phone_camera;

    public CapstonePipline capstone_pipeline;

    private boolean streaming;

    {
        name = "Phone Camera (OCV)";
    }

    public OCVPhoneCamera(Robot robot) {
        super(robot);
    }

    @Override
    public void registerHardware(HardwareMap hwmap) {
        int cameraMonitorViewId = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmap.appContext.getPackageName());
        phone_camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    }

    @Override
    public void startup() {
        super.startup();

        phone_camera.openCameraDevice();

        capstone_pipeline = new CapstonePipline();

        set_pipeline(capstone_pipeline);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("FRAME", phone_camera.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", phone_camera.getFps()));

        if (get_pattern() != 0) {
            telemetry.addData("L SAT", (int)capstone_pipeline.means[0].val[0]+" "
                    +(int)capstone_pipeline.means[0].val[1]+" "
                    +(int)capstone_pipeline.means[0].val[2]+" "
                    +(int)capstone_pipeline.means[0].val[3]);
            telemetry.addData("M SAT", (int)capstone_pipeline.means[1].val[0]+" "
                    +(int)capstone_pipeline.means[1].val[1]+" "
                    +(int)capstone_pipeline.means[1].val[2]+" "
                    +(int)capstone_pipeline.means[1].val[3]);
            telemetry.addData("R SAT", (int)capstone_pipeline.means[2].val[0]+" "
                    +(int)capstone_pipeline.means[2].val[1]+" "
                    +(int)capstone_pipeline.means[2].val[2]+" "
                    +(int)capstone_pipeline.means[2].val[3]);
        }

        telemetry.addData("PATTERN", capstone_pipeline.pattern);
    }

    public void set_pipeline(OpenCvPipeline pl) {
        phone_camera.setPipeline(pl);
    }

    public void start_streaming() {
        if (!streaming) {
            phone_camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            streaming = true;
        }
    }

    public int get_pattern() {
        return capstone_pipeline.pattern;
    }

    public void stop_streaming() {
        phone_camera.stopStreaming();
        streaming = false;
    }

    class CapstonePipline extends OpenCvPipeline {
        Scalar[] means = new Scalar[3];

        int pattern;

        @RequiresApi(api = Build.VERSION_CODES.N)
        @Override
        public Mat processFrame(Mat input) {

            input.convertTo(input, CV_8UC1, 1, 10);

            int[] l_rect = {
                    (int) (input.cols() * (OCVPhoneCameraConfig.rects_offset_x - OCVPhoneCameraConfig.rect_separation - OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.rows() * (OCVPhoneCameraConfig.lr_rects_offset_y - OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.cols() * (OCVPhoneCameraConfig.rects_offset_x - OCVPhoneCameraConfig.rect_separation + OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.rows() * (OCVPhoneCameraConfig.lr_rects_offset_y + OCVPhoneCameraConfig.rect_size/2))
            };

            int[] m_rect = {
                    (int) (input.cols() * (OCVPhoneCameraConfig.rects_offset_x - OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.rows() * (OCVPhoneCameraConfig.center_rect_offset_y - OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.cols() * (OCVPhoneCameraConfig.rects_offset_x + OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.rows() * (OCVPhoneCameraConfig.center_rect_offset_y + OCVPhoneCameraConfig.rect_size/2))
            };

            int[] r_rect = {
                    (int) (input.cols() * (OCVPhoneCameraConfig.rects_offset_x + OCVPhoneCameraConfig.rect_separation - OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.rows() * (OCVPhoneCameraConfig.lr_rects_offset_y - OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.cols() * (OCVPhoneCameraConfig.rects_offset_x + OCVPhoneCameraConfig.rect_separation + OCVPhoneCameraConfig.rect_size/2)),
                    (int) (input.rows() * (OCVPhoneCameraConfig.lr_rects_offset_y + OCVPhoneCameraConfig.rect_size/2))
            };

            Mat l_mat = input.submat(l_rect[1], l_rect[3], l_rect[0], l_rect[2]);
            Mat m_mat = input.submat(m_rect[1], m_rect[3], m_rect[0], m_rect[2]);
            Mat r_mat = input.submat(r_rect[1], r_rect[3], r_rect[0], r_rect[2]);

            means[0] = Core.mean(l_mat);
            means[1] = Core.mean(m_mat);
            means[2] = Core.mean(r_mat);

            int maximum_index = 0;
            for (int i = 0; i < 3; i++) {
                maximum_index = (means[i].val[0] + means[i].val[2] - means[i].val[1]) > (means[maximum_index].val[0] + means[maximum_index].val[2] - means[maximum_index].val[1]) ? i : maximum_index;
            }

            pattern = maximum_index + 1; // Pattern should not be 0 index, 0 is reserved for a "null"

            Imgproc.rectangle(input, new Point(l_rect[0], l_rect[1]), new Point(l_rect[2], l_rect[3]), new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, new Point(m_rect[0], m_rect[1]), new Point(m_rect[2], m_rect[3]), new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, new Point(r_rect[0], r_rect[1]), new Point(r_rect[2], r_rect[3]), new Scalar(0, 0, 255), 1);

            return input;
        }
    }
}