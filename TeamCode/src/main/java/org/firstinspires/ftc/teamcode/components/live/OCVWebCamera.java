package org.firstinspires.ftc.teamcode.components.live;

import static org.opencv.core.CvType.CV_8UC1;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Config
class OCVWebCameraConfig {

    public static double center_rect_offset_y = 0.5;
    public static double rects_offset_x = 0.50;
    public static double lr_rects_offset_y = 0.50;
    public static double rect_separation = 0.25;
    public static double rect_size = 0.05;
}

public class OCVWebCamera extends Component {

    OpenCvCamera web_camera;

    public IntoTheDeepPipline intoTheDeep_pipeline;

    private boolean streaming;

    {
        name = "Web Camera (OCV)";
    }

    public OCVWebCamera(Robot robot) { super(robot); }

    @Override
    public void registerHardware (HardwareMap hwmap) {
        int cameraMonitorViewId = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmap.appContext.getPackageName());
        web_camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);
    }

    @Override
    public void startup() {
        super.startup();

        web_camera.openCameraDevice();

        intoTheDeep_pipeline = new IntoTheDeepPipline();

        set_pipeline(intoTheDeep_pipeline);

        start_streaming();
    }

    @Override
    public void shutdown() {
        super.shutdown();

        stop_streaming();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("FRAME", web_camera.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", web_camera.getFps()));

        if (get_pattern() != 0) {
            telemetry.addData("L SAT", (int)intoTheDeep_pipeline.means[0].val[0]+" "
                    +(int)intoTheDeep_pipeline.means[0].val[1]+" "
                    +(int)intoTheDeep_pipeline.means[0].val[2]+" "
                    +(int)intoTheDeep_pipeline.means[0].val[3]);
            telemetry.addData("M SAT", (int)intoTheDeep_pipeline.means[1].val[0]+" "
                    +(int)intoTheDeep_pipeline.means[1].val[1]+" "
                    +(int)intoTheDeep_pipeline.means[1].val[2]+" "
                    +(int)intoTheDeep_pipeline.means[1].val[3]);
            telemetry.addData("R SAT", (int)intoTheDeep_pipeline.means[2].val[0]+" "
                    +(int)intoTheDeep_pipeline.means[2].val[1]+" "
                    +(int)intoTheDeep_pipeline.means[2].val[2]+" "
                    +(int)intoTheDeep_pipeline.means[2].val[3]);
        }

        telemetry.addData("PATTERN", intoTheDeep_pipeline.pattern);
    }

    public void set_pipeline(OpenCvPipeline pl) {
        web_camera.setPipeline(pl);
    }

    public void start_streaming() {
        if (!streaming) {
            web_camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            streaming = true;
        }
    }

    public int get_pattern() {
        return intoTheDeep_pipeline.pattern;
    }

    public void stop_streaming() {
        web_camera.stopStreaming();
        streaming = false;
    }

    class IntoTheDeepPipline extends OpenCvPipeline {
        Scalar[] means = new Scalar[15];

        int pattern;

        @RequiresApi(api = Build.VERSION_CODES.N)
        @Override
        public Mat processFrame(Mat input) {

            input.convertTo(input, CV_8UC1, 1, 10);

            int[][] rect_array = new int[15][4]; // [number of rectangles] [sides of a rectangle]

            rect_array = createRectangleArr(input, rect_array);

            setMean(input, rect_array);

            int maximum_index = 0;
            for (int i = 0; i < means.length; i++) {
                maximum_index = (means[i].val[0] + means[i].val[2] - means[i].val[1]) > (means[maximum_index].val[0] + means[maximum_index].val[2] - means[maximum_index].val[1]) ? i : maximum_index;
            }

            pattern = maximum_index + 1; // Pattern should not be 0 index, 0 is reserved for a "null"

            drawRectangleArr(input, rect_array);

            return input;
        }

        private int [][] createRectangleArr(Mat input, int[][] rect_array) {
            for (int i = 0; i < rect_array.length; i++) {
                rect_array[i] = new int[]{
                        (int) (input.cols() * (OCVWebCameraConfig.rects_offset_x - OCVWebCameraConfig.rect_separation - OCVWebCameraConfig.rect_size / 2)),
                        (int) (input.rows() * (OCVWebCameraConfig.lr_rects_offset_y - OCVWebCameraConfig.rect_size / 2)),
                        (int) (input.cols() * (OCVWebCameraConfig.rects_offset_x - OCVWebCameraConfig.rect_separation + OCVWebCameraConfig.rect_size / 2)),
                        (int) (input.rows() * (OCVWebCameraConfig.lr_rects_offset_y + OCVWebCameraConfig.rect_size / 2))
                };
            }

            return rect_array;
        }

        private void setMean(Mat input, int[][] rect_array) {
            for (int i = 0; i < rect_array.length; i++) {
                Mat cur_mat = input.submat(rect_array[i][1], rect_array[i][3], rect_array[i][0], rect_array[i][2]);
                means[i] = Core.mean(cur_mat);
            }
        }

        private void drawRectangleArr(Mat input, int[][] rect_array) {
            for (int i = 0; i < rect_array.length; i++) {
                Imgproc.rectangle(input, new Point(rect_array[i][0], rect_array[i][1]), new Point(rect_array[i][2], rect_array[i][3]), new Scalar(255, 0, 0), 1);
            }
        }
    }
}