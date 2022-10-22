package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;

public class Vision {
    private boolean active;

    // we can have our "target" color values here
    private static final int[] red = { 200, 200, 200 };
    private static final int[] green = { 200, 200, 200 };
    private static final int[] blue = { 200, 200, 200 };

    private static final double EPSILON = 0.2;

    public Vision(boolean enabled) {
        active = enabled;
    }

    public void toggleActive() {
        active = !active;
    }

    public void initCamera(OpenCvCamera camera) {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                while (true) {
                    if (!active) {
                        continue;
                    }
                }

            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    static double[] scanFrame(Bitmap frame) {
        int width = frame.getWidth();
        int height = frame.getHeight();
//        int channels = frame.getColorSpace();
        return new double[] {0};
    }

    static boolean pixelCompare(int r, int b, int g, int[] target) {
        assert target.length == 3;
        // double first will auto cast target[n] to double so we get correct result
        double rx = Math.abs(1 - (double) r / target[0]);
        double gx = Math.abs(1 - (double) g / target[1]);
        double bx = Math.abs(1 - (double) b / target[2]);
        return (rx + gx + bx) / 3 <= EPSILON;
    }
}
