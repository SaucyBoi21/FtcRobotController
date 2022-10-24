package org.firstinspires.ftc.teamcode

import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import android.graphics.Bitmap
import org.firstinspires.ftc.teamcode.Vision

class Vision(private var active: Boolean) {
    fun toggleActive() {
        active = !active
    }

    fun initCamera(camera: OpenCvCamera) {
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                while (true) {
                    if (!active) {
                        continue
                    }
                }
            }

            override fun onError(errorCode: Int) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        })
    }

    companion object {
        // we can have our "target" color values here
        private val red = intArrayOf(200, 200, 200)
        private val green = intArrayOf(200, 200, 200)
        private val blue = intArrayOf(200, 200, 200)
        private const val EPSILON = 0.2
        fun scanFrame(frame: Bitmap): DoubleAray {
            val width = frame.width
            val height = frame.height
            //        int channels = frame.getColorSpace();
            return doubleArrayOf(0.0)
        }

        fun pixelCompare(r: Int, b: Int, g: Int, target: IntArray): Boolean {
            assert(target.size == 3)
            // double first will auto cast target[n] to double so we get correct result
            val rx = Math.abs(1 - r.toDouble() / target[0])
            val gx = Math.abs(1 - g.toDouble() / target[1])
            val bx = Math.abs(1 - b.toDouble() / target[2])
            return (rx + gx + bx) / 3 <= EPSILON
        }
    }
}