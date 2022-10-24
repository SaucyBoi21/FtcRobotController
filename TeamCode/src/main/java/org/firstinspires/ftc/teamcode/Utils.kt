@file:JvmName("Utils")
package org.firstinspires.ftc.teamcode


@JvmStatic
fun powerCurve(input: Double, amt: Double): Double {
    return amt * input.pow(3.0) + input * (1 - amt)
}
