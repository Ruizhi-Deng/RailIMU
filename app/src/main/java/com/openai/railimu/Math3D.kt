package com.openai.railimu

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Vec3(val x: Double, val y: Double, val z: Double) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)
    operator fun div(s: Double) = Vec3(x / s, y / s, z / s)
    fun dot(o: Vec3) = x * o.x + y * o.y + z * o.z
    fun cross(o: Vec3) = Vec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x
    )
    fun norm() = sqrt(dot(this))
    fun normalized(): Vec3 {
        val n = norm()
        return if (n > 1e-12) this / n else ZERO
    }
    fun limited(maxNorm: Double): Vec3 {
        if (maxNorm <= 0.0) return ZERO
        val n = norm()
        return if (n > maxNorm && n > 1e-12) this * (maxNorm / n) else this
    }
    companion object { val ZERO = Vec3(0.0, 0.0, 0.0) }
}

data class Quaternion(val w: Double, val x: Double, val y: Double, val z: Double) {
    operator fun times(o: Quaternion) = Quaternion(
        w * o.w - x * o.x - y * o.y - z * o.z,
        w * o.x + x * o.w + y * o.z - z * o.y,
        w * o.y - x * o.z + y * o.w + z * o.x,
        w * o.z + x * o.y - y * o.x + z * o.w
    )

    fun normalized(): Quaternion {
        val n = sqrt(w*w + x*x + y*y + z*z)
        return if (n > 1e-12) Quaternion(w/n, x/n, y/n, z/n) else IDENTITY
    }

    fun conjugate() = Quaternion(w, -x, -y, -z)

    fun rotate(v: Vec3): Vec3 {
        val tx = 2.0 * (y * v.z - z * v.y)
        val ty = 2.0 * (z * v.x - x * v.z)
        val tz = 2.0 * (x * v.y - y * v.x)
        return Vec3(
            v.x + w * tx + (y * tz - z * ty),
            v.y + w * ty + (z * tx - x * tz),
            v.z + w * tz + (x * ty - y * tx)
        )
    }

    companion object {
        val IDENTITY = Quaternion(1.0, 0.0, 0.0, 0.0)
        fun fromBodyAngularVelocity(omega: Vec3, dt: Double): Quaternion {
            val rate = omega.norm()
            if (rate < 1e-12 || dt <= 0.0) return IDENTITY
            val half = 0.5 * rate * dt
            val s = sin(half) / rate
            return Quaternion(cos(half), omega.x*s, omega.y*s, omega.z*s)
        }
    }
}
