package com.openai.railimu

import kotlin.math.PI

class RailEstimator(private val lowPassCutoffHz: Double = 5.0) {
    data class Calibration(
        val measuredGravityVectorBody: Vec3,
        val gyroBiasBody: Vec3,
        val measuredG: Double,
        val accelSamples: Int,
        val gyroSamples: Int
    )

    data class State(
        val timestampNs: Long = 0L,
        val rawAccelBody: Vec3 = Vec3.ZERO,
        val rawGyroBody: Vec3 = Vec3.ZERO,
        val qLocalFromBody: Quaternion = Quaternion.IDENTITY,
        val accelLocalIncludingGravity: Vec3 = Vec3.ZERO,
        val linearAccelLocal: Vec3 = Vec3.ZERO,
        val filteredAccelLocal: Vec3 = Vec3.ZERO,
        val velocityLocal: Vec3 = Vec3.ZERO,
        val positionLocal: Vec3 = Vec3.ZERO,
        val elapsedSeconds: Double = 0.0
    ) {
        val speed: Double get() = velocityLocal.norm()
        val displacement: Double get() = positionLocal.norm()
    }

    private var calibration: Calibration? = null
    private var measuring = false
    private var gUsed = 9.80665
    private var gravityLocal = Vec3.ZERO
    private var q = Quaternion.IDENTITY
    private var latestGyro = Vec3.ZERO
    private var lastGyroNs: Long? = null
    private var lastAccelNs: Long? = null
    private var startNs: Long? = null
    private var filtered = Vec3.ZERO
    private var previousFiltered: Vec3? = null
    private var velocity = Vec3.ZERO
    private var position = Vec3.ZERO
    private var state = State()

    fun setCalibration(c: Calibration) { calibration = c }
    fun calibration(): Calibration? = calibration
    fun isMeasuring() = measuring
    fun state() = state
    fun gUsed() = gUsed

    fun start(gOverride: Double) {
        val c = requireNotNull(calibration) { "Calibration required" }
        require(gOverride in 5.0..15.0)
        gUsed = gOverride
        gravityLocal = c.measuredGravityVectorBody.normalized() * gUsed
        q = Quaternion.IDENTITY
        latestGyro = Vec3.ZERO
        lastGyroNs = null
        lastAccelNs = null
        startNs = null
        filtered = Vec3.ZERO
        previousFiltered = null
        velocity = Vec3.ZERO
        position = Vec3.ZERO
        state = State()
        measuring = true
    }

    fun stop() { measuring = false }

    fun onGyroscope(timestampNs: Long, rawGyroBody: Vec3) {
        latestGyro = rawGyroBody
        if (!measuring) return
        val c = calibration ?: return
        if (startNs == null) startNs = timestampNs
        val prev = lastGyroNs
        lastGyroNs = timestampNs
        if (prev == null) return
        val dt = (timestampNs - prev) * 1e-9
        if (dt !in 0.0001..0.1) return
        val omega = rawGyroBody - c.gyroBiasBody
        q = (q * Quaternion.fromBodyAngularVelocity(omega, dt)).normalized()
    }

    fun onAccelerometer(timestampNs: Long, rawAccelBody: Vec3): State {
        if (!measuring) return state.copy(rawAccelBody = rawAccelBody, rawGyroBody = latestGyro)
        if (startNs == null) startNs = timestampNs

        val accelLocal = q.rotate(rawAccelBody)
        val linear = accelLocal - gravityLocal
        val prevNs = lastAccelNs
        lastAccelNs = timestampNs

        if (prevNs == null) {
            filtered = linear
            previousFiltered = filtered
            state = makeState(timestampNs, rawAccelBody, accelLocal, linear)
            return state
        }

        val dt = (timestampNs - prevNs) * 1e-9
        if (dt !in 0.0001..0.1) {
            state = makeState(timestampNs, rawAccelBody, accelLocal, linear)
            return state
        }

        val rc = 1.0 / (2.0 * PI * lowPassCutoffHz)
        val alpha = dt / (rc + dt)
        filtered = filtered + (linear - filtered) * alpha

        val aPrev = previousFiltered ?: filtered
        val oldV = velocity
        velocity = velocity + (aPrev + filtered) * (0.5 * dt)
        position = position + (oldV + velocity) * (0.5 * dt)
        previousFiltered = filtered

        state = makeState(timestampNs, rawAccelBody, accelLocal, linear)
        return state
    }

    private fun makeState(t: Long, rawA: Vec3, accelLocal: Vec3, linear: Vec3): State {
        val elapsed = startNs?.let { (t - it) * 1e-9 } ?: 0.0
        return State(
            timestampNs = t,
            rawAccelBody = rawA,
            rawGyroBody = latestGyro,
            qLocalFromBody = q,
            accelLocalIncludingGravity = accelLocal,
            linearAccelLocal = linear,
            filteredAccelLocal = filtered,
            velocityLocal = velocity,
            positionLocal = position,
            elapsedSeconds = elapsed
        )
    }
}

class CalibrationAccumulator {
    private var aSum = Vec3.ZERO
    private var gSum = Vec3.ZERO
    private var aN = 0
    private var gN = 0

    fun addAccel(v: Vec3) { aSum += v; aN++ }
    fun addGyro(v: Vec3) { gSum += v; gN++ }

    fun result(): RailEstimator.Calibration {
        require(aN >= 20 && gN >= 20) { "Not enough sensor samples" }
        val meanA = aSum / aN.toDouble()
        val meanG = gSum / gN.toDouble()
        val g = meanA.norm()
        require(g in 5.0..15.0) { "Measured g is implausible" }
        return RailEstimator.Calibration(meanA, meanG, g, aN, gN)
    }
}
