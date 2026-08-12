package com.openai.railimu

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.tan

class RailEstimator {
    data class Parameters(
        val lowPassCutoffHz: Double = 1.0,
        val gravityCorrectionTauS: Double = 60.0,
        val gravityMagnitudeGateMps2: Double = 0.20,
        val maxGravityCorrectionDegPerSec: Double = 0.15
    ) {
        fun validate() {
            require(lowPassCutoffHz in 0.05..20.0) { "LPF cutoff must be 0.05..20 Hz" }
            require(gravityCorrectionTauS == 0.0 || gravityCorrectionTauS in 1.0..10000.0) { "Gravity correction tau must be 0 (off) or 1..10000 s" }
            require(gravityMagnitudeGateMps2 in 0.0..5.0) { "Gravity magnitude gate must be 0..5 m/s²" }
            require(maxGravityCorrectionDegPerSec in 0.0..10.0) { "Max gravity correction must be 0..10 deg/s" }
        }
    }

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
        val elapsedSeconds: Double = 0.0,
        val gravityCorrectionActive: Boolean = false,
        val gravityCorrectionOmegaBody: Vec3 = Vec3.ZERO,
        val accelMagnitudeError: Double = 0.0,
        val manualZuptCount: Int = 0
    ) {
        val speed: Double get() = velocityLocal.norm()
        val displacement: Double get() = positionLocal.norm()
    }

    private class Butterworth2LowPass {
        private var x1 = Vec3.ZERO
        private var x2 = Vec3.ZERO
        private var y1 = Vec3.ZERO
        private var y2 = Vec3.ZERO
        private var initialized = false

        fun reset(initial: Vec3 = Vec3.ZERO) {
            x1 = initial; x2 = initial; y1 = initial; y2 = initial; initialized = true
        }

        fun update(x: Vec3, cutoffHz: Double, dt: Double): Vec3 {
            if (!initialized) reset(x)
            val fc = cutoffHz.coerceAtMost(0.45 / dt)
            val k = tan(PI * fc * dt)
            val norm = 1.0 / (1.0 + sqrt(2.0) * k + k * k)
            val b0 = k * k * norm
            val b1 = 2.0 * b0
            val b2 = b0
            val a1 = 2.0 * (k * k - 1.0) * norm
            val a2 = (1.0 - sqrt(2.0) * k + k * k) * norm
            val y = x * b0 + x1 * b1 + x2 * b2 - y1 * a1 - y2 * a2
            x2 = x1; x1 = x; y2 = y1; y1 = y
            return y
        }
    }

    private var calibration: Calibration? = null
    private var parameters = Parameters()
    private var measuring = false
    private var gUsed = 9.80665
    private var gravityLocal = Vec3.ZERO
    private var q = Quaternion.IDENTITY
    private var latestGyro = Vec3.ZERO
    private var latestAccel = Vec3.ZERO
    private var lastGyroNs: Long? = null
    private var lastAccelNs: Long? = null
    private var startNs: Long? = null
    private val lowPass = Butterworth2LowPass()
    private var filtered = Vec3.ZERO
    private var previousFiltered: Vec3? = null
    private var velocity = Vec3.ZERO
    private var position = Vec3.ZERO
    private var correctionActive = false
    private var correctionOmega = Vec3.ZERO
    private var accelMagnitudeError = 0.0
    private var manualZuptCount = 0
    private var state = State()

    fun setCalibration(c: Calibration) { calibration = c }
    fun calibration(): Calibration? = calibration
    fun isMeasuring() = measuring
    fun state() = state
    fun gUsed() = gUsed
    fun parameters() = parameters

    fun start(gOverride: Double, newParameters: Parameters) {
        val c = requireNotNull(calibration) { "Calibration required" }
        require(gOverride in 5.0..15.0) { "g must be 5..15 m/s²" }
        newParameters.validate()
        parameters = newParameters
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
        correctionActive = false
        correctionOmega = Vec3.ZERO
        accelMagnitudeError = 0.0
        manualZuptCount = 0
        lowPass.reset(Vec3.ZERO)
        state = State()
        measuring = true
    }

    fun stop() { measuring = false }

    fun zeroVelocity() {
        if (!measuring) return
        velocity = Vec3.ZERO
        filtered = Vec3.ZERO
        previousFiltered = Vec3.ZERO
        lowPass.reset(Vec3.ZERO)
        manualZuptCount++
        state = state.copy(
            velocityLocal = velocity,
            filteredAccelLocal = filtered,
            manualZuptCount = manualZuptCount
        )
    }

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

        correctionActive = false
        correctionOmega = Vec3.ZERO
        val accelNorm = latestAccel.norm()
        accelMagnitudeError = abs(accelNorm - gUsed)
        if (
            parameters.gravityCorrectionTauS > 0.0 &&
            accelNorm > 1e-6 &&
            accelMagnitudeError <= parameters.gravityMagnitudeGateMps2
        ) {
            val measuredSupportDirBody = latestAccel / accelNorm
            val predictedSupportDirBody = q.conjugate().rotate(gravityLocal).normalized()
            // q maps body -> Start-local. A positive q tilt makes the predicted body-frame
            // gravity direction tilt oppositely, so measured x predicted gives the restoring sign.
            val errorBody = measuredSupportDirBody.cross(predictedSupportDirBody)
            val maxRate = parameters.maxGravityCorrectionDegPerSec * PI / 180.0
            correctionOmega = (errorBody / parameters.gravityCorrectionTauS).limited(maxRate)
            correctionActive = correctionOmega.norm() > 0.0
        }

        val omega = rawGyroBody - c.gyroBiasBody + correctionOmega
        q = (q * Quaternion.fromBodyAngularVelocity(omega, dt)).normalized()
    }

    fun onAccelerometer(timestampNs: Long, rawAccelBody: Vec3): State {
        latestAccel = rawAccelBody
        if (!measuring) return state.copy(rawAccelBody = rawAccelBody, rawGyroBody = latestGyro)
        if (startNs == null) startNs = timestampNs

        val accelLocal = q.rotate(rawAccelBody)
        val linear = accelLocal - gravityLocal
        val prevNs = lastAccelNs
        lastAccelNs = timestampNs

        if (prevNs == null) {
            lowPass.reset(linear)
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

        filtered = lowPass.update(linear, parameters.lowPassCutoffHz, dt)
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
            elapsedSeconds = elapsed,
            gravityCorrectionActive = correctionActive,
            gravityCorrectionOmegaBody = correctionOmega,
            accelMagnitudeError = accelMagnitudeError,
            manualZuptCount = manualZuptCount
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
