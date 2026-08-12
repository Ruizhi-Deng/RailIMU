package com.openai.railimu

import android.content.Context
import java.io.File
import java.io.FileWriter
import java.util.Locale

class CsvLogger(context: Context) {
    val file: File = File.createTempFile("railimu_", ".csv", context.cacheDir)
    private val writer = FileWriter(file, false)
    private var rowsSinceFlush = 0

    init {
        writer.appendLine(
            "t_s,sensor_timestamp_ns,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz," +
                "q_w,q_x,q_y,q_z,acc_local_x,acc_local_y,acc_local_z," +
                "lin_ax,lin_ay,lin_az,filt_ax,filt_ay,filt_az," +
                "vx,vy,vz,px,py,pz,speed_mps,displacement_m,g_used," +
                "lpf_hz,gravity_tau_s,gravity_gate_mps2,max_corr_deg_s," +
                "corr_active,corr_wx,corr_wy,corr_wz,accel_mag_error,manual_zupt_count"
        )
    }

    fun append(s: RailEstimator.State, gUsed: Double, p: RailEstimator.Parameters) {
        val q = s.qLocalFromBody
        val values = listOf(
            s.elapsedSeconds, s.timestampNs.toDouble(),
            s.rawAccelBody.x, s.rawAccelBody.y, s.rawAccelBody.z,
            s.rawGyroBody.x, s.rawGyroBody.y, s.rawGyroBody.z,
            q.w, q.x, q.y, q.z,
            s.accelLocalIncludingGravity.x, s.accelLocalIncludingGravity.y, s.accelLocalIncludingGravity.z,
            s.linearAccelLocal.x, s.linearAccelLocal.y, s.linearAccelLocal.z,
            s.filteredAccelLocal.x, s.filteredAccelLocal.y, s.filteredAccelLocal.z,
            s.velocityLocal.x, s.velocityLocal.y, s.velocityLocal.z,
            s.positionLocal.x, s.positionLocal.y, s.positionLocal.z,
            s.speed, s.displacement, gUsed,
            p.lowPassCutoffHz, p.gravityCorrectionTauS, p.gravityMagnitudeGateMps2,
            p.maxGravityCorrectionDegPerSec,
            if (s.gravityCorrectionActive) 1.0 else 0.0,
            s.gravityCorrectionOmegaBody.x, s.gravityCorrectionOmegaBody.y, s.gravityCorrectionOmegaBody.z,
            s.accelMagnitudeError, s.manualZuptCount.toDouble()
        )
        writer.appendLine(values.joinToString(",") { String.format(Locale.US, "%.9f", it) })
        rowsSinceFlush++
        if (rowsSinceFlush >= 100) { writer.flush(); rowsSinceFlush = 0 }
    }

    fun close(): File { writer.flush(); writer.close(); return file }
}
