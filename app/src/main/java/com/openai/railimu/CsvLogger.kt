package com.openai.railimu

import android.content.Context
import java.io.File
import java.io.FileWriter
import java.util.Locale

object CsvLogger {
    fun write(
        context: Context,
        samples: List<RailEstimator.Sample>,
        summaries: List<RailEstimator.SegmentSummary>,
        gUsed: Double,
        p: RailEstimator.Parameters
    ): File {
        val file = File.createTempFile("railimu_v05_", ".csv", context.cacheDir)
        val summaryBySegment = summaries.associateBy { it.segmentId }
        FileWriter(file, false).use { w ->
            w.appendLine(
                "t_s,segment_id,finalized,sensor_timestamp_ns," +
                    "raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz," +
                    "q_w,q_x,q_y,q_z," +
                    "lin_ax,lin_ay,lin_az,filt_ax,filt_ay,filt_az," +
                    "driftcorr_ax,driftcorr_ay,driftcorr_az," +
                    "vx,vy,vz,px,py,pz,speed_mps,distance_m," +
                    "g_used,lpf_hz,stationary_window_s,stationary_accel_std_fulltrust,stationary_gyro_std_fulltrust," +
                    "gyro_bias_bridge_strength,tilt_closure_strength,endpoint_velocity_corr_strength,accel_bias_carryover_strength," +
                    "seg_duration_s,seg_raw_end_vx,seg_raw_end_vy,seg_raw_end_vz,seg_raw_end_speed," +
                    "seg_tilt_closure_deg,seg_start_bgx,seg_start_bgy,seg_start_bgz,seg_end_bgx,seg_end_bgy,seg_end_bgz," +
                    "seg_corrected_distance_m,seg_disp_x,seg_disp_y,seg_disp_z"
            )
            for (s in samples) {
                val m = summaryBySegment[s.segmentId]
                val q = s.qLocalFromBody
                val cols = mutableListOf<String>()
                fun add(x: Double) { cols += if (x.isFinite()) String.format(Locale.US, "%.9f", x) else "" }
                fun addInt(x: Int) { cols += x.toString() }
                add(s.elapsedSeconds); addInt(s.segmentId); addInt(if (s.finalized) 1 else 0); cols += s.timestampNs.toString()
                add(s.rawAccelBody.x); add(s.rawAccelBody.y); add(s.rawAccelBody.z)
                add(s.rawGyroBody.x); add(s.rawGyroBody.y); add(s.rawGyroBody.z)
                add(q.w); add(q.x); add(q.y); add(q.z)
                add(s.linearAccelLocal.x); add(s.linearAccelLocal.y); add(s.linearAccelLocal.z)
                add(s.filteredAccelLocal.x); add(s.filteredAccelLocal.y); add(s.filteredAccelLocal.z)
                add(s.driftCorrectedAccelLocal.x); add(s.driftCorrectedAccelLocal.y); add(s.driftCorrectedAccelLocal.z)
                add(s.velocityLocal.x); add(s.velocityLocal.y); add(s.velocityLocal.z)
                add(s.positionLocal.x); add(s.positionLocal.y); add(s.positionLocal.z)
                add(s.speedMps); add(s.distanceM)
                add(gUsed); add(p.lowPassCutoffHz); add(p.stationaryWindowS); add(p.stationaryAccelStdMaxMps2); add(p.stationaryGyroStdMaxRadS)
                add(p.gyroBiasBridgeStrength); add(p.tiltClosureStrength); add(p.endpointVelocityCorrectionStrength); add(p.accelBiasCarryoverStrength)
                if (m == null) repeat(17) { cols += "" } else {
                    add(m.durationS)
                    add(m.rawEndpointVelocity.x); add(m.rawEndpointVelocity.y); add(m.rawEndpointVelocity.z); add(m.rawEndpointSpeed)
                    add(m.tiltClosureDeg)
                    add(m.startGyroBias.x); add(m.startGyroBias.y); add(m.startGyroBias.z)
                    add(m.endGyroBias.x); add(m.endGyroBias.y); add(m.endGyroBias.z)
                    add(m.correctedDistanceM); add(m.correctedDisplacement.x); add(m.correctedDisplacement.y); add(m.correctedDisplacement.z)
                }
                w.appendLine(cols.joinToString(","))
            }
        }
        return file
    }
}
