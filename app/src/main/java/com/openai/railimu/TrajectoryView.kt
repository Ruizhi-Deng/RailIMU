package com.openai.railimu

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.view.View
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.log10
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

class TrajectoryView(context: Context) : View(context) {
    enum class Projection { HORIZONTAL, XY, XZ, YZ }

    private data class P2(val x: Double, val y: Double, val finalized: Boolean, val segmentId: Int)

    var projection: Projection = Projection.HORIZONTAL
        set(value) {
            field = value
            rebuildProjected()
        }

    private var source: List<RailEstimator.Sample> = emptyList()
    private var gravity = Vec3(0.0, 0.0, 1.0)
    private var points: List<P2> = emptyList()

    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(226, 230, 235)
        strokeWidth = 1f
    }
    private val axisPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(175, 181, 188)
        strokeWidth = 1.5f
    }
    private val finalizedPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(20, 105, 210)
        style = Paint.Style.STROKE
        strokeWidth = 5f
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }
    private val provisionalPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(230, 135, 35)
        style = Paint.Style.STROKE
        strokeWidth = 4f
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
        pathEffect = DashPathEffect(floatArrayOf(14f, 10f), 0f)
    }
    private val stationPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(20, 105, 210)
        style = Paint.Style.FILL
    }
    private val startPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(35, 155, 80)
        style = Paint.Style.FILL
    }
    private val currentPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(230, 80, 45)
        style = Paint.Style.FILL
    }
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(70, 76, 84)
        textSize = 30f
    }

    fun setData(samples: List<RailEstimator.Sample>, gravityUnitLocal: Vec3) {
        source = downsample(samples, 3500)
        gravity = if (gravityUnitLocal.norm() > 1e-9) gravityUnitLocal.normalized() else Vec3(0.0, 0.0, 1.0)
        rebuildProjected()
    }

    private fun downsample(samples: List<RailEstimator.Sample>, maxPoints: Int): List<RailEstimator.Sample> {
        if (samples.size <= maxPoints) return samples
        val stride = ceil(samples.size.toDouble() / maxPoints.toDouble()).toInt().coerceAtLeast(1)
        val out = ArrayList<RailEstimator.Sample>(maxPoints + 64)
        var lastSegment = Int.MIN_VALUE
        for (i in samples.indices) {
            val s = samples[i]
            val boundary = s.segmentId != lastSegment || i == samples.lastIndex ||
                (i + 1 < samples.size && samples[i + 1].segmentId != s.segmentId)
            if (i % stride == 0 || boundary) out.add(s)
            lastSegment = s.segmentId
        }
        return out
    }

    private fun rebuildProjected() {
        if (source.isEmpty()) {
            points = emptyList()
            invalidate()
            return
        }
        val basis = when (projection) {
            Projection.HORIZONTAL -> {
                val helper = if (abs(gravity.x) < 0.85) Vec3(1.0, 0.0, 0.0) else Vec3(0.0, 1.0, 0.0)
                val e1 = (helper - gravity * helper.dot(gravity)).normalized()
                val e2 = gravity.cross(e1).normalized()
                e1 to e2
            }
            Projection.XY -> Vec3(1.0, 0.0, 0.0) to Vec3(0.0, 1.0, 0.0)
            Projection.XZ -> Vec3(1.0, 0.0, 0.0) to Vec3(0.0, 0.0, 1.0)
            Projection.YZ -> Vec3(0.0, 1.0, 0.0) to Vec3(0.0, 0.0, 1.0)
        }
        val e1 = basis.first
        val e2 = basis.second
        points = source.map { P2(it.positionLocal.dot(e1), it.positionLocal.dot(e2), it.finalized, it.segmentId) }
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawColor(Color.rgb(250, 251, 252))
        if (points.size < 2) {
            canvas.drawText("No route yet", 36f, 60f, textPaint)
            canvas.drawText("Start recording to see the trajectory.", 36f, 102f, textPaint)
            return
        }

        var minX = points.minOf { it.x }
        var maxX = points.maxOf { it.x }
        var minY = points.minOf { it.y }
        var maxY = points.maxOf { it.y }
        if (maxX - minX < 1.0) { minX -= 0.5; maxX += 0.5 }
        if (maxY - minY < 1.0) { minY -= 0.5; maxY += 0.5 }

        val pad = 56f
        val drawableW = max(1f, width - 2f * pad)
        val drawableH = max(1f, height - 2f * pad)
        val spanX = maxX - minX
        val spanY = maxY - minY
        val scale = min(drawableW / spanX.toFloat(), drawableH / spanY.toFloat())
        val usedW = spanX.toFloat() * scale
        val usedH = spanY.toFloat() * scale
        val left = (width - usedW) * 0.5f
        val top = (height - usedH) * 0.5f

        fun sx(x: Double): Float = left + ((x - minX) * scale).toFloat()
        fun sy(y: Double): Float = top + usedH - ((y - minY) * scale).toFloat()

        val majorSpan = max(spanX, spanY)
        val grid = niceGrid(majorSpan / 5.0)
        val gx0 = floor(minX / grid).toInt()
        val gx1 = ceil(maxX / grid).toInt()
        for (i in gx0..gx1) {
            val x = i * grid
            val px = sx(x)
            canvas.drawLine(px, top, px, top + usedH, if (abs(x) < grid * 0.25) axisPaint else gridPaint)
        }
        val gy0 = floor(minY / grid).toInt()
        val gy1 = ceil(maxY / grid).toInt()
        for (i in gy0..gy1) {
            val y = i * grid
            val py = sy(y)
            canvas.drawLine(left, py, left + usedW, py, if (abs(y) < grid * 0.25) axisPaint else gridPaint)
        }

        drawRoute(canvas, true, ::sx, ::sy)
        drawRoute(canvas, false, ::sx, ::sy)

        canvas.drawCircle(sx(points.first().x), sy(points.first().y), 10f, startPaint)
        for (i in points.indices) {
            val p = points[i]
            val endOfSegment = p.finalized && (i == points.lastIndex || points[i + 1].segmentId != p.segmentId)
            if (endOfSegment) canvas.drawCircle(sx(p.x), sy(p.y), 9f, stationPaint)
        }
        canvas.drawCircle(sx(points.last().x), sy(points.last().y), 8f, currentPaint)

        val barWorld = niceGrid(majorSpan / 4.0)
        val barPx = (barWorld * scale).toFloat().coerceAtMost(drawableW * 0.45f)
        val bx = pad
        val by = height - 28f
        canvas.drawLine(bx, by, bx + barPx, by, axisPaint)
        canvas.drawLine(bx, by - 8f, bx, by + 8f, axisPaint)
        canvas.drawLine(bx + barPx, by - 8f, bx + barPx, by + 8f, axisPaint)
        canvas.drawText(formatDistance(barWorld), bx, by - 14f, textPaint)
    }

    private fun drawRoute(canvas: Canvas, finalized: Boolean, sx: (Double) -> Float, sy: (Double) -> Float) {
        val paint = if (finalized) finalizedPaint else provisionalPaint
        var path = Path()
        var active = false
        var previousSegment = Int.MIN_VALUE
        for (p in points) {
            if (p.finalized != finalized) {
                if (active) canvas.drawPath(path, paint)
                path = Path()
                active = false
                previousSegment = Int.MIN_VALUE
                continue
            }
            if (!active || p.segmentId != previousSegment) {
                if (active) canvas.drawPath(path, paint)
                path = Path()
                path.moveTo(sx(p.x), sy(p.y))
                active = true
            } else {
                path.lineTo(sx(p.x), sy(p.y))
            }
            previousSegment = p.segmentId
        }
        if (active) canvas.drawPath(path, paint)
    }

    private fun niceGrid(raw: Double): Double {
        if (!raw.isFinite() || raw <= 0.0) return 1.0
        val base = 10.0.pow(floor(log10(raw)))
        val n = raw / base
        val m = when {
            n <= 1.0 -> 1.0
            n <= 2.0 -> 2.0
            n <= 5.0 -> 5.0
            else -> 10.0
        }
        return m * base
    }

    private fun formatDistance(m: Double): String {
        return if (m >= 1000.0) String.format("%.1f km", m / 1000.0) else String.format("%.0f m", m)
    }
}
