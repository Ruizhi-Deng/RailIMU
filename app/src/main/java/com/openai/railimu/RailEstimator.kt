package com.openai.railimu

import kotlin.math.*

class RailEstimator {
    data class Parameters(
        val lowPassCutoffHz: Double = 1.0,
        val stationaryWindowS: Double = 2.0,
        val stationaryAccelStdMaxMps2: Double = 0.08,
        val stationaryGyroStdMaxRadS: Double = 0.006,
        val gyroBiasBridgeStrength: Double = 1.0,
        val tiltClosureStrength: Double = 1.0,
        val endpointVelocityCorrectionStrength: Double = 1.0
    ) {
        fun validate() {
            require(lowPassCutoffHz in 0.05..20.0)
            require(stationaryWindowS in 0.5..10.0)
            require(stationaryAccelStdMaxMps2 in 0.001..2.0)
            require(stationaryGyroStdMaxRadS in 0.0001..0.2)
            require(gyroBiasBridgeStrength in 0.0..1.5)
            require(tiltClosureStrength in 0.0..1.0)
            require(endpointVelocityCorrectionStrength in 0.0..1.0)
        }
    }

    data class Calibration(
        val measuredGravityVectorBody: Vec3,
        val gyroBiasBody: Vec3,
        val measuredG: Double,
        val accelSamples: Int,
        val gyroSamples: Int,
        val accelStdMps2: Double,
        val gyroStdRadS: Double
    )

    data class Sample(
        val segmentId: Int,
        val finalized: Boolean,
        val timestampNs: Long,
        val elapsedSeconds: Double,
        val rawAccelBody: Vec3,
        val rawGyroBody: Vec3,
        val qLocalFromBody: Quaternion,
        val linearAccelLocal: Vec3,
        val filteredAccelLocal: Vec3,
        val driftCorrectedAccelLocal: Vec3,
        val velocityLocal: Vec3,
        val positionLocal: Vec3,
        val speedMps: Double,
        val distanceM: Double
    )

    data class SegmentSummary(
        val segmentId: Int,
        val durationS: Double,
        val rawEndpointVelocity: Vec3,
        val rawEndpointSpeed: Double,
        val tiltClosureDeg: Double,
        val startGyroBias: Vec3,
        val endGyroBias: Vec3,
        val correctedDistanceM: Double,
        val correctedDisplacement: Vec3,
        val accelSamples: Int,
        val gyroSamples: Int
    )

    data class State(
        val measuring: Boolean=false,
        val segmentId: Int=0,
        val elapsedSeconds: Double=0.0,
        val rawAccelBody: Vec3=Vec3.ZERO,
        val rawGyroBody: Vec3=Vec3.ZERO,
        val filteredAccelLocal: Vec3=Vec3.ZERO,
        val velocityLocal: Vec3=Vec3.ZERO,
        val positionLocal: Vec3=Vec3.ZERO,
        val speedMps: Double=0.0,
        val correctedTotalDistanceM: Double=0.0,
        val finalizedSegments: Int=0,
        val currentSegmentFinalized: Boolean=false,
        val lastSegmentSummary: SegmentSummary?=null
    )

    private data class TimedVec3(val t: Long, val v: Vec3)
    private data class AccelEvent(val t: Long, val a: Vec3, val gyroSnapshot: Vec3)

    private class Butterworth2LowPass {
        private var x1=Vec3.ZERO; private var x2=Vec3.ZERO; private var y1=Vec3.ZERO; private var y2=Vec3.ZERO; private var initialized=false
        fun reset(initial: Vec3=Vec3.ZERO){x1=initial;x2=initial;y1=initial;y2=initial;initialized=true}
        fun update(x: Vec3, fc0: Double, dt: Double): Vec3 {
            if(!initialized) reset(x)
            val fc=fc0.coerceAtMost(0.45/dt); val k=tan(Math.PI*fc*dt); val n=1.0/(1.0+sqrt(2.0)*k+k*k)
            val b0=k*k*n; val b1=2*b0; val b2=b0; val a1=2*(k*k-1)*n; val a2=(1-sqrt(2.0)*k+k*k)*n
            val y=x*b0+x1*b1+x2*b2-y1*a1-y2*a2
            x2=x1;x1=x;y2=y1;y1=y;return y
        }
    }

    private var calibration: Calibration?=null
    private var p=Parameters()
    private var measuring=false
    private var gUsed=9.80665
    private var gravityLocal=Vec3.ZERO
    private var gravityUnitLocal=Vec3.ZERO
    private var startNs: Long?=null
    private var segmentId=0
    private var qSegmentStart=Quaternion.IDENTITY
    private var currentGyroBias=Vec3.ZERO
    private var segmentStartGyroBias=Vec3.ZERO
    private var segmentStartPosition=Vec3.ZERO
    private var correctedTotalDistance=0.0
    private var finalizedSegments=0
    private var lastSummary: SegmentSummary?=null
    private val summaries=mutableListOf<SegmentSummary>()

    private val gyroEvents=mutableListOf<TimedVec3>()
    private val accelEvents=mutableListOf<AccelEvent>()
    private val finalizedSamples=mutableListOf<Sample>()
    private val liveSamples=mutableListOf<Sample>()

    // live provisional state, gyro-only using the segment start bias
    private var qLive=Quaternion.IDENTITY
    private var latestGyro=Vec3.ZERO
    private var lastGyroNs:Long?=null; private var lastAccelNs:Long?=null
    private var liveV=Vec3.ZERO; private var liveP=Vec3.ZERO; private var liveA=Vec3.ZERO; private var prevLiveA:Vec3?=null
    private var liveDistance=0.0; private var prevLiveSpeed=0.0
    private val liveLpf=Butterworth2LowPass()
    private var state=State()

    fun setCalibration(c: Calibration){calibration=c}
    fun calibration()=calibration
    fun parameters()=p
    fun gUsed()=gUsed
    fun isMeasuring()=measuring
    fun state()=state
    fun exportSamples(): List<Sample> = finalizedSamples + liveSamples
    fun segmentSummaries(): List<SegmentSummary> = summaries.toList()

    fun start(gOverride: Double, params: Parameters){
        val c=requireNotNull(calibration); params.validate(); require(gOverride in 5.0..15.0)
        p=params; gUsed=gOverride; gravityLocal=c.measuredGravityVectorBody.normalized()*gUsed; gravityUnitLocal=gravityLocal.normalized()
        measuring=true; startNs=null; segmentId=0; qSegmentStart=Quaternion.IDENTITY; currentGyroBias=c.gyroBiasBody; segmentStartGyroBias=currentGyroBias
        segmentStartPosition=Vec3.ZERO; correctedTotalDistance=0.0; finalizedSegments=0; lastSummary=null; summaries.clear(); finalizedSamples.clear()
        resetSegmentBuffersAndLive()
        state=State(measuring=true)
    }

    fun stop(){measuring=false; state=state.copy(measuring=false)}

    private fun resetSegmentBuffersAndLive(){
        gyroEvents.clear(); accelEvents.clear(); liveSamples.clear(); qLive=qSegmentStart; latestGyro=Vec3.ZERO; lastGyroNs=null; lastAccelNs=null
        liveV=Vec3.ZERO; liveP=segmentStartPosition; liveA=Vec3.ZERO; prevLiveA=null; liveDistance=0.0; prevLiveSpeed=0.0; liveLpf.reset(Vec3.ZERO)
    }

    fun onGyroscope(t:Long, raw:Vec3){
        latestGyro=raw; if(!measuring)return; if(startNs==null)startNs=t; gyroEvents.add(TimedVec3(t,raw))
        val prev=lastGyroNs; lastGyroNs=t; if(prev==null)return; val dt=(t-prev)*1e-9; if(dt !in 0.0001..0.1)return
        qLive=(qLive*Quaternion.fromBodyAngularVelocity(raw-segmentStartGyroBias,dt)).normalized()
    }

    fun onAccelerometer(t:Long, raw:Vec3):State{
        if(!measuring)return state
        if(startNs==null)startNs=t; accelEvents.add(AccelEvent(t,raw,latestGyro))
        val lin=qLive.rotate(raw)-gravityLocal; val prev=lastAccelNs; lastAccelNs=t
        if(prev==null){liveLpf.reset(lin);liveA=lin;prevLiveA=lin}
        else { val dt=(t-prev)*1e-9; if(dt in 0.0001..0.1){liveA=liveLpf.update(lin,p.lowPassCutoffHz,dt); val oldV=liveV; val ap=prevLiveA?:liveA; liveV=liveV+(ap+liveA)*(0.5*dt); liveP=liveP+(oldV+liveV)*(0.5*dt); val sp=liveV.norm(); liveDistance+=0.5*(prevLiveSpeed+sp)*dt; prevLiveSpeed=sp; prevLiveA=liveA} }
        val elapsed=startNs?.let{(t-it)*1e-9}?:0.0
        val s=Sample(segmentId,false,t,elapsed,raw,latestGyro,qLive,lin,liveA,liveA,liveV,liveP,liveV.norm(),correctedTotalDistance+liveDistance)
        liveSamples.add(s)
        state=State(true,segmentId,elapsed,raw,latestGyro,liveA,liveV,liveP,liveV.norm(),correctedTotalDistance,finalizedSegments,false,lastSummary)
        return state
    }

    fun finalizeStationary(end: Calibration): SegmentSummary {
        require(measuring); require(accelEvents.size>=10 && gyroEvents.size>=10)
        val acc=accelEvents.toList(); val gyr=gyroEvents.toList(); val t0=minOf(acc.first().t,gyr.first().t); val t1=maxOf(acc.last().t,gyr.last().t); val T=(t1-t0)*1e-9
        require(T>0.1)
        // Reintegrate gyro with bias slowly bridged from the start-stop estimate to the end-stop estimate.
        val qRawAtAccel=arrayOfNulls<Quaternion>(acc.size)
        var qRaw=qSegmentStart; var lastT=t0; var heldGyro=gyr.first().v; var gi=0; var ai=0
        fun biasAt(t:Long):Vec3{val u=((t-t0).toDouble()/(t1-t0).toDouble()).coerceIn(0.0,1.0);return segmentStartGyroBias+(end.gyroBiasBody-segmentStartGyroBias)*(u*p.gyroBiasBridgeStrength)}
        while(gi<gyr.size || ai<acc.size){
            val tg=if(gi<gyr.size)gyr[gi].t else Long.MAX_VALUE; val ta=if(ai<acc.size)acc[ai].t else Long.MAX_VALUE; val tn=minOf(tg,ta)
            if(tn>lastT){val dt=(tn-lastT)*1e-9; val tm=lastT+(tn-lastT)/2L; qRaw=(qRaw*Quaternion.fromBodyAngularVelocity(heldGyro-biasAt(tm),dt)).normalized();lastT=tn}
            if(tg<=ta){heldGyro=gyr[gi].v;gi++}else{qRawAtAccel[ai]=qRaw;ai++}
        }
        val qRawEnd=(qRawAtAccel.lastOrNull()?:qRaw)!!
        val endGravityInLocalPred=qRawEnd.rotate(end.measuredGravityVectorBody.normalized())
        val fullClosure=Quaternion.fromTwoVectors(endGravityInLocalPred,gravityUnitLocal)
        val closure=Quaternion.slerp(Quaternion.IDENTITY,fullClosure,p.tiltClosureStrength)
        val closureDeg=closure.angleRad()*180.0/Math.PI

        val lpf=Butterworth2LowPass(); val vRaw=Array(acc.size){Vec3.ZERO}; val pRaw=Array(acc.size){Vec3.ZERO}; val filt=Array(acc.size){Vec3.ZERO}; val lin=Array(acc.size){Vec3.ZERO}; val qCorr=Array(acc.size){Quaternion.IDENTITY}
        var v=Vec3.ZERO; var pos=Vec3.ZERO; var prevA=Vec3.ZERO; var prevT:Long?=null
        for(i in acc.indices){
            val u=((acc[i].t-t0).toDouble()/(t1-t0).toDouble()).coerceIn(0.0,1.0); val cu=Quaternion.slerp(Quaternion.IDENTITY,closure,u); val qc=(cu*qRawAtAccel[i]!!).normalized();qCorr[i]=qc
            val l=qc.rotate(acc[i].a)-gravityLocal;lin[i]=l
            if(i==0){lpf.reset(l);filt[i]=l;prevA=l}else{val dt=(acc[i].t-(prevT?:acc[i-1].t))*1e-9; val aF=if(dt in 0.0001..0.1)lpf.update(l,p.lowPassCutoffHz,dt) else l;filt[i]=aF;if(dt in 0.0001..0.1){val oldV=v;v=v+(prevA+aF)*(0.5*dt);pos=pos+(oldV+v)*(0.5*dt)};prevA=aF}
            vRaw[i]=v;pRaw[i]=pos;prevT=acc[i].t
        }
        val vEnd=vRaw.last(); val strength=p.endpointVelocityCorrectionStrength; val biasAccel=vEnd*(strength/T)
        var corrPos=Vec3.ZERO; var segDist=0.0; var prevVC=Vec3.ZERO; var prevSpeed=0.0; var prevTime=acc.first().t
        val corrected=ArrayList<Sample>(acc.size)
        for(i in acc.indices){
            val u=((acc[i].t-t0).toDouble()/(t1-t0).toDouble()).coerceIn(0.0,1.0); val vc=vRaw[i]-vEnd*(strength*u)
            val dt=if(i==0)0.0 else (acc[i].t-prevTime)*1e-9
            if(i>0 && dt in 0.0001..0.1){corrPos=corrPos+(prevVC+vc)*(0.5*dt); val sp=vc.norm();segDist+=0.5*(prevSpeed+sp)*dt;prevSpeed=sp}else if(i==0)prevSpeed=vc.norm()
            prevVC=vc;prevTime=acc[i].t
            val elapsed=startNs?.let{(acc[i].t-it)*1e-9}?:0.0
            corrected.add(Sample(segmentId,true,acc[i].t,elapsed,acc[i].a,acc[i].gyroSnapshot,qCorr[i],lin[i],filt[i],filt[i]-biasAccel,vc,segmentStartPosition+corrPos,vc.norm(),correctedTotalDistance+segDist))
        }
        finalizedSamples.addAll(corrected); liveSamples.clear(); correctedTotalDistance+=segDist; segmentStartPosition+=corrPos
        val summary=SegmentSummary(segmentId,T,vEnd,vEnd.norm(),closureDeg,segmentStartGyroBias,end.gyroBiasBody,segDist,corrPos,acc.size,gyr.size)
        lastSummary=summary; summaries.add(summary); finalizedSegments++; segmentId++; currentGyroBias=end.gyroBiasBody; segmentStartGyroBias=currentGyroBias; qSegmentStart=(closure*qRawEnd).normalized()
        resetSegmentBuffersAndLive()
        val nowElapsed=startNs?.let{(t1-it)*1e-9}?:0.0
        state=State(true,segmentId,nowElapsed,end.measuredGravityVectorBody,Vec3.ZERO,Vec3.ZERO,Vec3.ZERO,segmentStartPosition,0.0,correctedTotalDistance,finalizedSegments,true,summary)
        return summary
    }
}

class CalibrationAccumulator {
    private var aSum=Vec3.ZERO; private var gSum=Vec3.ZERO; private var aSq=0.0; private var gSq=0.0; private var aN=0; private var gN=0
    fun addAccel(v:Vec3){aSum+=v;aSq+=v.norm2();aN++}
    fun addGyro(v:Vec3){gSum+=v;gSq+=v.norm2();gN++}
    fun result():RailEstimator.Calibration{
        require(aN>=20&&gN>=20); val ma=aSum/aN.toDouble();val mg=gSum/gN.toDouble();val g=ma.norm();require(g in 5.0..15.0)
        val asd=sqrt(max(0.0,aSq/aN-ma.norm2()));val gsd=sqrt(max(0.0,gSq/gN-mg.norm2()))
        return RailEstimator.Calibration(ma,mg,g,aN,gN,asd,gsd)
    }
}
