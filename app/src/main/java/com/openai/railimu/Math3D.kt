package com.openai.railimu

import kotlin.math.*

data class Vec3(val x: Double, val y: Double, val z: Double) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun unaryMinus() = Vec3(-x, -y, -z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)
    operator fun div(s: Double) = Vec3(x / s, y / s, z / s)
    fun dot(o: Vec3) = x * o.x + y * o.y + z * o.z
    fun cross(o: Vec3) = Vec3(y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x)
    fun norm2() = dot(this)
    fun norm() = sqrt(norm2())
    fun normalized(): Vec3 { val n=norm(); return if(n>1e-12) this/n else ZERO }
    companion object { val ZERO=Vec3(0.0,0.0,0.0) }
}

data class Quaternion(val w: Double, val x: Double, val y: Double, val z: Double) {
    operator fun times(o: Quaternion) = Quaternion(
        w*o.w-x*o.x-y*o.y-z*o.z,
        w*o.x+x*o.w+y*o.z-z*o.y,
        w*o.y-x*o.z+y*o.w+z*o.x,
        w*o.z+x*o.y-y*o.x+z*o.w
    )
    fun normalized(): Quaternion { val n=sqrt(w*w+x*x+y*y+z*z); return if(n>1e-12) Quaternion(w/n,x/n,y/n,z/n) else IDENTITY }
    fun conjugate()=Quaternion(w,-x,-y,-z)
    fun rotate(v: Vec3): Vec3 {
        val tx=2.0*(y*v.z-z*v.y); val ty=2.0*(z*v.x-x*v.z); val tz=2.0*(x*v.y-y*v.x)
        return Vec3(v.x+w*tx+(y*tz-z*ty), v.y+w*ty+(z*tx-x*tz), v.z+w*tz+(x*ty-y*tx))
    }
    fun angleRad(): Double = 2.0 * acos(abs(normalized().w).coerceIn(-1.0,1.0))
    companion object {
        val IDENTITY=Quaternion(1.0,0.0,0.0,0.0)
        fun fromBodyAngularVelocity(omega: Vec3, dt: Double): Quaternion {
            val r=omega.norm(); if(r<1e-12||dt<=0.0) return IDENTITY
            val h=0.5*r*dt; val s=sin(h)/r; return Quaternion(cos(h),omega.x*s,omega.y*s,omega.z*s)
        }
        fun fromTwoVectors(from: Vec3, to: Vec3): Quaternion {
            val a=from.normalized(); val b=to.normalized(); val d=a.dot(b).coerceIn(-1.0,1.0)
            if(d>1.0-1e-12) return IDENTITY
            if(d< -1.0+1e-10) {
                val helper=if(abs(a.x)<0.8) Vec3(1.0,0.0,0.0) else Vec3(0.0,1.0,0.0)
                val axis=a.cross(helper).normalized()
                return Quaternion(0.0,axis.x,axis.y,axis.z)
            }
            val c=a.cross(b); return Quaternion(1.0+d,c.x,c.y,c.z).normalized()
        }
        fun slerp(a0: Quaternion, b0: Quaternion, t0: Double): Quaternion {
            val t=t0.coerceIn(0.0,1.0); var a=a0.normalized(); var b=b0.normalized()
            var dot=a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z
            if(dot<0.0){ b=Quaternion(-b.w,-b.x,-b.y,-b.z); dot=-dot }
            if(dot>0.9995) return Quaternion(a.w+t*(b.w-a.w),a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)).normalized()
            val th0=acos(dot.coerceIn(-1.0,1.0)); val sth0=sin(th0); val th=th0*t
            val s0=cos(th)-dot*sin(th)/sth0; val s1=sin(th)/sth0
            return Quaternion(s0*a.w+s1*b.w,s0*a.x+s1*b.x,s0*a.y+s1*b.y,s0*a.z+s1*b.z).normalized()
        }
    }
}
