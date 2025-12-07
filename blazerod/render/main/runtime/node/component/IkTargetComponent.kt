package top.fifthlight.blazerod.runtime.node.component

import net.minecraft.client.render.RenderLayer
import net.minecraft.util.Colors
import org.joml.*
import top.fifthlight.blazerod.model.TransformId
import top.fifthlight.blazerod.runtime.ModelInstanceImpl
import top.fifthlight.blazerod.runtime.node.RenderNodeImpl
import top.fifthlight.blazerod.runtime.node.UpdatePhase
import top.fifthlight.blazerod.runtime.node.getTransformMap
import top.fifthlight.blazerod.runtime.node.getWorldTransform
import kotlin.math.*

/**
 * Stable IK implementation based on your original system but hardened:
 * - safe normalization / cross product (avoid NaN)
 * - smoothing (slerp) option to reduce jitter
 * - robust plane (single-axis) solver
 * - Euler decomposition/limit only when needed
 * - preserve save/rollback logic
 *
 * Replace your original IkTargetComponent with this file (or merge the logic).
 */
class IkTargetComponent(
    val ikIndex: Int,
    val limitRadian: Float,
    val loopCount: Int,
    val chains: List<Chain>,
    val effectorNodeIndex: Int,
    val transformId: TransformId,
) : RenderNodeComponent<IkTargetComponent>() {
    override fun onClosed() {}

    override val type: Type<IkTargetComponent>
        get() = Type.IkTarget

    companion object {
        private val updatePhases = listOf(
            UpdatePhase.Type.IK_UPDATE,
            UpdatePhase.Type.DEBUG_RENDER,
        )

        private const val FLOAT_PI = PI.toFloat()
        private const val FLOAT_TWO_PI = FLOAT_PI * 2

        private val decomposeTests = listOf(
            Vector3f(FLOAT_PI, FLOAT_PI, FLOAT_PI),   // + + +
            Vector3f(FLOAT_PI, FLOAT_PI, -FLOAT_PI),  // + + -
            Vector3f(FLOAT_PI, -FLOAT_PI, FLOAT_PI),  // + - +
            Vector3f(FLOAT_PI, -FLOAT_PI, -FLOAT_PI), // + - -
            Vector3f(-FLOAT_PI, FLOAT_PI, FLOAT_PI),  // - + +
            Vector3f(-FLOAT_PI, FLOAT_PI, -FLOAT_PI), // - + -
            Vector3f(-FLOAT_PI, -FLOAT_PI, FLOAT_PI), // - - +
            Vector3f(-FLOAT_PI, -FLOAT_PI, -FLOAT_PI) // - - -
        )
    }

    override val updatePhases: List<UpdatePhase.Type>
        get() = Companion.updatePhases

    class Chain(
        val nodeIndex: Int,
        val limit: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits?,
    ) {
        val prevAngle = Vector3f()
        val saveIKRot = Quaternionf()
        var planeModeAngle: Float = 0f
    }

    // ---- helpers for Euler handling (unchanged logic adapted) ----
    private fun normalizeAngle(angle: Float): Float {
        var ret = angle
        while (ret >= FLOAT_TWO_PI) {
            ret -= FLOAT_TWO_PI
        }
        while (ret < 0) {
            ret += FLOAT_TWO_PI
        }

        return ret
    }

    private fun diffAngle(a: Float, b: Float): Float = (normalizeAngle(a) - normalizeAngle(b)).let { diff ->
        when {
            diff > FLOAT_PI -> diff - FLOAT_TWO_PI
            diff < -FLOAT_PI -> diff + FLOAT_TWO_PI
            else -> diff
        }
    }

    private val testVec = Vector3f()
    private fun decompose(m: Matrix3fc, before: Vector3fc, r: Vector3f): Vector3f {
        val sy = -m.m02()
        val e = 1e-6f
        if ((1f - abs(sy)) < e) {
            r.y = asin(sy)
            // Find the closest angle to 180 degrees
            val sx = sin(before.x())
            val sz = sin(before.z())

            if (abs(sx) < abs(sz)) {
                // X is closer to 0 or 180
                val cx = cos(before.x())
                if (cx > 0) {
                    r.x = 0f
                    r.z = asin(-m.m10())
                } else {
                    r.x = FLOAT_PI
                    r.z = asin(m.m10())
                }
            } else {
                val cz = cos(before.z())
                if (cz > 0) {
                    r.z = 0f
                    r.x = asin(-m.m21())
                } else {
                    r.z = FLOAT_PI
                    r.x = asin(m.m21())
                }
            }
        } else {
            r.x = atan2(m.m12(), m.m22())
            r.y = asin(-m.m02())
            r.z = atan2(m.m01(), m.m00())
        }

        val errX = abs(diffAngle(r.x, before.x()))
        val errY = abs(diffAngle(r.y, before.y()))
        val errZ = abs(diffAngle(r.z, before.z()))
        var minErr = errX + errY + errZ
        for (testDiff in decomposeTests) {
            testVec.set(r.x, -r.y, r.z).add(testDiff)
            val err = abs(diffAngle(testVec.x(), before.x())) +
                    abs(diffAngle(testVec.y(), before.y())) +
                    abs(diffAngle(testVec.z(), before.z()))
            if (err < minErr) {
                minErr = err
                r.set(testVec)
            }
        }
        return r
    }

    private fun Vector3fc.getAxis(axis: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis) = when (axis) {
        top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.X -> x()
        top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.Y -> y()
        top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.Z -> z()
    }

    // ---- safe math utilities ----
    private fun safeNormalizeInPlace(v: Vector3f, fallback: Vector3f = Vector3f(0f, 1f, 0f), eps2: Float = 1e-8f): Vector3f {
        val l2 = v.lengthSquared()
        return if (l2 < eps2) {
            v.set(fallback)
        } else {
            v.mul(1f / sqrt(l2))
        }
    }

    private fun safeNormalize(v: Vector3f, out: Vector3f, fallback: Vector3f = Vector3f(0f, 1f, 0f), eps2: Float = 1e-8f): Vector3f {
        val l2 = v.lengthSquared()
        return if (l2 < eps2) {
            out.set(fallback)
        } else {
            out.set(v).mul(1f / sqrt(l2))
        }
    }

    private fun safeCross(a: Vector3f, b: Vector3f, out: Vector3f, fallback: Vector3f = Vector3f(0f, 1f, 0f), eps2: Float = 1e-8f): Vector3f {
        a.cross(b, out)
        val l2 = out.lengthSquared()
        if (l2 < eps2) {
            // vectors nearly parallel -> pick a stable axis
            // choose a fallback perpendicular to 'a' if possible
            val ax = abs(a.x)
            val ay = abs(a.y)
            val az = abs(a.z)
            if (ax < ay && ax < az) {
                // a is closest to X=0 plane, cross with X axis
                out.set(1f, 0f, 0f).cross(a, out).normalize()
            } else if (ay < az) {
                out.set(0f, 1f, 0f).cross(a, out).normalize()
            } else {
                out.set(0f, 0f, 1f).cross(a, out).normalize()
            }
        } else {
            out.mul(1f / sqrt(l2))
        }
        return out
    }

    // optionally smooth rotation application to prevent jitter
    private val applySmoothing = true
    private val smoothingFactor = 0.85f // 0..1 ; larger means less smoothing (closer to new rotation)

    // ---- reusable temporaries to avoid allocations ----
    private val targetPos = Vector3f()
    private val ikPos = Vector3f()
    private val invChain = Matrix4f()
    private val chainIkPos = Vector3f()
    private val chainTargetPos = Vector3f()
    private val prevRotationInv = Quaternionf()

    private val tmpAxis = Vector3f()
    private val tmpRot = Quaternionf()
    private val tmpChainRot = Quaternionf()
    private val tmpChainRotM = Matrix3f()
    private val tmpRotXYZ = Vector3f()
    private val tmpSaveRot = Quaternionf()
    private val tmpQuatSlerp = Quaternionf()

    // ---- core solver ----
    private fun solveCore(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iterateCount: Int,
    ) {
        // get effector world pos once
        instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)

        // traverse in order given by 'chains'. For CCD effect typical list should be from root->end or end->root depending on how you built list.
        // This implementation assumes chains are ordered from root->effector; earlier code used chains.last() for update -> preserve that behavior.
        for (chain in chains) {
            if (chain.nodeIndex == node.nodeIndex) {
                // target node equals chain node -> skip
                continue
            }

            val limit = chain.limit
            val axis = limit?.singleAxis
            if (axis != null) {
                // single-axis specialized solver
                solvePlane(node, instance, iterateCount, chain, limit, axis)
                continue
            }

            // world target pos and chain inverse transform
            instance.getWorldTransform(node).getTranslation(targetPos)
            instance.getWorldTransform(chain.nodeIndex).invert(invChain)

            // transform ik & target into chain local space
            invChain.transformPosition(ikPos, chainIkPos)
            invChain.transformPosition(targetPos, chainTargetPos)

            // safe normalize (avoid NaN when positions are near zero)
            val ikLen2 = chainIkPos.lengthSquared()
            val tgtLen2 = chainTargetPos.lengthSquared()

            // If both positions are nearly zero we cannot compute a meaningful rotation
            if (ikLen2 < 1e-8f || tgtLen2 < 1e-8f) {
                // skip this chain to avoid NaN propagation
                continue
            }

            // compute normalized direction vectors in chain local
            safeNormalize(chainIkPos, chainIkPos)     // normalized in-place
            safeNormalize(chainTargetPos, chainTargetPos)

            // dot -> angle
            var dot = chainTargetPos.dot(chainIkPos).coerceIn(-1f, 1f)
            var angle = acos(dot)

            // small-angle skip -> no update
            if (angle < 1e-6f) {
                continue
            }

            // clamp single-step angle
            angle = angle.coerceIn(-limitRadian, limitRadian)

            // axis: robust cross (avoid zero axis)
            safeCross(chainTargetPos, chainIkPos, tmpAxis)

            // create rotation quaternion
            tmpRot.rotationAxis(angle, tmpAxis)

            // accumulate with current 'sum' rotation for this transformId
            val currentSumRot = instance.getTransformMap(chain.nodeIndex)
                .getSum(transformId)
                .getUnnormalizedRotation(tmpChainRot)

            tmpChainRot.mul(tmpRot) // chainRot = currentSumRot * tmpRot

            // If joint has Euler limits, decompose and clamp
            if (limit != null) {
                tmpChainRotM.rotation(tmpChainRot)
                decompose(tmpChainRotM, chain.prevAngle, tmpRotXYZ)
                // clamp per-axis to limits
                val clampXYZ = tmpRotXYZ.coerceIn(limit.min, limit.max)
                    .sub(chain.prevAngle).coerceIn(-limitRadian, limitRadian).add(chain.prevAngle)
                tmpChainRotM.rotationXYZ(clampXYZ.x, clampXYZ.y, clampXYZ.z)
                chain.prevAngle.set(clampXYZ)
                tmpChainRot = Quaternionf().setFromNormalized(tmpChainRotM)
            }

            // compute rotation to write relative to previous transform (prev slot)
            instance.getTransformMap(chain.nodeIndex)
                .getSum(transformId.prev)
                .getUnnormalizedRotation(prevRotationInv).invert()

            // desired local delta rotation to store
            val desiredLocal = Quaternionf(tmpChainRot).mul(prevRotationInv)

            // smoothing: slerp between old stored rotation and new desired rotation to avoid frame jitter
            if (applySmoothing) {
                // get existing rotation stored in transformId (if any)
                val existing = instance.getTransformMap(chain.nodeIndex).get(transformId)
                if (existing != null) {
                    existing.getRotation(tmpSaveRot)
                } else {
                    tmpSaveRot.identity()
                }
                // slerp tmpSaveRot -> desiredLocal with factor smoothingFactor
                tmpQuatSlerp.set(tmpSaveRot).slerp(desiredLocal, smoothingFactor)
                instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                    rotation.set(tmpQuatSlerp)
                }
            } else {
                instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                    rotation.set(desiredLocal)
                }
            }

            // update node transforms so next chain sees updated global
            instance.updateNodeTransform(chain.nodeIndex)
        }
    }

    // ---- plane solver: single axis solver (X/Y/Z) ----
    private val rot1 = Quaternionf()
    private val resultVec1 = Vector3f()
    private val rot2 = Quaternionf()
    private val resultVec2 = Vector3f()

    private fun solvePlane(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iterateCount: Int,
        chain: Chain,
        limits: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits,
        axis: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis,
    ) {
        val rotateAxis = axis.axis

        instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
        instance.getWorldTransform(node).getTranslation(targetPos)
        instance.getWorldTransform(chain.nodeIndex).invert(invChain)
        invChain.transformPosition(ikPos, chainIkPos)
        invChain.transformPosition(targetPos, chainTargetPos)

        // safe normalization
        val ikLen2 = chainIkPos.lengthSquared()
        val tgtLen2 = chainTargetPos.lengthSquared()
        if (ikLen2 < 1e-8f || tgtLen2 < 1e-8f) return

        safeNormalize(chainIkPos, chainIkPos)
        safeNormalize(chainTargetPos, chainTargetPos)

        var dot = chainTargetPos.dot(chainIkPos).coerceIn(-1f, 1f)
        val baseAngle = acos(dot).coerceIn(-limitRadian, limitRadian)

        // try rotating target around allowed axis in both directions and pick the better
        rot1.rotationAxis(baseAngle, rotateAxis)
        chainTargetPos.rotate(rot1, resultVec1)
        val dot1 = resultVec1.dot(chainIkPos)

        rot2.rotationAxis(-baseAngle, rotateAxis)
        chainTargetPos.rotate(rot2, resultVec2)
        val dot2 = resultVec2.dot(chainIkPos)

        var newAngle = chain.planeModeAngle
        if (dot1 > dot2) newAngle += baseAngle else newAngle -= baseAngle

        // initial correction on first iteration (try to pick sign that fits range)
        val limitRange = limits.min.getAxis(axis) .. limits.max.getAxis(axis)
        if (iterateCount == 0) {
            if (newAngle !in limitRange) {
                if (-newAngle in limitRange) {
                    newAngle = -newAngle
                } else {
                    val halfRad = (limitRange.start + limitRange.endInclusive) * 0.5f
                    if (abs(halfRad - newAngle) > abs(halfRad + newAngle)) {
                        newAngle = -newAngle
                    }
                }
            }
        }

        newAngle = newAngle.coerceIn(limitRange.start, limitRange.endInclusive)
        chain.planeModeAngle = newAngle

        // write rotation (apply prev invert) with smoothing
        instance.getTransformMap(chain.nodeIndex)
            .getSum(transformId.prev)
            .getUnnormalizedRotation(prevRotationInv).invert()

        val desired = Quaternionf().rotationAxis(newAngle, rotateAxis).mul(prevRotationInv)

        if (applySmoothing) {
            val existing = instance.getTransformMap(chain.nodeIndex).get(transformId)
            if (existing != null) existing.getRotation(tmpSaveRot) else tmpSaveRot.identity()
            tmpQuatSlerp.set(tmpSaveRot).slerp(desired, smoothingFactor)
            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                rotation.set(tmpQuatSlerp)
            }
        } else {
            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                rotation.set(desired)
            }
        }

        instance.updateNodeTransform(chain.nodeIndex)
    }

    override fun update(
        phase: UpdatePhase,
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
    ) {
        val enabled = instance.modelData.ikEnabled[ikIndex]
        if (!enabled) {
            return
        }
        when (phase) {
            is UpdatePhase.IkUpdate -> {
                if (chains.isEmpty()) {
                    return
                }
                // initialize per-iteration state
                for (chain in chains) {
                    chain.prevAngle.set(0f)
                    instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                        rotation.identity()
                    }
                    chain.planeModeAngle = 0f
                }

                // ensure transforms up to last chain are up-to-date
                instance.updateNodeTransform(chains.last().nodeIndex)

                var maxDist = Float.MAX_VALUE
                for (i in 0 until loopCount) {
                    solveCore(node, instance, i)

                    instance.getWorldTransform(node).getTranslation(targetPos)
                    instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
                    val dist = targetPos.distanceSquared(ikPos)

                    if (dist.isNaN()) {
                        // Numerical trouble, rollback and break
                        for (chain in chains) {
                            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                                rotation.set(chain.saveIKRot)
                            }
                        }
                        instance.updateNodeTransform(chains.last().nodeIndex)
                        break
                    }

                    if (dist < maxDist) {
                        maxDist = dist
                        // save rotations (avoid saving NaN)
                        for (chain in chains) {
                            val matrix = instance.getTransformMap(chain.nodeIndex).get(transformId)
                            if (matrix != null) {
                                matrix.getRotation(chain.saveIKRot)
                            } else {
                                chain.saveIKRot.identity()
                            }
                        }
                    } else {
                        // no improvement: rollback to saved best and stop
                        for (chain in chains) {
                            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                                rotation.set(chain.saveIKRot)
                            }
                        }
                        instance.updateNodeTransform(chains.last().nodeIndex)
                        break
                    }
                }
            }

            is UpdatePhase.DebugRender -> {
                val consumers = phase.vertexConsumerProvider

                val boxBuffer = consumers.getBuffer(RenderLayer.getDebugQuads())
                for (joint in chains) {
                    val jointMatrix = phase.viewProjectionMatrix.mul(
                        instance.getWorldTransform(joint.nodeIndex),
                        phase.cacheMatrix
                    )
                    boxBuffer.drawBox(jointMatrix, 0.005f, Colors.BLUE)
                }

                val effectorMatrix =
                    phase.viewProjectionMatrix.mul(instance.getWorldTransform(effectorNodeIndex), phase.cacheMatrix)
                boxBuffer.drawBox(effectorMatrix, 0.01f, Colors.RED)

                val targetMatrix =
                    phase.viewProjectionMatrix.mul(instance.getWorldTransform(node), phase.cacheMatrix)
                boxBuffer.drawBox(targetMatrix, 0.01f, Colors.GREEN)

                val lineBuffer = consumers.getBuffer(DEBUG_RENDER_LAYER)
                for (joint in chains) {
                    val jointMatrix = phase.viewProjectionMatrix.mul(
                        instance.getWorldTransform(joint.nodeIndex),
                        phase.cacheMatrix
                    )
                    val lineSize = .05f
                    lineBuffer.vertex(jointMatrix, 0.0f, 0.0f, 0.0f).color(Colors.RED).normal(0.0f, 1.0f, 0.0f)
                    lineBuffer.vertex(jointMatrix, lineSize, 0.0f, 0.0f).color(Colors.RED).normal(0.0f, 1.0f, 0.0f)
                    lineBuffer.vertex(jointMatrix, 0.0f, lineSize, 0.0f).color(Colors.GREEN).normal(0.0f, 1.0f, 0.0f)
                    lineBuffer.vertex(jointMatrix, 0.0f, lineSize, lineSize).color(Colors.GREEN)
                        .normal(0.0f, 1.0f, 0.0f)
                    lineBuffer.vertex(jointMatrix, lineSize, 0.0f, 0.0f).color(Colors.BLUE).normal(0.0f, 1.0f, 0.0f)
                    lineBuffer.vertex(jointMatrix, lineSize, 0.0f, lineSize).color(Colors.BLUE).normal(0.0f, 1.0f, 0.0f)
                }
            }

            else -> {}
        }
    }
}
