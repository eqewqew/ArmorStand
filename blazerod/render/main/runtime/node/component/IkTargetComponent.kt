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
        // 增加一个微小的阈值，防止浮点数精度问题导致的抖动
        private const val MIN_ROTATION_THRESHOLD = 1.0e-5f
        // 增加一个阻尼系数（重要！），让IK迭代更平滑，防止震荡
        // 0.5f 意味着每一步只走一半，虽然收敛慢一点点，但稳如老狗！
        private const val IK_DAMPING_FACTOR = 0.5f 

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
            val sx = sin(before.x())
            val sz = sin(before.z())

            if (abs(sx) < abs(sz)) {
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

    private fun Vector3f.coerceIn(min: Vector3fc, max: Vector3fc): Vector3f = set(
        x.coerceIn(min.x(), max.x()),
        y.coerceIn(min.y(), max.y()),
        z.coerceIn(min.z(), max.z()),
    )

    private fun Vector3f.coerceIn(min: Float, max: Float): Vector3f = set(
        x.coerceIn(min, max),
        y.coerceIn(min, max),
        z.coerceIn(min, max),
    )

    private val targetPos = Vector3f()
    private val ikPos = Vector3f()
    private val invChain = Matrix4f()
    private val chainIkPos = Vector3f()
    private val chainTargetPos = Vector3f()
    private val prevRotationInv = Quaternionf()

    private val cross = Vector3f()
    private val rot = Quaternionf()
    private val chainRot = Quaternionf()
    private val chainRotM = Matrix3f()
    private val rotXYZ = Vector3f()

    private fun solveCore(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iterateCount: Int,
    ) {
        val ikPos = instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
        for (chain in chains) {
            if (chain.nodeIndex == node.nodeIndex) {
                continue
            }
            val limit = chain.limit
            val axis = limit?.singleAxis
            if (axis != null) {
                solvePlane(node, instance, iterateCount, chain, chain.limit, axis)
                continue
            }

            val targetPos = instance.getWorldTransform(node).getTranslation(targetPos)
            val invChain = instance.getWorldTransform(chain.nodeIndex).invert(invChain)

            // Calculate local positions
            val chainIkPos = ikPos.mulPosition(invChain, chainIkPos)
            val chainTargetPos = targetPos.mulPosition(invChain, chainTargetPos)

            // Optimize: check lengths before normalizing to avoid NaN
            if (chainIkPos.lengthSquared() < 1e-8f || chainTargetPos.lengthSquared() < 1e-8f) {
                continue
            }

            val chainIkVec = chainIkPos.normalize()
            val chainTargetVec = chainTargetPos.normalize()

            val dot = chainTargetVec.dot(chainIkVec).coerceIn(-1f, 1f)

            // Apply Damping! This is crucial for fixing the "Jitter"
            // We multiply the angle by IK_DAMPING_FACTOR (e.g., 0.5)
            var angle = acos(dot) * IK_DAMPING_FACTOR

            if (abs(angle) < MIN_ROTATION_THRESHOLD) {
                continue
            }

            // Check if vectors are parallel before cross product to avoid twist/flip
            // If dot is close to 1 or -1, cross product length is near 0
            if (abs(dot) > 0.99999f) {
                continue
            }

            angle = angle.coerceIn(-limitRadian, limitRadian)
            
            // Safe normalize
            chainTargetVec.cross(chainIkVec, cross)
            if (cross.lengthSquared() < MIN_ROTATION_THRESHOLD) {
                continue
            }
            cross.normalize()
            
            val rot = rot.rotationAxis(angle, cross)

            val chainRot = instance.getTransformMap(chain.nodeIndex)
                .getSum(transformId)
                .getUnnormalizedRotation(chainRot)
                .mul(rot)
            
            if (limit != null) {
                val chainRotM = chainRotM.rotation(chainRot)
                val rotXYZ = decompose(chainRotM, chain.prevAngle, rotXYZ)
                
                // IMPORTANT: When applying limits, ensure we don't jump too drastically
                val clampXYZ = rotXYZ.coerceIn(limit.min, limit.max)
                    .sub(chain.prevAngle)
                    .coerceIn(-limitRadian, limitRadian) // Apply limit to delta
                    .add(chain.prevAngle)
                
                chainRotM.rotationXYZ(clampXYZ.x, clampXYZ.y, clampXYZ.z)
                chain.prevAngle.set(clampXYZ)

                chainRotM.getUnnormalizedRotation(chainRot)
            }

            val prevRotationInv = instance.getTransformMap(chain.nodeIndex)
                .getSum(transformId.prev)
                .getUnnormalizedRotation(prevRotationInv).invert()
            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                rotation.set(chainRot).mul(prevRotationInv)
            }
            instance.updateNodeTransform(chain.nodeIndex)
        }
    }

    private val rot1 = Quaternionf()
    private val targetVec1 = Vector3f()
    private val rot2 = Quaternionf()
    private val targetVec2 = Vector3f()
    
    private fun solvePlane(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iterateCount: Int,
        chain: Chain,
        limits: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits,
        axis: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis,
    ) {
        val rotateAxis = axis.axis

        val ikPos = instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
        val targetPos = instance.getWorldTransform(node).getTranslation(targetPos)

        val invChain = instance.getWorldTransform(chain.nodeIndex).invert(invChain)

        val chainIkPos = ikPos.mulPosition(invChain, chainIkPos)
        val chainTargetPos = targetPos.mulPosition(invChain, chainTargetPos)

        // Safety check for length
        if (chainIkPos.lengthSquared() < 1e-8f || chainTargetPos.lengthSquared() < 1e-8f) {
            return
        }

        val chainIkVec = chainIkPos.normalize()
        val chainTargetVec = chainTargetPos.normalize()

        val dot = chainTargetVec.dot(chainIkVec).coerceIn(-1f, 1f)

        // Also apply damping here
        var angle = acos(dot) * IK_DAMPING_FACTOR
        
        if (abs(angle) < MIN_ROTATION_THRESHOLD) {
            return
        }

        angle = angle.coerceIn(-limitRadian, limitRadian)

        // Determine direction
        val rot1 = rot1.rotationAxis(angle, rotateAxis)
        val targetVec1 = chainTargetVec.rotate(rot1, targetVec1)
        val dot1 = targetVec1.dot(chainIkVec)

        val rot2 = rot2.rotationAxis(-angle, rotateAxis)
        val targetVec2 = chainTargetVec.rotate(rot2, targetVec2)
        val dot2 = targetVec2.dot(chainIkVec)

        var newAngle = chain.planeModeAngle
        if (dot1 > dot2) {
            newAngle += angle
        } else {
            newAngle -= angle
        }

        val limitRange = limits.min.getAxis(axis)..limits.max.getAxis(axis)
        
        // Improve stability during first iteration to prevent snapping
        if (iterateCount == 0) {
            if (newAngle !in limitRange) {
                if (-newAngle in limitRange) {
                     // Check if flipping sign is closer to valid range to avoid weird twists
                    newAngle = -newAngle
                } else {
                    val halfRad = (limitRange.start + limitRange.endInclusive) * .5f
                    if (abs(halfRad - newAngle) > abs(halfRad + newAngle)) {
                        newAngle = -newAngle
                    }
                }
            }
        }

        newAngle = newAngle.coerceIn(limitRange)
        chain.planeModeAngle = newAngle

        val prevRotationInv = instance.getTransformMap(chain.nodeIndex)
            .getSum(transformId.prev)
            .getUnnormalizedRotation(prevRotationInv).invert()
            
        instance.setTransformDecomposed(chain.nodeIndex, transformId) {
            rotation.rotationAxis(newAngle, rotateAxis).mul(prevRotationInv)
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
                for (chain in chains) {
                    chain.prevAngle.set(0f)
                    instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                        rotation.identity()
                    }
                    chain.planeModeAngle = 0f
                }
                instance.updateNodeTransform(chains.last().nodeIndex)

                var maxDist = Float.MAX_VALUE
                for (i in 0 until loopCount) {
                    solveCore(node, instance, i)

                    val targetPos = instance.getWorldTransform(node).getTranslation(targetPos)
                    val ikPos = instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
                    
                    val dist = targetPos.distanceSquared(ikPos)
                    
                    // Optimization: If close enough, stop iterating
                    if (dist < MIN_ROTATION_THRESHOLD) break

                    if (dist < maxDist) {
                        maxDist = dist
                        for (chain in chains) {
                            val matrix = instance.getTransformMap(chain.nodeIndex).get(transformId)
                            if (matrix != null) {
                                matrix.getRotation(chain.saveIKRot)
                            } else {
                                chain.saveIKRot.identity()
                            }
                        }
                    } else {
                        for (chain in chains) {
                            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                                rotation.set(chain.saveIKRot)
                            }
                        }
                        instance.updateNodeTransform(chains.last().nodeIndex)
                        // If it's getting worse, stop earlier
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
