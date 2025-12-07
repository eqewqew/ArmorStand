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

        // saba code uses hardcoded pi values for decompose tests
        private val decomposeTests = listOf(
            Vector3f(FLOAT_PI, FLOAT_PI, FLOAT_PI),
            Vector3f(FLOAT_PI, FLOAT_PI, -FLOAT_PI),
            Vector3f(FLOAT_PI, -FLOAT_PI, FLOAT_PI),
            Vector3f(FLOAT_PI, -FLOAT_PI, -FLOAT_PI),
            Vector3f(-FLOAT_PI, FLOAT_PI, FLOAT_PI),
            Vector3f(-FLOAT_PI, FLOAT_PI, -FLOAT_PI),
            Vector3f(-FLOAT_PI, -FLOAT_PI, FLOAT_PI),
            Vector3f(-FLOAT_PI, -FLOAT_PI, -FLOAT_PI)
        )
    }

    override val updatePhases: List<UpdatePhase.Type>
        get() = Companion.updatePhases

    class Chain(
        val nodeIndex: Int,
        val limit: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits?,
    ) {
        // Corresponds to m_prevAngle
        val prevAngle = Vector3f()
        // Corresponds to m_saveIKRot
        val saveIKRot = Quaternionf()
        // Corresponds to m_planeModeAngle
        var planeModeAngle: Float = 0f
    }

    // --- Math Utils from saba ---

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

    private fun diffAngle(a: Float, b: Float): Float {
        val diff = normalizeAngle(a) - normalizeAngle(b)
        if (diff > FLOAT_PI) {
            return diff - FLOAT_TWO_PI
        } else if (diff < -FLOAT_PI) {
            return diff + FLOAT_TWO_PI
        }
        return diff
    }

    // saba Decompose function ported 1:1
    private fun decompose(m: Matrix3fc, before: Vector3fc, r: Vector3f): Vector3f {
        val sy = -m.m02()
        val e = 1.0e-6f
        
        if ((1.0f - abs(sy)) < e) {
            r.y = asin(sy)
            // Search for angle closer to 180 deg (PI)
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

        val pi = FLOAT_PI
        // Note: saba code constructs tests array dynamically, we use pre-allocated list but add r values
        // We need to match the logic: { r.x + pi, pi - r.y, r.z + pi } etc.
        
        var minErr = abs(diffAngle(r.x, before.x())) + 
                     abs(diffAngle(r.y, before.y())) + 
                     abs(diffAngle(r.z, before.z()))
        
        val tmp = Vector3f()
        
        // Iterating manually to match the exact combinations in saba's source
        // tests[] in C++:
        // { r.x + pi, pi - r.y, r.z + pi },
        // { r.x + pi, pi - r.y, r.z - pi },
        // ...
        
        for (test in decomposeTests) {
            // decomposeTests contains the offsets (+pi, +pi, +pi) etc.
            // But saba logic is a bit specific about signs for Y.
            // saba: pi - r.y or -pi - r.y.
            // Let's implement saba's loop explicitly to be safe.
            
            // However, the `decomposeTests` I defined earlier matches the permutation of signs if we assume we are adding them.
            // Wait, saba does `pi - r.y`. My constant list assumes adding. 
            // Let's rewrite the loop to strictly follow saba's structure.
            
            // Re-evaluating saba's test vectors:
            // 1. x+pi,  pi-y, z+pi
            // 2. x+pi,  pi-y, z-pi
            // 3. x+pi, -pi-y, z+pi
            // 4. x+pi, -pi-y, z-pi
            // 5. x-pi,  pi-y, z+pi
            // 6. x-pi,  pi-y, z-pi
            // 7. x-pi, -pi-y, z+pi
            // 8. x-pi, -pi-y, z-pi
            
            val tx = if (decomposeTests.indexOf(test) < 4) r.x + pi else r.x - pi
            val ty = if (decomposeTests.indexOf(test) % 4 < 2) pi - r.y else -pi - r.y
            val tz = if (decomposeTests.indexOf(test) % 2 == 0) r.z + pi else r.z - pi
            
            tmp.set(tx, ty, tz)
            
            val err = abs(diffAngle(tmp.x, before.x())) +
                      abs(diffAngle(tmp.y, before.y())) +
                      abs(diffAngle(tmp.z, before.z()))
            
            if (err < minErr) {
                minErr = err
                r.set(tmp)
            }
        }
        return r
    }
    
    // saba uses glm::clamp
    private fun clamp(v: Float, min: Float, max: Float): Float = v.coerceIn(min, max)
    
    private fun clampVec3(v: Vector3f, min: Vector3fc, max: Vector3fc): Vector3f {
        v.x = v.x.coerceIn(min.x(), max.x())
        v.y = v.y.coerceIn(min.y(), max.y())
        v.z = v.z.coerceIn(min.z(), max.z())
        return v
    }

    // Temporary variables to avoid allocation
    private val ikPos = Vector3f()
    private val targetPos = Vector3f()
    private val chainIkPos = Vector3f()
    private val chainTargetPos = Vector3f()
    private val invChain = Matrix4f()
    private val cross = Vector3f()
    private val rotQuat = Quaternionf()
    
    // --- Main Logic ---

    private fun solveCore(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iteration: Int,
    ) {
        // ikPos (global)
        instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)

        for (chain in chains) {
            if (chain.nodeIndex == node.nodeIndex) {
                // Same target and chain, skip to avoid NaN
                continue
            }

            // Check for Axis Limit (Knee constraint etc.)
            val limit = chain.limit
            if (limit != null) {
                // Logic from saba: Check if it's a single axis constraint
                // We use the provided singleAxis property or check limits manually if needed.
                // Assuming `singleAxis` property correctly identifies X/Y/Z only limits.
                val axis = limit.singleAxis
                if (axis != null) {
                    solvePlane(node, instance, iteration, chain, limit, axis)
                    continue
                }
            }

            // Standard IK
            instance.getWorldTransform(node).getTranslation(targetPos)
            
            // invChain
            instance.getWorldTransform(chain.nodeIndex).invert(invChain)

            // chainIkPos = invChain * ikPos
            ikPos.mulPosition(invChain, chainIkPos)
            // chainTargetPos = invChain * targetPos
            targetPos.mulPosition(invChain, chainTargetPos)

            val chainIkVec = chainIkPos.normalize()
            val chainTargetVec = chainTargetPos.normalize()

            var dot = chainTargetVec.dot(chainIkVec)
            dot = clamp(dot, -1.0f, 1.0f)

            var angle = acos(dot)
            // 1.0e-3f rad is approx 0.057 degrees
            if (angle < 1.0e-3f) {
                continue
            }

            // m_limitAngle check
            angle = clamp(angle, -limitRadian, limitRadian)

            // cross = normalize(cross(target, ik))
            chainTargetVec.cross(chainIkVec, cross).normalize()
            
            // rot = rotate(angle, cross)
            rotQuat.identity().rotateAxis(angle, cross)

            // chainRot = GetIKRotate() * rot
            // In our system, getTransformMap returns the current local rotation (including previous IK)
            val chainRot = Quaternionf()
            instance.getTransformMap(chain.nodeIndex).getSum(transformId).getUnnormalizedRotation(chainRot)
            chainRot.mul(rotQuat) // apply rotation

            if (limit != null) {
                // Decompose and Clamp
                val chainRotM = Matrix3f().set(chainRot)
                val rotXYZ = Vector3f()
                decompose(chainRotM, chain.prevAngle, rotXYZ)
                
                // clampXYZ = clamp(rotXYZ, min, max)
                clampVec3(rotXYZ, limit.min, limit.max)
                
                // clampXYZ = clamp(clampXYZ - prev, -limit, limit) + prev
                val dx = clamp(rotXYZ.x - chain.prevAngle.x, -limitRadian, limitRadian)
                val dy = clamp(rotXYZ.y - chain.prevAngle.y, -limitRadian, limitRadian)
                val dz = clamp(rotXYZ.z - chain.prevAngle.z, -limitRadian, limitRadian)
                
                rotXYZ.x = dx + chain.prevAngle.x
                rotXYZ.y = dy + chain.prevAngle.y
                rotXYZ.z = dz + chain.prevAngle.z
                
                // Recompose: r = rotate(x, 1,0,0) * rotate(y, 0,1,0) * rotate(z, 0,0,1)
                // GLM order: R = Rx * Ry * Rz (if applied as R * v)
                // JOML: rotateX(x).rotateY(y).rotateZ(z) results in `this * Rx * Ry * Rz`.
                chainRot.identity()
                    .rotateX(rotXYZ.x)
                    .rotateY(rotXYZ.y)
                    .rotateZ(rotXYZ.z)
                
                chain.prevAngle.set(rotXYZ)
            }

            // Apply to node
            // chainNode->SetIKRotate(ikRot)
            // ikRot = chainRot * inverse(AnimateRotate)
            // In our system: set transform = chainRot * inverse(prev_layer)
            
            val prevRotationInv = Quaternionf()
            instance.getTransformMap(chain.nodeIndex)
                .getSum(transformId.prev)
                .getUnnormalizedRotation(prevRotationInv)
                .invert()
            
            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                rotation.set(chainRot).mul(prevRotationInv)
            }
            instance.updateNodeTransform(chain.nodeIndex)
        }
    }

    private fun solvePlane(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iteration: Int,
        chain: Chain,
        limit: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits,
        axis: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis,
    ) {
        // Setup Axis and Plane
        // GLM: vec3(1,0,0) etc.
        val rotateAxis = Vector3f()
        val rotateAxisIndex: Int
        
        when (axis) {
            top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.X -> {
                rotateAxisIndex = 0
                rotateAxis.set(1f, 0f, 0f)
            }
            top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.Y -> {
                rotateAxisIndex = 1
                rotateAxis.set(0f, 1f, 0f)
            }
            top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.Z -> {
                rotateAxisIndex = 2
                rotateAxis.set(0f, 0f, 1f)
            }
        }

        instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
        instance.getWorldTransform(node).getTranslation(targetPos)
        instance.getWorldTransform(chain.nodeIndex).invert(invChain)

        // Local positions
        ikPos.mulPosition(invChain, chainIkPos)
        targetPos.mulPosition(invChain, chainTargetPos)

        val chainIkVec = chainIkPos.normalize()
        val chainTargetVec = chainTargetPos.normalize()

        var dot = chainTargetVec.dot(chainIkVec)
        dot = clamp(dot, -1.0f, 1.0f)

        var angle = acos(dot)
        angle = clamp(angle, -limitRadian, limitRadian)

        // Test +angle
        val rot1 = Quaternionf().rotateAxis(angle, rotateAxis)
        // targetVec1 = rot1 * chainTargetVec
        val targetVec1 = Vector3f(chainTargetVec).rotate(rot1)
        val dot1 = targetVec1.dot(chainIkVec)

        // Test -angle
        val rot2 = Quaternionf().rotateAxis(-angle, rotateAxis)
        // targetVec2 = rot2 * chainTargetVec
        val targetVec2 = Vector3f(chainTargetVec).rotate(rot2)
        val dot2 = targetVec2.dot(chainIkVec)

        var newAngle = chain.planeModeAngle
        if (dot1 > dot2) {
            newAngle += angle
        } else {
            newAngle -= angle
        }

        // 0th iteration special logic for limits
        if (iteration == 0) {
            val minLim = limit.min.get(rotateAxisIndex)
            val maxLim = limit.max.get(rotateAxisIndex)
            
            if (newAngle < minLim || newAngle > maxLim) {
                if (-newAngle > minLim && -newAngle < maxLim) {
                    newAngle *= -1
                } else {
                    val halfRad = (minLim + maxLim) * 0.5f
                    if (abs(halfRad - newAngle) > abs(halfRad + newAngle)) {
                        newAngle *= -1
                    }
                }
            }
        }

        // Clamp
        val minLim = limit.min.get(rotateAxisIndex)
        val maxLim = limit.max.get(rotateAxisIndex)
        newAngle = clamp(newAngle, minLim, maxLim)
        
        chain.planeModeAngle = newAngle

        // Apply
        // ikRotM = rotate(newAngle, axis) * inverse(AnimateRotate)
        val ikRot = Quaternionf().rotateAxis(newAngle, rotateAxis)
        
        val prevRotationInv = Quaternionf()
        instance.getTransformMap(chain.nodeIndex)
            .getSum(transformId.prev)
            .getUnnormalizedRotation(prevRotationInv)
            .invert()

        instance.setTransformDecomposed(chain.nodeIndex, transformId) {
            rotation.set(ikRot).mul(prevRotationInv)
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

                // Initialize IKChain (saba: Solve start)
                for (chain in chains) {
                    chain.prevAngle.set(0f)
                    // chain.m_node->SetIKRotate(identity)
                    instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                        rotation.identity()
                    }
                    chain.planeModeAngle = 0f
                    // Update transforms
                }
                // Update the last chain node (and consequently others if hierarchical, 
                // but we might need to loop update if they are dependent. 
                // saba calls UpdateGlobalTransform on each chain node in the loop. 
                // Here we call on the last one, assuming parent updates propagate or will be handled in SolveCore loops)
                // For safety, let's update all? Or just depend on SolveCore to update.
                // The provided code updated the last one.
                instance.updateNodeTransform(chains.last().nodeIndex)

                var maxDist = Float.MAX_VALUE
                
                for (i in 0 until loopCount) {
                    solveCore(node, instance, i)

                    // Check distance
                    instance.getWorldTransform(node).getTranslation(targetPos)
                    instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
                    
                    val dist = targetPos.distance(ikPos) // saba uses glm::length(target - ik)
                    
                    if (dist < maxDist) {
                        maxDist = dist
                        // Save rotations
                        for (chain in chains) {
                            val matrix = instance.getTransformMap(chain.nodeIndex).get(transformId)
                            if (matrix != null) {
                                matrix.getRotation(chain.saveIKRot)
                            } else {
                                chain.saveIKRot.identity()
                            }
                        }
                    } else {
                        // Restore rotations and Break
                        for (chain in chains) {
                            val prevRotationInv = Quaternionf()
                            instance.getTransformMap(chain.nodeIndex)
                                .getSum(transformId.prev)
                                .getUnnormalizedRotation(prevRotationInv)
                                .invert()
                                
                            instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                                // Restore the saved IK rotation
                                // Note: saveIKRot captured the FULL local rotation (Anim * IK) ?
                                // No, in `SolveCore` we update the node transform, so `get(transformId)`
                                // returns the current IK transform component if `transformId` is specific to IK.
                                // If `transformId` is the IK layer ID, then `get(transformId)` returns just the IK rotation.
                                
                                // Wait, `SolveCore` logic:
                                // `chainRot` includes `getSum(transformId)` which is ALL layers.
                                // Then we set `transformId` component to `chainRot * inv(prev)`.
                                // So `get(transformId)` returns the IK component.
                                // `chain.saveIKRot` stores this component.
                                // So we can just set it back.
                                
                                rotation.set(chain.saveIKRot)
                            }
                        }
                        // Need to update transforms after restore? saba does.
                        // instance.updateNodeTransform(chains.last().nodeIndex)
                        // But we are breaking, so the render phase will see the restored state.
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
