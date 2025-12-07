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
        
        // 阈值稍微调大一点点，过滤掉极微小的颤动
        private const val MIN_ROTATION_THRESHOLD = 0.0001f 
        // 阻尼系数，0.5 比较平衡，太小会慢，太大会抖
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
        while (ret >= FLOAT_PI) ret -= FLOAT_TWO_PI
        while (ret < -FLOAT_PI) ret += FLOAT_TWO_PI
        return ret
    }

    private fun diffAngle(a: Float, b: Float): Float = normalizeAngle(a - b)

    private val testVec = Vector3f()
    
    // 优化后的 decompose，增加了对 NaN 的防护
    private fun decompose(m: Matrix3fc, before: Vector3fc, r: Vector3f): Vector3f {
        val sy = -m.m02()
        val e = 1e-6f
        
        // 简单的反解逻辑
        if ((1f - abs(sy)) < e) {
             // Gimbal lock case
            r.y = asin(sy.coerceIn(-1f, 1f))
            val sx = sin(before.x())
            val sz = sin(before.z())

            if (abs(sx) < abs(sz)) {
                val cx = cos(before.x())
                if (cx > 0) {
                    r.x = 0f
                    r.z = asin((-m.m10()).coerceIn(-1f, 1f))
                } else {
                    r.x = FLOAT_PI
                    r.z = asin(m.m10().coerceIn(-1f, 1f))
                }
            } else {
                val cz = cos(before.z())
                if (cz > 0) {
                    r.z = 0f
                    r.x = asin((-m.m21()).coerceIn(-1f, 1f))
                } else {
                    r.z = FLOAT_PI
                    r.x = asin(m.m21().coerceIn(-1f, 1f))
                }
            }
        } else {
            r.x = atan2(m.m12(), m.m22())
            r.y = asin(sy.coerceIn(-1f, 1f))
            r.z = atan2(m.m01(), m.m00())
        }

        // 寻找最小变化的解，防止跳变
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

    // 重命名一下防止歧义
    private fun Vector3f.coerceInVectors(min: Vector3fc, max: Vector3fc): Vector3f = set(
        x.coerceIn(min.x(), max.x()),
        y.coerceIn(min.y(), max.y()),
        z.coerceIn(min.z(), max.z()),
    )

    // 给标量用的 clamp，改个名字，编译器就不会搞混啦！
    private fun Vector3f.clampScalar(min: Float, max: Float): Vector3f = set(
        x.coerceIn(min, max),
        y.coerceIn(min, max),
        z.coerceIn(min, max),
    )
    
    // 辅助函数：把世界空间的向量转换到节点的局部空间（仅旋转）
    // 这对于正确计算单轴旋转非常重要
    private fun transformVecToLocal(vec: Vector3f, nodeGlobalRot: Quaternionfc): Vector3f {
        val invRot = nodeGlobalRot.invert(Quaternionf())
        return vec.rotate(invRot)
    }

    // 复用变量，减少 GC
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
    private val tmpAxisVec = Vector3f()

    private fun solveCore(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iterateCount: Int,
    ) {
        val ikPos = instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
        for (chain in chains) {
            if (chain.nodeIndex == node.nodeIndex) continue

            val limit = chain.limit
            val axis = limit?.singleAxis
            if (axis != null) {
                solvePlane(node, instance, iterateCount, chain, chain.limit, axis)
                continue
            }

            val targetPos = instance.getWorldTransform(node).getTranslation(targetPos)
            val invChain = instance.getWorldTransform(chain.nodeIndex).invert(invChain)

            val chainIkPos = ikPos.mulPosition(invChain, chainIkPos)
            val chainTargetPos = targetPos.mulPosition(invChain, chainTargetPos)

            // 1. 长度检查：防止处理极短向量导致的 NaN
            if (chainIkPos.lengthSquared() < 1e-8f || chainTargetPos.lengthSquared() < 1e-8f) continue

            val chainIkVec = chainIkPos.normalize()
            val chainTargetVec = chainTargetPos.normalize()

            val dot = chainTargetVec.dot(chainIkVec).coerceIn(-1f, 1f)
            
            // 2. 角度检查：如果已经很接近了，就不要动，防止微小抖动
            if (dot > 0.99999f) continue

            var angle = acos(dot) * IK_DAMPING_FACTOR // 阻尼
            if (abs(angle) < MIN_ROTATION_THRESHOLD) continue

            angle = angle.coerceIn(-limitRadian, limitRadian)

            chainTargetVec.cross(chainIkVec, cross)
            // 3. 叉乘长度检查：防止共线时的 NaN
            if (cross.lengthSquared() < MIN_ROTATION_THRESHOLD) continue
            cross.normalize()

            val rot = rot.rotationAxis(angle, cross)

            val chainRot = instance.getTransformMap(chain.nodeIndex)
                .getSum(transformId)
                .getUnnormalizedRotation(chainRot)
                .mul(rot)

            if (limit != null) {
                val chainRotM = chainRotM.rotation(chainRot)
                val rotXYZ = decompose(chainRotM, chain.prevAngle, rotXYZ)
                
                // 4. 角度限制逻辑优化：平滑限制
                // 修复了这里的方法调用：
                // coerceIn -> coerceInVectors
                // coerceIn -> clampScalar
                val clampXYZ = rotXYZ.coerceInVectors(limit.min, limit.max)
                    .sub(chain.prevAngle)
                    .clampScalar(-limitRadian, limitRadian)
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

    // 重写的 solvePlane：使用投影法而不是试探法
    private fun solvePlane(
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
        iterateCount: Int,
        chain: Chain,
        limits: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits,
        axis: top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis,
    ) {
        // 修复了这里的 when 语句，直接判断 axis 枚举
        val rotateAxisVec = when (axis) {
            top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.X -> Vector3f(1f, 0f, 0f)
            top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.Y -> Vector3f(0f, 1f, 0f)
            top.fifthlight.blazerod.model.IkTarget.IkJoint.Limits.Axis.Z -> Vector3f(0f, 0f, 1f)
            else -> Vector3f(1f, 0f, 0f) // Fallback for safety
        }

        val ikPos = instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
        val targetPos = instance.getWorldTransform(node).getTranslation(targetPos)
        val invChain = instance.getWorldTransform(chain.nodeIndex).invert(invChain)

        // 转换到关节局部空间
        val chainIkPos = ikPos.mulPosition(invChain, chainIkPos)
        val chainTargetPos = targetPos.mulPosition(invChain, chainTargetPos)

        if (chainIkPos.lengthSquared() < 1e-8f || chainTargetPos.lengthSquared() < 1e-8f) return

        // --- 核心修改开始 ---
        // 将向量投影到旋转平面上 (即去除旋转轴分量)
        
        // v_plane = v - axis * (v . axis)
        val ikDot = chainIkPos.dot(rotateAxisVec)
        val targetDot = chainTargetPos.dot(rotateAxisVec)
        
        // 投影向量
        chainIkPos.sub(rotateAxisVec.mul(ikDot, tmpAxisVec))
        chainTargetPos.sub(rotateAxisVec.mul(targetDot, tmpAxisVec))

        // 再次检查长度，防止投影后变成零向量（例如目标点刚好在旋转轴上）
        if (chainIkPos.lengthSquared() < 1e-8f || chainTargetPos.lengthSquared() < 1e-8f) return

        val chainIkVec = chainIkPos.normalize()
        val chainTargetVec = chainTargetPos.normalize()

        val dot = chainTargetVec.dot(chainIkVec).coerceIn(-1f, 1f)
        var deltaAngle = acos(dot) * IK_DAMPING_FACTOR // 阻尼

        if (abs(deltaAngle) < MIN_ROTATION_THRESHOLD) return

        deltaAngle = deltaAngle.coerceIn(-limitRadian, limitRadian)

        // 使用叉乘判断旋转方向
        // cross = target x ik
        chainTargetVec.cross(chainIkVec, cross)
        
        // 如果 cross 方向与旋转轴方向相同，则为正，反之为负
        if (cross.dot(rotateAxisVec) < 0) {
            deltaAngle = -deltaAngle
        }

        // --- 核心修改结束 ---

        var newAngle = normalizeAngle(chain.planeModeAngle + deltaAngle)

        // 限制角度
        val minLimit = limits.min.getAxis(axis)
        val maxLimit = limits.max.getAxis(axis)
        
        // 更严格的限制逻辑
        newAngle = newAngle.coerceIn(minLimit, maxLimit)

        // 记录状态
        chain.planeModeAngle = newAngle

        // 应用旋转
        val prevRotationInv = instance.getTransformMap(chain.nodeIndex)
            .getSum(transformId.prev)
            .getUnnormalizedRotation(prevRotationInv).invert()
            
        instance.setTransformDecomposed(chain.nodeIndex, transformId) {
            rotation.rotationAxis(newAngle, rotateAxisVec).mul(prevRotationInv)
        }
        instance.updateNodeTransform(chain.nodeIndex)
    }

    override fun update(
        phase: UpdatePhase,
        node: RenderNodeImpl,
        instance: ModelInstanceImpl,
    ) {
        val enabled = instance.modelData.ikEnabled[ikIndex]
        if (!enabled) return
        
        when (phase) {
            is UpdatePhase.IkUpdate -> {
                if (chains.isEmpty()) return

                // Reset
                for (chain in chains) {
                    chain.prevAngle.set(0f)
                    instance.setTransformDecomposed(chain.nodeIndex, transformId) {
                        rotation.identity()
                    }
                    chain.planeModeAngle = 0f
                }
                instance.updateNodeTransform(chains.last().nodeIndex)

                var maxDist = Float.MAX_VALUE
                
                // 循环解算
                for (i in 0 until loopCount) {
                    solveCore(node, instance, i)

                    val targetPos = instance.getWorldTransform(node).getTranslation(targetPos)
                    val ikPos = instance.getWorldTransform(effectorNodeIndex).getTranslation(ikPos)
                    val dist = targetPos.distanceSquared(ikPos)

                    if (dist < MIN_ROTATION_THRESHOLD) break // 足够近了就停止

                    if (dist < maxDist) {
                        maxDist = dist
                        // 保存当前最佳状态
                        for (chain in chains) {
                            val matrix = instance.getTransformMap(chain.nodeIndex).get(transformId)
                            if (matrix != null) {
                                matrix.getRotation(chain.saveIKRot)
                            } else {
                                chain.saveIKRot.identity()
                            }
                        }
                    } else {
                        // 如果距离变大了（发散），回退到上一次最佳状态并强制结束
                        // 这是一个很重要的防抖机制
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
                // 绘制关节
                for (joint in chains) {
                    val jointMatrix = phase.viewProjectionMatrix.mul(
                        instance.getWorldTransform(joint.nodeIndex),
                        phase.cacheMatrix
                    )
                    boxBuffer.drawBox(jointMatrix, 0.005f, Colors.BLUE)
                }

                // 绘制效应器
                val effectorMatrix =
                    phase.viewProjectionMatrix.mul(instance.getWorldTransform(effectorNodeIndex), phase.cacheMatrix)
                boxBuffer.drawBox(effectorMatrix, 0.01f, Colors.RED)

                // 绘制目标
                val targetMatrix =
                    phase.viewProjectionMatrix.mul(instance.getWorldTransform(node), phase.cacheMatrix)
                boxBuffer.drawBox(targetMatrix, 0.01f, Colors.GREEN)

                // 绘制轴线
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
                    lineBuffer.vertex(jointMatrix, 0.0f, lineSize, lineSize).color(Colors.GREEN).normal(0.0f, 1.0f, 0.0f)
                    lineBuffer.vertex(jointMatrix, lineSize, 0.0f, 0.0f).color(Colors.BLUE).normal(0.0f, 1.0f, 0.0f)
                    lineBuffer.vertex(jointMatrix, lineSize, 0.0f, lineSize).color(Colors.BLUE).normal(0.0f, 1.0f, 0.0f)
                }
            }
            else -> {}
        }
    }
}
