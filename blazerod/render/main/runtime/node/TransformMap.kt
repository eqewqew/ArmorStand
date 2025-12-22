package top.fifthlight.blazerod.runtime.node

import org.joml.Matrix4f
import org.joml.Matrix4fc
import org.joml.Quaternionf
import org.joml.Vector3f
import top.fifthlight.blazerod.model.NodeTransform
import top.fifthlight.blazerod.model.NodeTransformView
import top.fifthlight.blazerod.model.TransformId
import java.util.*

/**
 * (✨ω✨) 主人的终极 TransformMap (完整修复版 v3.1)
 * 核心功能：
 * 1. 【防扭曲】：通过 parentProvider 理解真实的父子关系，不再把大腿接到头顶上！
 * 2. 【防抖动】：使用 setGlobalMatrix 自动转换坐标系，让 IK 的世界坐标稳稳落地。
 * 3. 【高性能】：递归脏标记 (markDirty) 确保只有动了的骨骼才会重算。
 */
class TransformMap(
    first: NodeTransformView?,
    // 这里的 parentProvider 就是用来接收 ModelInstanceImpl 里传过来的“认爹逻辑”的！
    // 增加了默认值是为了防止其他地方调用报错，默认还是“贪吃蛇”逻辑，但 ModelInstanceImpl 会覆盖它。
    private val parentProvider: (TransformId) -> TransformId? = { id -> 
        if (id.ordinal > 0) TransformId.entries[id.ordinal - 1] else null 
    }
) {
    // 存储局部变换 (Local Space) - 所有的移动最终都存在这里
    val transforms = EnumMap<TransformId, NodeTransform>(TransformId::class.java).also {
        it[TransformId.FIRST] = first?.clone() ?: NodeTransform.Decomposed()
    }

    private val dirtyTransforms = EnumSet.noneOf(TransformId::class.java)

    // 缓存累积矩阵 (World Space) - 也就是绝对坐标
    private val intermediateMatrices = EnumMap<TransformId, Matrix4f>(TransformId::class.java).also {
        it[TransformId.FIRST] = Matrix4f().also { matrix -> first?.applyOnMatrix(matrix) }
    }

    // 子节点查找表：用于正确的脏标记传播
    private val childrenMap = EnumMap<TransformId, MutableList<TransformId>>(TransformId::class.java)

    init {
        // (🏗️) 构造父子关系树
        // 这样当我们标记一个节点脏了的时候，就能顺藤摸瓜把它的所有孩子都标记脏
        for (id in TransformId.entries) {
            val parent = if (id == TransformId.FIRST) null else parentProvider(id)
            if (parent != null) {
                childrenMap.getOrPut(parent) { ArrayList() }.add(id)
            }
        }
    }

    // (🌲) 递归标记脏数据
    // 以前是 markDirty(id..end)，现在是精准打击！
    private fun markDirty(id: TransformId) {
        if (!dirtyTransforms.add(id)) return // 如果已经是脏的，就不用再跑了
        
        // 找到所有的孩子，继续把它们标记为脏
        childrenMap[id]?.forEach { child ->
            markDirty(child)
        }
    }

    fun clearFrom(id: TransformId = TransformId.FIRST) {
        // 清理缓存，这步很重要
        transforms.keys.removeIf { it >= id }
        intermediateMatrices.keys.removeIf { it >= id }
        markDirty(id)
    }

    private val tempAccumulatedMatrix = Matrix4f()
    private val pathStack = ArrayList<TransformId>()

    // (🧮) 核心计算逻辑：按树形结构回溯
    private fun calculateIntermediateMatrices(targetId: TransformId): Matrix4fc {
        pathStack.clear()
        var currentTracer: TransformId? = targetId

        // 1. 向上回溯，直到找到一个“干净”的祖先或者根节点
        while (currentTracer != null) {
            if (currentTracer !in dirtyTransforms && intermediateMatrices.containsKey(currentTracer)) {
                break
            }
            pathStack.add(currentTracer)
            
            // 使用 parentProvider 找爸爸，而不是 ordinal - 1
            if (currentTracer == TransformId.FIRST) {
                currentTracer = null
            } else {
                currentTracer = parentProvider(currentTracer)
            }
        }

        // 2. 初始化基准矩阵
        if (currentTracer != null) {
            // 找到了干净的祖先，就从它开始算
            tempAccumulatedMatrix.set(intermediateMatrices[currentTracer]!!)
        } else {
            // 没找到缓存，或者追溯到了根节点，或者根节点自己也是脏的
             if (pathStack.isNotEmpty() && pathStack.last() == TransformId.FIRST) {
               // 如果栈顶是 FIRST，说明 FIRST 也要重算
               transforms[TransformId.FIRST]!!.setOnMatrix(tempAccumulatedMatrix)
               pathStack.removeAt(pathStack.lastIndex)
            } else {
                // 兜底：单位矩阵
                tempAccumulatedMatrix.identity()
            }
        }

        // 3. 从上往下（栈的逆序）应用变换，算出最终结果
        for (i in pathStack.indices.reversed()) {
            val id = pathStack[i]
            val transform = transforms[id]
            
            if (transform != null) {
                // 矩阵乘法：ParentWorld * ChildLocal
                transform.applyOnMatrix(tempAccumulatedMatrix)
            }
            
            // 顺便把沿途算出来的矩阵都缓存起来，下次就不用重算了
            val matrixToUpdate = intermediateMatrices.getOrPut(id) { Matrix4f() }
            matrixToUpdate.set(tempAccumulatedMatrix)
            dirtyTransforms.remove(id)
        }

        return tempAccumulatedMatrix
    }

    fun get(id: TransformId): NodeTransformView? = transforms[id]

    fun getSum(id: TransformId): Matrix4fc {
        // 智能获取：如果是脏的就重算，不脏就直接给缓存
        return if (dirtyTransforms.contains(id)) {
            calculateIntermediateMatrices(id)
        } else {
            intermediateMatrices[id] ?: calculateIntermediateMatrices(id)
        }
    }

    // ==========================================
    // (✨神器) 新增：设置全局矩阵 (setGlobalMatrix)
    // ==========================================
    private val tempParentInverse = Matrix4f()
    private val tempLocalMatrix = Matrix4f()

    /**
     * 设置指定节点的【世界空间】变换。
     * 就算 IK 给你的是世界坐标，我也能自动把它转成局部坐标存起来！
     * 这就是解决“父动子乱动”的关键！
     */
    fun setGlobalMatrix(id: TransformId, globalMatrix: Matrix4fc) {
        val parent = if (id == TransformId.FIRST) null else parentProvider(id)

        if (parent == null) {
            // 没有父节点，全局就是局部
            setMatrix(id, globalMatrix)
        } else {
            // 有父节点：局部 = 父节点逆矩阵 * 全局
            // 1. 获取父节点最新的全局矩阵 (getSum 会自动处理脏标记)
            val parentGlobal = getSum(parent)
            
            // 2. 计算父节点的逆矩阵
            parentGlobal.invert(tempParentInverse)
            
            // 3. 计算局部矩阵：ParentInverse * TargetGlobal
            tempParentInverse.mul(globalMatrix, tempLocalMatrix)
            
            // 4. 存进去！
            setMatrix(id, tempLocalMatrix)
        }
    }

    // ==========================================
    // 下面是各种设置方法，记得它们都调用了递归的 markDirty
    // ==========================================

    fun updateDecomposed(id: TransformId, updater: NodeTransform.Decomposed.() -> Unit) {
        val currentTransform = transforms[id]
        val targetTransform: NodeTransform.Decomposed

        if (currentTransform is NodeTransform.Decomposed) {
            targetTransform = currentTransform
        } else {
             targetTransform = NodeTransform.Decomposed(
                translation = currentTransform?.getTranslation(Vector3f()) ?: Vector3f(),
                rotation = currentTransform?.getRotation(Quaternionf()) ?: Quaternionf(),
                scale = currentTransform?.getScale(Vector3f()) ?: Vector3f(1f)
            )
            transforms[id] = targetTransform
            intermediateMatrices.getOrPut(id, ::Matrix4f)
        }
        
        updater(targetTransform)
        markDirty(id) // <--- 递归标记！
    }

    fun updateMatrix(id: TransformId, updater: NodeTransform.Matrix.() -> Unit) {
        val currentTransform = transforms[id]
        val targetTransform: NodeTransform.Matrix

        if (currentTransform is NodeTransform.Matrix) {
            targetTransform = currentTransform
        } else {
            targetTransform = NodeTransform.Matrix().apply {
                currentTransform?.setOnMatrix(matrix)
            }
            transforms[id] = targetTransform
            intermediateMatrices.getOrPut(id, ::Matrix4f)
        }

        updater(targetTransform)
        markDirty(id)
    }

    fun updateBedrock(id: TransformId, updater: NodeTransform.Bedrock.() -> Unit) {
        val current = transforms[id]
        val bedrock = if (current is NodeTransform.Bedrock) {
            current
        } else {
            val pivot = (transforms[TransformId.ABSOLUTE] as? NodeTransform.Bedrock)?.pivot
            val new = NodeTransform.Bedrock(
                pivot = pivot ?: Vector3f(),
                rotation = Quaternionf(),
                translation = Vector3f(),
                scale = Vector3f(1f),
            )
            transforms[id] = new
            intermediateMatrices.getOrPut(id) { Matrix4f() }
            new
        }
        updater(bedrock)
        markDirty(id)
    }

    fun setMatrix(id: TransformId, matrix: Matrix4fc) {
        val currentTransform = transforms[id]
        val targetTransform: NodeTransform.Matrix

        if (currentTransform is NodeTransform.Matrix) {
            targetTransform = currentTransform
        } else {
            targetTransform = NodeTransform.Matrix()
            transforms[id] = targetTransform
            intermediateMatrices.getOrPut(id, ::Matrix4f)
        }

        targetTransform.matrix.set(matrix)
        markDirty(id)
    }

    fun setMatrix(id: TransformId, decomposed: NodeTransformView.Decomposed) {
        val currentTransform = transforms[id]
        val targetTransform: NodeTransform.Decomposed

        if (currentTransform is NodeTransform.Decomposed) {
            targetTransform = currentTransform
        } else {
            targetTransform = NodeTransform.Decomposed()
            transforms[id] = targetTransform
            intermediateMatrices.getOrPut(id, ::Matrix4f)
        }

        targetTransform.set(decomposed)
        markDirty(id)
    }
}
