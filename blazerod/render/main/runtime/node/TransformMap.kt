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
 * (✨ω✨) 主人的终极 TransformMap (完整修复版)
 * 修复内容：
 * 1. 补全了 updateBedrock, updateMatrix, setMatrix(Decomposed) 防止编译报错。
 * 2. 构造函数 parentProvider 增加了默认值，暂时兼容 ModelInstanceImpl 的旧调用。
 */
class TransformMap(
    first: NodeTransformView?,
    // (🩹) 紧急创可贴：增加了默认值！
    // 如果调用方不传这个参数，默认认为枚举列表里“上一个”就是“爸爸” (兼容旧逻辑)
    // 等主人有空了，一定要在 ModelInstanceImpl 里传入真实的父子关系，才能彻底根治抖动哦！
    private val parentProvider: (TransformId) -> TransformId? = { id -> 
        if (id.ordinal > 0) TransformId.entries[id.ordinal - 1] else null 
    }
) {
    // 存储局部变换 (Local Space)
    val transforms = EnumMap<TransformId, NodeTransform>(TransformId::class.java).also {
        it[TransformId.FIRST] = first?.clone() ?: NodeTransform.Decomposed()
    }

    private val dirtyTransforms = EnumSet.noneOf(TransformId::class.java)

    // 缓存累积矩阵 (World Space)
    private val intermediateMatrices = EnumMap<TransformId, Matrix4f>(TransformId::class.java).also {
        it[TransformId.FIRST] = Matrix4f().also { matrix -> first?.applyOnMatrix(matrix) }
    }

    // 子节点查找表
    private val childrenMap = EnumMap<TransformId, MutableList<TransformId>>(TransformId::class.java)

    init {
        // 构建父子关系图
        for (id in TransformId.entries) {
            val parent = if (id == TransformId.FIRST) null else parentProvider(id)
            if (parent != null) {
                childrenMap.getOrPut(parent) { ArrayList() }.add(id)
            }
        }
    }

    // 递归标记脏数据
    private fun markDirty(id: TransformId) {
        if (!dirtyTransforms.add(id)) return
        childrenMap[id]?.forEach { child ->
            markDirty(child)
        }
    }

    fun clearFrom(id: TransformId = TransformId.FIRST) {
        transforms.keys.removeIf { it >= id }
        intermediateMatrices.keys.removeIf { it >= id }
        markDirty(id)
    }

    private val tempAccumulatedMatrix = Matrix4f()
    private val pathStack = ArrayList<TransformId>()

    private fun calculateIntermediateMatrices(targetId: TransformId): Matrix4fc {
        pathStack.clear()
        var currentTracer: TransformId? = targetId

        // 1. 回溯
        while (currentTracer != null) {
            if (currentTracer !in dirtyTransforms && intermediateMatrices.containsKey(currentTracer)) {
                break
            }
            pathStack.add(currentTracer)
            
            if (currentTracer == TransformId.FIRST) {
                currentTracer = null
            } else {
                currentTracer = parentProvider(currentTracer)
            }
        }

        // 2. 初始化
        if (currentTracer != null) {
            tempAccumulatedMatrix.set(intermediateMatrices[currentTracer]!!)
        } else {
             if (pathStack.isNotEmpty() && pathStack.last() == TransformId.FIRST) {
               transforms[TransformId.FIRST]!!.setOnMatrix(tempAccumulatedMatrix)
               pathStack.removeAt(pathStack.lastIndex)
            } else {
                tempAccumulatedMatrix.identity()
            }
        }

        // 3. 累积
        for (i in pathStack.indices.reversed()) {
            val id = pathStack[i]
            val transform = transforms[id]
            if (transform != null) {
                transform.applyOnMatrix(tempAccumulatedMatrix)
            }
            
            val matrixToUpdate = intermediateMatrices.getOrPut(id) { Matrix4f() }
            matrixToUpdate.set(tempAccumulatedMatrix)
            dirtyTransforms.remove(id)
        }

        return tempAccumulatedMatrix
    }

    fun get(id: TransformId): NodeTransformView? = transforms[id]

    fun getSum(id: TransformId): Matrix4fc {
        return if (dirtyTransforms.contains(id)) {
            calculateIntermediateMatrices(id)
        } else {
            intermediateMatrices[id] ?: calculateIntermediateMatrices(id)
        }
    }

    // ==========================================
    // 新增：设置全局矩阵 (防扭曲的核心！)
    // ==========================================
    private val tempParentInverse = Matrix4f()
    private val tempLocalMatrix = Matrix4f()

    fun setGlobalMatrix(id: TransformId, globalMatrix: Matrix4fc) {
        val parent = if (id == TransformId.FIRST) null else parentProvider(id)

        if (parent == null) {
            setMatrix(id, globalMatrix)
        } else {
            val parentGlobal = getSum(parent)
            parentGlobal.invert(tempParentInverse)
            tempParentInverse.mul(globalMatrix, tempLocalMatrix)
            setMatrix(id, tempLocalMatrix)
        }
    }

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
        markDirty(id)
    }

    // ==========================================
    // 👇 这次补全了主人缺失的方法！！👇
    // ==========================================

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
