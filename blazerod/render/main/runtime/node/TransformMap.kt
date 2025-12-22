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
 * (✨ω✨) 主人的终极 TransformMap
 * 修复了：
 * 1. 脏标记传播：自动构建父子关系图，不再依赖 Enum 顺序。
 * 2. 坐标空间问题：增加了 setGlobalMatrix，自动处理世界转局部。
 */
class TransformMap(
    first: NodeTransformView?,
    private val parentProvider: (TransformId) -> TransformId?
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

    // (NEW) 子节点查找表：用于正确的脏标记传播
    private val childrenMap = EnumMap<TransformId, MutableList<TransformId>>(TransformId::class.java)

    init {
        // 初始化时，扫描所有枚举，建立“谁是父亲 -> 有哪些孩子”的查找表
        // 这样 markDirty 就可以精准打击啦！
        for (id in TransformId.entries) {
            val parent = if (id == TransformId.FIRST) null else parentProvider(id)
            if (parent != null) {
                childrenMap.getOrPut(parent) { ArrayList() }.add(id)
            }
        }
    }

    // 递归标记脏数据 (不再依赖 Enum 顺序)
    private fun markDirty(id: TransformId) {
        // 如果已经脏了，就不用继续往下跑了，节省性能 (剪枝)
        if (!dirtyTransforms.add(id)) return
        
        // 找到所有的孩子，继续把它们标记为脏
        childrenMap[id]?.forEach { child ->
            markDirty(child)
        }
    }

    // ... clearFrom 等方法保持原样，或者用 markDirty 替代 ...
    fun clearFrom(id: TransformId = TransformId.FIRST) {
        transforms.keys.removeIf { it >= id } // 这里的清理逻辑可能还要看业务需求，暂时保留
        intermediateMatrices.keys.removeIf { it >= id }
        markDirty(id)
    }

    private val tempAccumulatedMatrix = Matrix4f()
    private val pathStack = ArrayList<TransformId>()

    // 计算逻辑保持之前的树形回溯，这是对的！
    private fun calculateIntermediateMatrices(targetId: TransformId): Matrix4fc {
        pathStack.clear()
        var currentTracer: TransformId? = targetId

        // 1. 回溯找干净的祖先
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

        // 2. 初始化基准
        if (currentTracer != null) {
            tempAccumulatedMatrix.set(intermediateMatrices[currentTracer]!!)
        } else {
            // 如果追溯到根节点且根节点也脏，或者没有缓存
             if (pathStack.isNotEmpty() && pathStack.last() == TransformId.FIRST) {
               transforms[TransformId.FIRST]!!.setOnMatrix(tempAccumulatedMatrix)
               pathStack.removeAt(pathStack.lastIndex)
            } else {
                tempAccumulatedMatrix.identity()
            }
        }

        // 3. 向下累积
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
    // (✨重要) 新增：设置全局矩阵 (防扭曲的核心！)
    // ==========================================
    private val tempParentInverse = Matrix4f()
    private val tempLocalMatrix = Matrix4f()

    /**
     * 设置指定节点的【世界空间】变换。
     * 就算你给我的是世界坐标，我也能自动把它转成相对于父节点的局部坐标存起来！
     * 这样父节点动的时候，它就会乖乖跟着动，而不会扭曲啦！
     */
    fun setGlobalMatrix(id: TransformId, globalMatrix: Matrix4fc) {
        val parent = if (id == TransformId.FIRST) null else parentProvider(id)

        if (parent == null) {
            // 没有父节点，全局就是局部
            setMatrix(id, globalMatrix)
        } else {
            // 有父节点：局部 = 父节点逆矩阵 * 全局
            // 1. 获取父节点最新的全局矩阵 (getSum 会自动处理脏标记，保证拿到最新的)
            val parentGlobal = getSum(parent)
            
            // 2. 计算父节点的逆矩阵
            parentGlobal.invert(tempParentInverse)
            
            // 3. 计算局部矩阵：ParentInverse * TargetGlobal
            tempParentInverse.mul(globalMatrix, tempLocalMatrix)
            
            // 4. 存进去！
            setMatrix(id, tempLocalMatrix)
        }
    }

    // ... 原有的 updateDecomposed, updateMatrix 等方法 ...
    // ... 记得把里面的 markDirty(id) 改成调用上面新写的递归版 markDirty(id) ...

    fun updateDecomposed(id: TransformId, updater: NodeTransform.Decomposed.() -> Unit) {
        // ... (保持原有逻辑获取 targetTransform) ...
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
        markDirty(id) // <--- 这里现在是递归的了！
    }

    fun setMatrix(id: TransformId, matrix: Matrix4fc) {
        // ... (保持原有逻辑) ...
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
        markDirty(id) // <--- 递归标记！
    }
    
    // 省略 updateMatrix, updateBedrock, setMatrix(Decomposed) 以节省篇幅
    // 只要记住把里面的 markDirty(id) 调用都指向新的递归方法即可！
}
