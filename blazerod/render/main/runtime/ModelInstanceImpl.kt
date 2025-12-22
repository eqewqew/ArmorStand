package top.fifthlight.blazerod.runtime

import net.minecraft.client.render.VertexConsumerProvider
import org.joml.Matrix4f
import org.joml.Matrix4fc
import top.fifthlight.blazerod.api.refcount.AbstractRefCount
import top.fifthlight.blazerod.api.resource.ModelInstance
import top.fifthlight.blazerod.api.resource.RenderScene
import top.fifthlight.blazerod.model.NodeTransform
import top.fifthlight.blazerod.model.NodeTransformView
import top.fifthlight.blazerod.model.TransformId
import top.fifthlight.blazerod.runtime.data.LocalMatricesBuffer
import top.fifthlight.blazerod.runtime.data.MorphTargetBuffer
import top.fifthlight.blazerod.runtime.data.RenderSkinBuffer
import top.fifthlight.blazerod.runtime.node.RenderNodeImpl
import top.fifthlight.blazerod.runtime.node.TransformMap
import top.fifthlight.blazerod.runtime.node.UpdatePhase
import top.fifthlight.blazerod.runtime.node.markNodeTransformDirty
import top.fifthlight.blazerod.runtime.resource.CameraTransformImpl
import top.fifthlight.blazerod.util.cowbuffer.CowBuffer
import top.fifthlight.blazerod.util.cowbuffer.copy
import top.fifthlight.blazerod.util.iterator.mapToArray
import top.fifthlight.mergetools.api.ActualConstructor
import top.fifthlight.mergetools.api.ActualImpl
import java.util.function.Consumer

@ActualImpl(ModelInstance::class)
class ModelInstanceImpl(
    override val scene: RenderSceneImpl,
) : AbstractRefCount(), ModelInstance {
    @ActualConstructor("of")
    constructor(scene: RenderScene) : this(scene as RenderSceneImpl)

    override val typeId: String
        get() = "model_instance"

    val modelData = ModelData(scene)

    init {
        scene.increaseReferenceCount()
    }

    class ModelData(scene: RenderSceneImpl) : AutoCloseable {
        var undirtyNodeCount = 0

        // (✨修复重点) 这里注入了父子关系逻辑！
        // 告诉 TransformMap 真正的骨骼层级，防止腿部扭曲！
        val transformMaps = scene.nodes.mapToArray { node ->
            TransformMap(node.absoluteTransform) { id ->
                // 1. 定义一个安全的查找函数，找不到也不报错
                fun find(name: String): TransformId? = TransformId.entries.find { it.name == name }

                // 2. 根据名字匹配父节点 (标准骨骼结构)
                val parent = when (id.name) {
                    // --- 躯干 (Torso) ---
                    "HEAD" -> find("NECK") ?: find("UPPER_CHEST") ?: find("BODY")
                    "NECK" -> find("UPPER_CHEST") ?: find("CHEST") ?: find("BODY")
                    "UPPER_CHEST" -> find("CHEST") ?: find("SPINE")
                    "CHEST" -> find("SPINE") ?: find("HIPS") ?: find("BODY")
                    "SPINE" -> find("HIPS") ?: find("BODY")
                    "HIPS" -> find("FIRST") // 屁股通常连着根节点
                    "BODY" -> find("FIRST") // 或者 BODY 连着 FIRST

                    // --- 左腿 (Left Leg) [防扭曲关键点！] ---
                    "LEFT_TOES" -> find("LEFT_FOOT")
                    "LEFT_FOOT" -> find("LEFT_LOWER_LEG") ?: find("LEFT_CALF") ?: find("LEG_L_LOWER")
                    "LEFT_LOWER_LEG", "LEFT_CALF", "LEG_L_LOWER" -> 
                        find("LEFT_UPPER_LEG") ?: find("LEFT_THIGH") ?: find("LEG_L_UPPER")
                    "LEFT_UPPER_LEG", "LEFT_THIGH", "LEG_L_UPPER" -> 
                        find("HIPS") ?: find("BODY") // <--- 确保大腿连着屁股！

                    // --- 右腿 (Right Leg) ---
                    "RIGHT_TOES" -> find("RIGHT_FOOT")
                    "RIGHT_FOOT" -> find("RIGHT_LOWER_LEG") ?: find("RIGHT_CALF") ?: find("LEG_R_LOWER")
                    "RIGHT_LOWER_LEG", "RIGHT_CALF", "LEG_R_LOWER" -> 
                        find("RIGHT_UPPER_LEG") ?: find("RIGHT_THIGH") ?: find("LEG_R_UPPER")
                    "RIGHT_UPPER_LEG", "RIGHT_THIGH", "LEG_R_UPPER" -> 
                        find("HIPS") ?: find("BODY")

                    // --- 左臂 (Left Arm) ---
                    "LEFT_HAND" -> find("LEFT_LOWER_ARM")
                    "LEFT_LOWER_ARM" -> find("LEFT_UPPER_ARM")
                    "LEFT_UPPER_ARM" -> find("UPPER_CHEST") ?: find("CHEST")

                    // --- 右臂 (Right Arm) ---
                    "RIGHT_HAND" -> find("RIGHT_LOWER_ARM")
                    "RIGHT_LOWER_ARM" -> find("RIGHT_UPPER_ARM")
                    "RIGHT_UPPER_ARM" -> find("UPPER_CHEST") ?: find("CHEST")

                    // --- 兜底 ---
                    else -> null
                }

                // 3. 如果找到了父节点就返回，找不到就只能连到 FIRST 防止断链
                parent ?: if (id.name == "FIRST") null else TransformId.FIRST
            }
        }

        val transformDirty = Array(scene.nodes.size) { true }

        val worldTransforms = Array(scene.nodes.size) { Matrix4f() }

        val localMatricesBuffer = run {
            val buffer = LocalMatricesBuffer(scene.primitiveComponents.size)
            buffer.clear()
            CowBuffer.acquire(buffer).also { it.increaseReferenceCount() }
        }

        val skinBuffers = scene.skins.mapIndexed { index, skin ->
            val skinBuffer = RenderSkinBuffer(skin.jointSize)
            skinBuffer.clear()
            CowBuffer.acquire(skinBuffer).also { it.increaseReferenceCount() }
        }

        val targetBuffers = scene.morphedPrimitiveComponents.mapIndexed { index, component ->
            val primitive = component.primitive
            val targets = primitive.targets!!
            val targetBuffers = MorphTargetBuffer(
                positionTargets = targets.position.targetsCount,
                colorTargets = targets.color.targetsCount,
                texCoordTargets = targets.texCoord.targetsCount,
            )
            for (targetGroup in primitive.targetGroups) {
                fun processGroup(index: Int?, channel: MorphTargetBuffer.WeightChannel, weight: Float) =
                    index?.let {
                        channel[index] = weight
                    }
                processGroup(targetGroup.position, targetBuffers.positionChannel, targetGroup.weight)
                processGroup(targetGroup.color, targetBuffers.colorChannel, targetGroup.weight)
                processGroup(targetGroup.texCoord, targetBuffers.texCoordChannel, targetGroup.weight)
            }
            CowBuffer.acquire(targetBuffers).also { it.increaseReferenceCount() }
        }

        val cameraTransforms = scene.cameras.map { CameraTransformImpl.of(it) }

        val ikEnabled = Array(scene.ikTargetData.size) { true }

        override fun close() {
            localMatricesBuffer.decreaseReferenceCount()
            skinBuffers.forEach { it.decreaseReferenceCount() }
            targetBuffers.forEach { it.decreaseReferenceCount() }
        }
    }

    override fun clearTransform() {
        modelData.undirtyNodeCount = 0
        for (i in scene.nodes.indices) {
            modelData.transformMaps[i].clearFrom(TransformId.ABSOLUTE.next)
            modelData.transformDirty[i] = true
        }
    }

    override fun setTransformMatrix(nodeIndex: Int, transformId: TransformId, matrix: Matrix4f) {
        markNodeTransformDirty(scene.nodes[nodeIndex])
        val transform = modelData.transformMaps[nodeIndex]
        // (💡小贴士) 如果是 IK 算出的世界坐标，建议用 transform.setGlobalMatrix(transformId, matrix)
        // 不过这里是通用接口，保持 setMatrix 也可以，只要外部传入的是局部坐标就行。
        transform.setMatrix(transformId, matrix)
    }

    override fun setTransformDecomposed(
        nodeIndex: Int,
        transformId: TransformId,
        decomposed: NodeTransformView.Decomposed,
    ) {
        markNodeTransformDirty(scene.nodes[nodeIndex])
        val transform = modelData.transformMaps[nodeIndex]
        transform.setMatrix(transformId, decomposed)
    }

    override fun setTransformDecomposed(
        nodeIndex: Int,
        transformId: TransformId,
        updater: Consumer<NodeTransform.Decomposed>,
    ) =
        setTransformDecomposed(nodeIndex, transformId) { updater.accept(this) }

    override fun setTransformDecomposed(
        nodeIndex: Int,
        transformId: TransformId,
        updater: NodeTransform.Decomposed.() -> Unit,
    ) {
        markNodeTransformDirty(scene.nodes[nodeIndex])
        val transform = modelData.transformMaps[nodeIndex]
        transform.updateDecomposed(transformId, updater)
    }

    override fun setTransformBedrock(
        nodeIndex: Int,
        transformId: TransformId,
        updater: NodeTransform.Bedrock.() -> Unit,
    ) {
        markNodeTransformDirty(scene.nodes[nodeIndex])
        val transform = modelData.transformMaps[nodeIndex]
        transform.updateBedrock(transformId, updater)
    }

    override fun getIkEnabled(index: Int) = modelData.ikEnabled[index]

    override fun setIkEnabled(index: Int, enabled: Boolean) {
        val prevEnabled = modelData.ikEnabled[index]
        modelData.ikEnabled[index] = enabled
        if (prevEnabled && !enabled) {
            val component = scene.ikTargetComponents[index]
            for (chain in component.chains) {
                markNodeTransformDirty(scene.nodes[chain.nodeIndex])
                val transform = modelData.transformMaps[chain.nodeIndex]
                transform.clearFrom(component.transformId)
            }
        }
    }

    override fun setGroupWeight(morphedPrimitiveIndex: Int, targetGroupIndex: Int, weight: Float) {
        val primitiveComponent = scene.morphedPrimitiveComponents[morphedPrimitiveIndex]
        val group = primitiveComponent.primitive.targetGroups[targetGroupIndex]
        val weightsIndex = requireNotNull(primitiveComponent.morphedPrimitiveIndex) {
            "Component $primitiveComponent don't have target? Check model loader"
        }
        val weights = modelData.targetBuffers[weightsIndex]
        weights.edit {
            group.position?.let { positionChannel[it] = weight }
            group.color?.let { colorChannel[it] = weight }
            group.texCoord?.let { texCoordChannel[it] = weight }
        }
    }

    override fun getCameraTransform(index: Int) = modelData.cameraTransforms.getOrNull(index)

    override fun updateCamera() {
        scene.updateCamera(this)
    }

    override fun debugRender(viewProjectionMatrix: Matrix4fc, consumers: VertexConsumerProvider) {
        scene.debugRender(this, viewProjectionMatrix, consumers)
    }

    override fun updateRenderData() {
        scene.updateRenderData(this)
    }

    internal fun updateNodeTransform(nodeIndex: Int) {
        val node = scene.nodes[nodeIndex]
        updateNodeTransform(node)
    }

    internal fun updateNodeTransform(node: RenderNodeImpl) {
        if (modelData.undirtyNodeCount == scene.nodes.size) {
            return
        }
        node.update(UpdatePhase.GlobalTransformPropagation, node, this)
        for (child in node.children) {
            updateNodeTransform(child)
        }
    }

    override fun createRenderTask(
        modelMatrix: Matrix4fc,
        light: Int,
        overlay: Int,
    ): RenderTaskImpl {
        return RenderTaskImpl.acquire(
            instance = this,
            modelMatrix = modelMatrix,
            light = light,
            overlay = overlay,
            localMatricesBuffer = modelData.localMatricesBuffer.copy(),
            skinBuffer = modelData.skinBuffers.copy(),
            morphTargetBuffer = modelData.targetBuffers.copy().also { buffer ->
                // Upload indices don't change the actual data
                buffer.forEach {
                    it.content.uploadIndices()
                }
            },
        ).apply {
            scene.renderTransform?.matrix?.let {
                this.modelMatrix.mul(it)
            }
        }
    }

    override fun onClosed() {
        scene.decreaseReferenceCount()
        modelData.close()
    }
}
