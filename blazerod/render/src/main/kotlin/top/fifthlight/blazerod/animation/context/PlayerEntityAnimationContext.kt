package top.fifthlight.blazerod.animation.context

import net.minecraft.client.option.Perspective
import net.minecraft.component.DataComponentTypes
import net.minecraft.entity.LivingEntity
import net.minecraft.entity.player.PlayerEntity
import net.minecraft.util.Identifier
import net.minecraft.util.math.MathHelper
import top.fifthlight.blazerod.model.animation.AnimationContext
import top.fifthlight.blazerod.model.animation.AnimationContext.Property.*
import top.fifthlight.blazerod.model.animation.AnimationContext.RenderingTargetType
import top.fifthlight.blazerod.util.ObjectPool
import kotlin.math.abs

open class PlayerEntityAnimationContext protected constructor() : LivingEntityAnimationContext(), AutoCloseable {
    companion object {
        private val POOL = ObjectPool(
            identifier = Identifier.of("blazerod", "player_entity_animation_context"),
            create = ::PlayerEntityAnimationContext,
            onReleased = {
                released = false
                realPlayerEntity = null
            },
            onClosed = { },
        )

        fun acquire(entity: PlayerEntity) = POOL.acquire().apply {
            realPlayerEntity = entity
        }

        fun <T> with(entity: PlayerEntity, block: (PlayerEntityAnimationContext) -> T) =
            acquire(entity).use { block(it) }

        @JvmStatic
        protected val propertyTypes = EntityAnimationContext.propertyTypes + listOf(
            RenderTarget,
            PlayerHeadXRotation,
            PlayerHeadYRotation,
            PlayerIsFirstPerson,
            PlayerPersonView,
            PlayerIsSpectator,
            PlayerIsSneaking,
            PlayerIsSprinting,
            PlayerIsSwimming,
            PlayerBodyXRotation,
            PlayerBodyYRotation,
            PlayerIsEating,
            PlayerIsUsingItem,
            PlayerLevel,
            PlayerIsJumping,
            PlayerIsSleeping,
        )
            @JvmName("getPlayerEntityPropertyTypes")
            get
    }

    override fun close() {
        if (released) {
            return
        }
        released = true
        POOL.release(this)
    }

    protected var realPlayerEntity: PlayerEntity? = null
        set(value) {
            realEntity = value
            field = value
        }
    override val entity: PlayerEntity
        get() = realPlayerEntity ?: throw IllegalStateException("Entity is null")

    private fun clampBodyYaw(entity: LivingEntity, degrees: Float, tickProgress: Float): Float {
        if (entity.vehicle is LivingEntity) {
            var f = MathHelper.lerpAngleDegrees(tickProgress, entity.lastBodyYaw, entity.bodyYaw)
            val g = 85.0f
            val h = MathHelper.clamp(MathHelper.wrapDegrees(degrees - f), -g, g)
            f = degrees - h
            if (abs(h) > 50.0f) {
                f += h * 0.2f
            }

            return f
        } else {
            return MathHelper.lerpAngleDegrees(tickProgress, entity.lastBodyYaw, entity.bodyYaw)
        }
    }

    @Suppress("UNCHECKED_CAST")
    override fun <T> getProperty(type: AnimationContext.Property<T>): T? = when (type) {
        RenderTarget -> RenderingTargetType.PLAYER

        PlayerHeadXRotation -> floatBuffer.apply {
            val rawBodyYaw = MathHelper.lerpAngleDegrees(getDeltaTick(), entity.lastHeadYaw, entity.headYaw)
            val bodyYaw = clampBodyYaw(entity, rawBodyYaw, getDeltaTick())
            value = -MathHelper.wrapDegrees(rawBodyYaw - bodyYaw)
        }

        PlayerHeadYRotation -> floatBuffer.apply {
            value = entity.getLerpedPitch(getDeltaTick())
        }

        PlayerIsFirstPerson -> booleanBuffer.apply {
            val isSelf = entity == client.player
            val isFirstPerson = client.options.perspective == Perspective.FIRST_PERSON
            value = isSelf && isFirstPerson
        }

        PlayerPersonView -> intBuffer.apply {
            val isSelf = entity == client.player
            val perspective = client.options.perspective
            value = when {
                !isSelf -> 1
                perspective == Perspective.FIRST_PERSON -> 0
                perspective == Perspective.THIRD_PERSON_BACK -> 1
                perspective == Perspective.THIRD_PERSON_FRONT -> 2
                else -> 0
            }
        }

        PlayerIsSpectator -> booleanBuffer.apply { value = entity.isSpectator }

        PlayerIsSneaking -> booleanBuffer.apply { value = entity.isSneaking }

        PlayerIsSprinting -> booleanBuffer.apply { value = entity.isSprinting }

        PlayerIsSwimming -> booleanBuffer.apply { value = entity.isSwimming }

        PlayerBodyXRotation -> floatBuffer.apply {
            val rawBodyYaw = MathHelper.lerpAngleDegrees(getDeltaTick(), entity.lastHeadYaw, entity.headYaw)
            val bodyYaw = clampBodyYaw(entity, rawBodyYaw, getDeltaTick())
            value = -bodyYaw
        }

        PlayerBodyYRotation -> floatBuffer.apply { value = 0f }

        PlayerIsEating -> booleanBuffer.apply {
            val isUsingItem = entity.isUsingItem
            val usingItemHasConsumingComponent = entity.activeItem.components.get(DataComponentTypes.CONSUMABLE) != null
            value = isUsingItem && usingItemHasConsumingComponent
        }

        PlayerIsUsingItem -> booleanBuffer.apply { value = entity.isUsingItem }

        PlayerIsJumping -> booleanBuffer.apply { value = entity.isJumping }

        PlayerIsSleeping -> booleanBuffer.apply { value = entity.isSleeping }

        PlayerLevel -> intBuffer.apply { value = entity.experienceLevel }

        PlayerFoodLevel -> intBuffer.apply {
            value = entity.hungerManager.foodLevel
        }

        else -> super.getProperty(type)
    } as T?

    override fun getPropertyTypes() = propertyTypes
}