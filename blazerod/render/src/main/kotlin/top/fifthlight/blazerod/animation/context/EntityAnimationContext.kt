package top.fifthlight.blazerod.animation.context

import net.minecraft.entity.Entity
import net.minecraft.util.Identifier
import net.minecraft.util.math.Direction
import org.joml.Math.sqrt
import top.fifthlight.blazerod.model.animation.AnimationContext
import top.fifthlight.blazerod.model.animation.AnimationContext.Property.*
import top.fifthlight.blazerod.model.animation.AnimationContext.RenderingTargetType
import top.fifthlight.blazerod.util.ObjectPool
import top.fifthlight.blazerod.util.set
import top.fifthlight.blazerod.util.sub

open class EntityAnimationContext protected constructor() : BaseAnimationContext(), AutoCloseable {
    companion object {
        private val POOL = ObjectPool(
            identifier = Identifier.of("blazerod", "entity_animation_context"),
            create = ::EntityAnimationContext,
            onReleased = {
                released = false
                realEntity = null
            },
            onClosed = { },
        )

        fun acquire(entity: Entity) = POOL.acquire().apply {
            realEntity = entity
        }

        fun <T> with(entity: Entity, block: (EntityAnimationContext) -> T) =
            acquire(entity).use { block(it) }

        @JvmStatic
        protected val propertyTypes = BaseAnimationContext.propertyTypes + setOf(
            RenderTarget,
            EntityPosition,
            EntityPositionDelta,
            EntityHorizontalFacing,
            EntityGroundSpeed,
            EntityVerticalSpeed,
            EntityHasRider,
            EntityIsRiding,
            EntityIsInWater,
            EntityIsInWaterOrRain,
            EntityIsInFire,
            EntityIsOnGround,
        )
            @JvmName("getEntityPropertyTypes")
            get
    }

    protected var released = false
    protected var realEntity: Entity? = null
    protected open val entity: Entity
        get() = realEntity ?: throw IllegalStateException("Entity is null")

    override fun close() {
        if (released) {
            return
        }
        released = true
        POOL.release(this)
    }

    @Suppress("UNCHECKED_CAST")
    override fun <T> getProperty(type: AnimationContext.Property<T>): T? = when (type) {
        RenderTarget -> RenderingTargetType.ENTITY

        EntityPosition -> vector3dBuffer.set(entity.pos)

        EntityPositionDelta -> entity.pos.sub(entity.lastRenderPos, vector3dBuffer)

        EntityHorizontalFacing -> when (entity.horizontalFacing) {
            Direction.NORTH -> 2
            Direction.SOUTH -> 3
            Direction.WEST -> 4
            Direction.EAST -> 5
            Direction.UP, Direction.DOWN -> throw AssertionError("Invalid cardinal facing")
        }.let { intBuffer.apply { value = it } }

        EntityGroundSpeed -> doubleBuffer.apply {
            value = sqrt(entity.movement.x * entity.movement.x + entity.movement.z * entity.movement.z)
        }

        EntityVerticalSpeed -> doubleBuffer.apply { value = entity.movement.y }

        EntityHasRider -> booleanBuffer.apply { value = entity.hasPassengers() }

        EntityIsRiding -> booleanBuffer.apply { value = entity.hasVehicle() }

        EntityIsInWater -> booleanBuffer.apply { value = entity.isTouchingWater }

        EntityIsInWaterOrRain -> booleanBuffer.apply { value = entity.isTouchingWaterOrRain }

        EntityIsInFire -> booleanBuffer.apply { value = entity.isOnFire }

        EntityIsOnGround -> booleanBuffer.apply { value = entity.isOnGround }

        else -> super.getProperty(type)
    } as T?

    override fun getPropertyTypes() = propertyTypes
}