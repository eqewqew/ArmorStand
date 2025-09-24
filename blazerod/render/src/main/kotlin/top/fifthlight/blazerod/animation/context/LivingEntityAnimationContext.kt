package top.fifthlight.blazerod.animation.context

import net.minecraft.entity.EquipmentSlot
import net.minecraft.entity.LivingEntity
import net.minecraft.util.Identifier
import top.fifthlight.blazerod.model.animation.AnimationContext
import top.fifthlight.blazerod.model.animation.AnimationContext.Property.*
import top.fifthlight.blazerod.util.ObjectPool

open class LivingEntityAnimationContext protected constructor() : EntityAnimationContext(), AutoCloseable {
    companion object {
        private val POOL = ObjectPool(
            identifier = Identifier.of("blazerod", "living_entity_animation_context"),
            create = ::LivingEntityAnimationContext,
            onReleased = {
                released = false
                realLivingEntity = null
            },
            onClosed = { },
        )

        fun acquire(entity: LivingEntity) = POOL.acquire().apply {
            realLivingEntity = entity
        }

        fun <T> with(entity: LivingEntity, block: (LivingEntityAnimationContext) -> T) =
            acquire(entity).use { block(it) }

        @JvmStatic
        protected val propertyTypes = EntityAnimationContext.propertyTypes + setOf(
            LivingEntityHealth,
            LivingEntityMaxHealth,
            LivingEntityHurtTime,
            LivingEntityIsDead,
            LivingEntityEquipmentCount,
        )
            @JvmName("getLivingEntityPropertyTypes")
            get
    }

    override fun close() {
        if (released) {
            return
        }
        released = true
        POOL.release(this)
    }

    protected var realLivingEntity: LivingEntity? = null
        set(value) {
            realEntity = value
            field = value
        }
    override val entity: LivingEntity
        get() = realLivingEntity ?: throw IllegalStateException("Entity is null")

    @Suppress("UNCHECKED_CAST")
    override fun <T> getProperty(type: AnimationContext.Property<T>): T? = when (type) {
        LivingEntityHealth -> floatBuffer.apply { value = entity.health }

        LivingEntityMaxHealth -> floatBuffer.apply { value = entity.maxHealth }

        LivingEntityHurtTime -> intBuffer.apply { value = entity.hurtTime }

        LivingEntityIsDead -> booleanBuffer.apply { value = !entity.isAlive }

        LivingEntityEquipmentCount -> intBuffer.apply {
            value = EquipmentSlot.entries.count { entity.hasStackEquipped(it) }
        }

        else -> super.getProperty(type)
    } as T?

    override fun getPropertyTypes() = propertyTypes
}