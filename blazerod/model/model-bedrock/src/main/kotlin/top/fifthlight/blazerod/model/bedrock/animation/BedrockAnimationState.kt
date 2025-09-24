package top.fifthlight.blazerod.model.bedrock.animation

import org.slf4j.LoggerFactory
import team.unnamed.mocha.MochaEngine
import team.unnamed.mocha.parser.ast.Expression
import team.unnamed.mocha.runtime.binding.JavaObjectBinding
import team.unnamed.mocha.runtime.standard.MochaMath
import team.unnamed.mocha.runtime.value.MutableObjectBinding
import top.fifthlight.blazerod.model.animation.AnimationContext
import top.fifthlight.blazerod.model.animation.AnimationState
import top.fifthlight.blazerod.model.bedrock.molang.context.QueryContext
import top.fifthlight.blazerod.model.bedrock.molang.context.YsmContext
import top.fifthlight.blazerod.model.bedrock.molang.value.MolangValue


class BedrockAnimationState(
    val context: AnimationContext,
    duration: Float,
    startDelay: MolangValue,
    loopDelay: MolangValue,
    loopMode: AnimationLoopMode,
    animTimeUpdate: BedrockAnimation.AnimationTimeUpdate,
) : AnimationState {
    companion object {
        @Suppress("UnstableApiUsage")
        private val mathBinding = JavaObjectBinding.of(MochaMath::class.java, null, MochaMath())

        private val logger = LoggerFactory.getLogger(BedrockAnimationState::class.java)
    }

    private val engine = MochaEngine.create<AnimationContext?>(null) { builder ->
        builder.set("math", mathBinding)
        val variableBinding = MutableObjectBinding()
        builder.set("variable", variableBinding)
        builder.set("v", variableBinding)
        builder.set("query", QueryContext)
        builder.set("ysm", YsmContext)
    }

    fun evalExpressions(context: AnimationContext, expressions: List<Expression>): Double {
        try {
            engine.setEntity(context)
            return engine.eval(expressions)
        } catch (ex: Throwable) {
            logger.error("Error evaluating expressions", ex)
            return 0.0
        } finally {
            engine.setEntity(null)
        }
    }

    override fun updateTime(context: AnimationContext) {
        // TODO
    }

    // TODO
    override fun getTime() = 0f
}