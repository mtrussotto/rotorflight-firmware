#include "platform.h"

#ifdef USE_TELEMETRY_CASTLE

#include "build/atomic.h"
#include "build/dprintf.h"

#include "drivers/castle_telemetry_decode.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"

typedef struct castleInterrupt_s {
    timerCCHandlerRec_t pwmEdgeCb;
    timerChannel_t* timer;
    IO_t io;
    timCCR_t outputEnableTime;
    timCCR_t resetValue;
    timCCR_t saveCCR;
    castleTelemetry_t telem[2];
    uint8_t whichTelem; // telem we are writing to (0 or 1).
    uint8_t telemIndex; // where in the telem struct are we?
    uint8_t directChannel;
    uint8_t timingChannel;
} castleInterrupt_t;

static FAST_DATA_ZERO_INIT castleInterrupt_t castleState;

void getCastleTelemetry(castleTelemetry_t* telem) {
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        memcpy(telem, &castleState.telem[castleState.whichTelem^1], sizeof(castleTelemetry_t));
    }
    // We're downcounting, so the telemetry values need to be complemented.
    for (size_t i = 1; i < (sizeof(castleTelemetry_t) >> 1); i++) {
        ((uint16_t*)telem)[i] = castleState.resetValue - ((uint16_t*)telem)[i];
    }
}

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    castleInterrupt_t* state = container_of(cbRec, castleInterrupt_t, pwmEdgeCb);
    // Castle uses an inverted (active low) PWM pulse and between pulses, the ESC will quickly
    // pull the line down and then release it (they call it a 'tick').  The timing of the tick
    // starting from the end of the PWM pulse is the telemetry value.
    //
    // The timing of the interrupts are not critical; all sensitive timing is done by the timer.
    // The only requirements on the interrupt is the PWM-end interrupt must be serviced before 0.5ms
    // after the end of the pulse, and the output-on interrupt must be serviced before the next counter
    // reset.

    static int count = 52;
    static int count2 = 52;
    uint16_t counter = LL_TIM_GetCounter(state->timer->tim);
    if (counter <= state->outputEnableTime) {
        uint32_t pinIsHigh = IORead(state->io);
        // About 1 sec, but prime.
        if (++count == 53) {
            dprintf("PWM interrupt 1, tick = %d, capture = %d counter = %d, pin %d\r\n", *timerCCR(state->timer->tim, state->directChannel), capture, counter, pinIsHigh);
            count = 0;
        }
        uint16_t telemVal = *timerCCR(state->timer->tim, state->directChannel);
        if (!pinIsHigh) {
            // If the pin is low at output enable time while in
            // open-drain mode, the pull-up is not working.  Assuming the
            // wiring is correct, the battery may be disconnected.  We
            // should ignore any "telemetry" in this case, it's just noise.
            // Set telemIndex to 0 to wait for a sync frame.
            state->telemIndex = 0;
        } else if (telemVal < counter) {
            // No capture occurred, so this was a sync frame.
            state->telemIndex = 1;
        } else if (state->telemIndex > 0) {
            ((uint16_t*)&state->telem[state->whichTelem])[state->telemIndex] = telemVal;
            if (++state->telemIndex == CASTLE_TELEM_NFRAMES) {
                state->telemIndex = 0;
                // Note the first valid telemetry generation is 1.
                state->telem[state->whichTelem^1].generation =
                    ++state->telem[state->whichTelem].generation;
                dprintf("T%d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\r\n", state->whichTelem,
                        state->telem[state->whichTelem].generation,
                        state->telem[state->whichTelem].oneMs,
                        state->telem[state->whichTelem].battVoltage,
                        state->telem[state->whichTelem].rippleVoltage,
                        state->telem[state->whichTelem].battCurrent,
                        state->telem[state->whichTelem].throttle,
                        state->telem[state->whichTelem].outputPower,
                        state->telem[state->whichTelem].rpm,
                        state->telem[state->whichTelem].becVoltage,
                        state->telem[state->whichTelem].becCurrent,
                        state->telem[state->whichTelem].linTempOrHalfMs,
                        state->telem[state->whichTelem].ntcTempOrHalfMs
                       );
                state->whichTelem ^= 1;
            }
        }
        // Reconfigure the channel to generate the next PWM pulse
        LL_TIM_CC_DisableChannel(state->timer->tim, timerLLChannel(state->directChannel));
        LL_TIM_OC_ConfigOutput(state->timer->tim, timerLLChannel(state->directChannel), LL_TIM_OCPOLARITY_LOW | LL_TIM_OCIDLESTATE_HIGH);
        LL_TIM_OC_SetMode(state->timer->tim, timerLLChannel(state->directChannel), LL_TIM_OCMODE_PWM1);
        // Switch the motor control register back to the real register.
        // This must be done before enabling preload, or the output will
        // switch on based on the capture value.
        state->timer->ccr = timerCCR(state->timer->tim, state->directChannel);
        *state->timer->ccr = state->saveCCR;
        LL_TIM_OC_EnablePreload(state->timer->tim, timerLLChannel(state->directChannel));
        LL_TIM_CC_EnableChannel(state->timer->tim, timerLLChannel(state->directChannel));
        // Switch to push-pull output, so the falling edge of the next pulse is clean.
        LL_GPIO_SetPinOutputType(IO_GPIO(state->io), IO_Pin(state->io), LL_GPIO_OUTPUT_PUSHPULL);
        *timerCCR(state->timer->tim, state->timingChannel) = state->resetValue;
    } else {
        if (++count2 == 53) {
            dprintf("PWM interrupt, pulseEndTime = %d ccr = %d, ccrhi %d\r\n", capture, *timerCCR(state->timer->tim, state->directChannel), *timerCCR(state->timer->tim, state->timingChannel));
            count2 = 0;
        }
        // Save the PWM CCR value from the shadow register and reconfigure
        // motor control to point to the saveCCR field.
        volatile timCCR_t* directCCR = timerCCR(state->timer->tim, state->directChannel);
        state->saveCCR = *directCCR;
        state->timer->ccr = &state->saveCCR;

        // Switch to Open Drain.  The internal pull-ups are not nearly
        // strong enough, so they are not used.  Instead, an external
        // pull-up to the ESC power needs to be used.  The esc has a
        // pull-down of 6.65k (according to spec), use a pull-up to make
        // the resulting voltage at least as high as the MCU V_Ih value
        // (in the datasheet), but in no case more than the MCU tolerance
        // value for the pin.  The pin used on the Radiomaster Nexus is 5V
        // tolerant, so 10K should work from 5.6V to 12.5V for a pull-up
        // voltage.  But MEASURE the resulting voltage BEFORE plugging it
        // in, I disclaim any responsibility for frying your flight
        // controller. If you're pulling up to a higher voltage, a 15k pull-down
        // (which should still work down to a pull-up voltage of 7.3V) is much cheaper than
        // a fried flight controller!  If you're running very low voltages, a 6.8k resistor
        // works from 4.5V to 9.8V.
        LL_GPIO_SetPinOutputType(IO_GPIO(state->io), IO_Pin(state->io), LL_GPIO_OUTPUT_OPENDRAIN);

        // Reconfigure the channel to record the falling edge of the
        // 'tick'.  We do not need to take an interrupt on capture; we
        // can record the edge when the interrupt to turn on output
        // happens.
        LL_TIM_CC_DisableChannel(state->timer->tim, timerLLChannel(state->directChannel));
        LL_TIM_IC_Config(state->timer->tim, timerLLChannel(state->directChannel), LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1 | LL_TIM_IC_POLARITY_FALLING);
        LL_TIM_CC_EnableChannel(state->timer->tim, timerLLChannel(state->directChannel));
        *timerCCR(state->timer->tim, state->timingChannel) = state->outputEnableTime;
    }
}

// Assumes timer is already set up for output.
bool castleInputConfig(const timerHardware_t* timerHardware,
                       timerChannel_t* timerChannel,
                       uint32_t hz) {
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if (Handle == NULL)
        return false;

    if (castleState.timer) {
        dprintf("Castle telemetry already configured, we only support one.\r\n");
        return false;
    }
    // Find an unassigned capture/compare register.
    uint8_t timingChannel = 0xFF;
    for (int8_t channelIndex = CC_CHANNELS_PER_TIMER - 1; channelIndex > 0; channelIndex--) {
        uint8_t channel = CC_CHANNEL_FROM_INDEX(channelIndex);
        if (!timerGetConfiguredByNumberAndChannel(timerGetTIMNumber(timerHardware->tim),
                channel)) {
            timingChannel = channel;
            break;
        }
    }
    if (timingChannel == 0xFF) {
        dprintf("No channels are unassigned (available) on the motor timer: TIM%d\r\n",
                timerGetTIMNumber(timerHardware->tim));
        return false;
    }
    dprintf("Configuring input for Castle telemetry, TIM%d, chans = %d, %d!\r\n",
            timerGetTIMNumber(timerHardware->tim), CC_INDEX_FROM_CHANNEL(timerHardware->channel), CC_INDEX_FROM_CHANNEL(timingChannel));
    castleState.timer = timerChannel;
    castleState.directChannel = timerHardware->channel;
    castleState.timingChannel = timingChannel;
    castleState.io = IOGetByTag(timerHardware->tag);
    // We set an interrupt to turn on the output 3ms before timer
    // reset.  Since the pulse is right-edge-aligned and the maximum
    // width is 2ms, that gives us 1ms to turn it on.
    uint16_t threeMs = (3 * hz) / 1000;
    castleState.outputEnableTime = threeMs;
    castleState.resetValue = __HAL_TIM_GET_AUTORELOAD(Handle);
    dprintf("threeMs = %d\r\n", castleState.outputEnableTime);

    // Initialize the timer compare register for when to turn off the output.
    TIM_OC_InitTypeDef TIM_OCInitStructure;
    // Initialize the other channel for timing.
    TIM_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
    TIM_OCInitStructure.Pulse = castleState.resetValue;

    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(Handle, &TIM_OCInitStructure, timingChannel);

    // Configure interrupts on the timing channel.  The main channel
    // does not need interrupts.
    timerHardware_t otherHardware = *timerHardware;
    otherHardware.channel = timingChannel;
    timerChCCHandlerInit(&castleState.pwmEdgeCb, pwmEdgeCallback);
    timerChConfigCallbacks(&otherHardware, &castleState.pwmEdgeCb, NULL);
    timerNVICConfigure(timerInputIrq(timerHardware->tim));
    HAL_TIM_OC_Start_IT(Handle, otherHardware.channel);
    return true;
}

#endif // USE_TELEMETRY_CASTLE
