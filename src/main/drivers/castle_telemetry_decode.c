#include <math.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/dprintf.h"

#include "drivers/castle_telemetry_decode.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"

typedef struct castleInterrupt_s {
    timerCCHandlerRec_t pwmEdgeCb;
    TIM_TypeDef* timer;
    IO_t io;
    timCCR_t nine;
    timCCR_t pulseEndTime;
    castleTelemetry_t telem[2];
    uint8_t whichTelem; // telem we are writing to (0 or 1).
    uint8_t telemIndex; // where in the telem struct are we?
    uint8_t directChannel;
    uint8_t indirectChannel;
} castleInterrupt_t;

static FAST_DATA_ZERO_INIT castleInterrupt_t castleState;

void getCastleTelemetry(castleTelemetry_t* telem) {
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        memcpy(telem, &castleState.telem[castleState.whichTelem^1], sizeof(castleTelemetry_t));
    }
}

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    castleInterrupt_t* state = container_of(cbRec, castleInterrupt_t, pwmEdgeCb);
    // This code makes some dubious assumptions
    // 1) The PWM out pin is assigned to the low channel (1 or 3)
    // 2) The high channel is available.
    // 3) The output IRQ is the same as the input IRQ
    // 4) Complementary channels are not enabled.
    // This is true for the Rotorflight Nexus (TIM4, Ch1 is used for the motor
    // and all other TIM4 channels are unassigned)
    // May not be true for other boards.
    // Castle uses an inverted (active low) PWM pulse and between pulses, the ESC will quickly
    // pull the line down and then release it (they call it a 'tick').  The timing of the tick
    // starting from the end of the PWM pulse is the telemetry value.
    //
    // The way the code works that as soon as the PWM pulse is done
    // (goes high/inactive), we flip the timer output pin driver to open
    // drain mode and freeze the output.  Then we use CCR1 to wait for
    // 9ms from timer start.  CCR2 captures the falling edge of the
    // 'tick'.  When CCR1 fires again, CCR2 (minus the original CCR1
    // value) contains the telemetry value we want. (If CCR2 hasn't changed, no tick occurred)
    //
    //
    // We also use CCR2 to capture the end of the PWM pulse.  We should know
    // how long it was from CCR1, but we can't get to the real CCR1; only the
    // preload register, which may have changed.  Yes, the MCU can measure its
    // own outputs.
    //
    // The timing of the interrupts are not critical; all sensitive timing is done by the timer.
    // The only requirements on the interrupt is the PWM-end interrupt must be serviced before 0.5ms
    // after the end of the pulse, and the 9ms interrupt must be serviced before the next counter
    // reset (this is at least 1ms at 100Hz).
    //
    // 5) Since we're running at interrupt level, no barriers are necessary.
    //    This is more hope than anything else I'm afraid.

    static int count = 52;
    static int count2 = 52;
    uint16_t counter = LL_TIM_GetCounter(state->timer);
    uint32_t pinIsHigh = IORead(state->io);
    if (counter >= state->nine) {
        // About 1 sec, but prime.
        if (++count == 53) {
            dprintf("PWM interrupt 1, pulseEndTime = %d, val2 = %d, capture = %d counter = %d, pin %d\r\n", state->pulseEndTime, *timerCCR(state->timer, state->indirectChannel), capture, counter, pinIsHigh);
            count = 0;
        }
        uint16_t telemVal = *timerCCR(state->timer, state->indirectChannel) - state->pulseEndTime;
        if (!pinIsHigh) {
            // If the pin is low 9ms past the start of the pulse while in
            // open-drain mode, the pull-up is not working.  Assuming the
            // wiring is correct, the battery may be disconnected.  We
            // should ignore any "telemetry" in this case, it's just noise.
            // Set telemIndex to 0 to wait for a sync frame.
            state->telemIndex = 0;
        } else if (telemVal == 0) {
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
        // Switch to push-pull output, so the falling edge of the next pulse is clean.
        LL_GPIO_SetPinOutputType(IO_GPIO(state->io), IO_Pin(state->io), LL_GPIO_OUTPUT_PUSHPULL);
        LL_TIM_OC_SetMode(state->timer, timerLLChannel(state->directChannel), LL_TIM_OCMODE_PWM1);
        // Switch the other channel back to the end-edge of the next pulse.
        LL_TIM_IC_SetPolarity(state->timer, timerLLChannel(state->indirectChannel),
                              LL_TIM_IC_POLARITY_RISING);
    } else {
        if (++count2 == 53) {
            dprintf("PWM interrupt, pulseEndTime = %d ccr = %d, ccrhi %d\r\n", capture, *timerCCR(state->timer, state->directChannel), *timerCCR(state->timer, state->indirectChannel));
            count2 = 0;
        }
        // Save the PWM CCR value from the shadow register.
        volatile timCCR_t* directCCR = timerCCR(state->timer, state->directChannel);
        captureCompare_t saveCCR = *directCCR;

        // Save the capture value for the rising edge of our own output.
        state->pulseEndTime = *timerCCR(state->timer, state->indirectChannel);
        LL_TIM_OC_SetMode(state->timer, timerLLChannel(state->directChannel), LL_TIM_OCMODE_FROZEN);
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
        // Set the other channel to record the falling edge of the 'tick'.
        LL_TIM_IC_SetPolarity(state->timer, timerLLChannel(state->directChannel ^ TIM_CHANNEL_2), LL_TIM_IC_POLARITY_FALLING);
        // Turn off preload because we expect this next compare value to occur in the same
        // counting cycle.
        LL_TIM_OC_DisablePreload(state->timer,
                                 timerLLChannel(state->directChannel));
        // This timer is upcounting, so the correct new CCR value is about 9ms
        // (max 2ms pulse + wait 7ms for tick.  Max correct tick is 5.5ms
        // (including the required 0.5ms delay), so that's plenty of
        // margin, and still allows 100Hz (10ms cycle) operation)
        *directCCR = state->nine;

        LL_TIM_OC_EnablePreload(state->timer,
                                timerLLChannel(state->directChannel));
        // Put the CCR register back for the next cycle (since preload is now on,
        // the 9ms we just set will not be cleared).
        *directCCR = saveCCR;
    }
}

bool castleInputConfig(const timerHardware_t* timerHardware, float hz) {
    if (castleState.timer) {
        dprintf("Castle telemetry already configured, we only support one.\r\n");
        return false;
    }
    uint8_t complementChannel = timerHardware->channel ^ TIM_CHANNEL_2;
    if (timerGetConfiguredByNumberAndChannel(timerGetTIMNumber(timerHardware->tim),
                                             complementChannel)) {
        dprintf("Castle telemetry complementary channel is configured, thus unavailable: TIM%d, CH%d\r\n",
                timerHardware->tim, complementChannel);
        return false;
    }
    dprintf("Configuring input for Castle telemetry, timer = %d, chans = %d, %d!\r\n",
            timerGetTIMNumber(timerHardware->tim), timerHardware->channel, complementChannel);
    castleState.timer = timerHardware->tim;
    castleState.directChannel = timerHardware->channel;
    castleState.indirectChannel = complementChannel;
    castleState.io = IOGetByTag(timerHardware->tag);
    float nineMs = 9e-3 * hz;
    castleState.nine = lrintf(nineMs);
    dprintf("Nine = %d\r\n", castleState.nine);

    // Initialize the input capture.
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if (Handle == NULL)
	return false;
    TIM_IC_InitTypeDef icInit;
    // Note: Castle Link is inverted.
    icInit.ICPolarity =  TIM_ICPOLARITY_RISING;
    icInit.ICSelection = TIM_ICSELECTION_INDIRECTTI; // Indirect capture
    icInit.ICPrescaler = TIM_ICPSC_DIV1;  // Every edge
    icInit.ICFilter = 0; // No filtering
    HAL_TIM_IC_ConfigChannel(Handle, &icInit, complementChannel);
    HAL_TIM_IC_Start(Handle, complementChannel);

    // Configure interrupts.  These occur on the direct channel.
    timerChCCHandlerInit(&castleState.pwmEdgeCb, pwmEdgeCallback);
    timerChConfigCallbacks(timerHardware, &castleState.pwmEdgeCb, NULL);
    timerNVICConfigure(timerInputIrq(timerHardware->tim));

    // Restart output in interrupt mode.
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
	HAL_TIMEx_PWMN_Stop(Handle, timerHardware->channel);
	HAL_TIMEx_PWMN_Start_IT(Handle, timerHardware->channel);
    }
    else {
	HAL_TIM_PWM_Stop(Handle, timerHardware->channel);
	HAL_TIM_PWM_Start_IT(Handle, timerHardware->channel);
    }
    return true;
}
