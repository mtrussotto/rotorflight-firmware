/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_PWM_OUTPUT

#include "build/atomic.h"
#include "build/dprintf.h"
#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

FAST_DATA_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) return;

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(Handle, &TIM_OCInitStructure, channel);
#else
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = value;

    timerOCInit(tim, channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
#endif
}

typedef struct castleInterrupt_s {
    timerCCHandlerRec_t pwmEdgeCb;
    pwmOutputPort_t* port;
    const timerHardware_t* timerHardware;
    volatile timCCR_t *ccr_hi;
    volatile timCCR_t *ccr_lo;
    timCCR_t nine;
    timCCR_t val1;
    castleTelemetry_t telem[2];
    uint8_t whichTelem; // telem we are writing to (0 or 1).
    uint8_t telemIndex; // where in the telem struct are we?
} castleInterrupt_t;

static FAST_DATA_ZERO_INIT castleInterrupt_t castleState;

void pwmGetCastleTelemetry(castleTelemetry_t* telem) {
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        memcpy(telem, &castleState.telem[castleState.whichTelem^1], sizeof(castleTelemetry_t));
    }
}

void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
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
    captureCompare_t saveCCR;
    uint16_t telemVal;
    uint16_t counter = LL_TIM_GetCounter(state->timerHardware->tim);

    uint32_t pinIsHigh = IORead(state->port->io);
    if (counter >= state->nine) {
        // About 1 sec, but prime.
        if (++count == 53) {
            dprintf("PWM interrupt 1, val1 = %d, val2 = %d, capture = %d counter = %d, pin %d\r\n", state->val1, *state->ccr_hi, capture, counter, pinIsHigh);
            count = 0;
        }
        telemVal = *state->ccr_hi - state->val1;
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
        LL_GPIO_SetPinOutputType(IO_GPIO(state->port->io), IO_Pin(state->port->io), LL_GPIO_OUTPUT_PUSHPULL);
        LL_TIM_OC_SetMode(state->timerHardware->tim, timerLLChannel(state->timerHardware->channel), LL_TIM_OCMODE_PWM1);
        // Switch the other channel back to the end-edge of the next pulse.
        LL_TIM_IC_SetPolarity(state->timerHardware->tim, timerLLChannel(state->timerHardware->channel ^ TIM_CHANNEL_2), LL_TIM_IC_POLARITY_RISING);
    } else {
        if (++count2 == 53) {
            dprintf("PWM interrupt, val1 = %d ccr = %d, ccrhi %d\r\n", capture, *state->ccr_lo, *state->ccr_hi);
            count2 = 0;
        }
        // Save the PWM CCR value from the shadow register.
        saveCCR = *state->port->channel.ccr;
        // Save the capture value
        state->val1 = *state->ccr_hi;
        // Freeze the output
        LL_TIM_OC_SetMode(state->timerHardware->tim, timerLLChannel(state->timerHardware->channel), LL_TIM_OCMODE_FROZEN);
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
        LL_GPIO_SetPinOutputType(IO_GPIO(state->port->io), IO_Pin(state->port->io), LL_GPIO_OUTPUT_OPENDRAIN);
        // Set the other channel to record the falling edge of the 'tick'.
        LL_TIM_IC_SetPolarity(state->timerHardware->tim, timerLLChannel(state->timerHardware->channel ^ TIM_CHANNEL_2), LL_TIM_IC_POLARITY_FALLING);
        // Turn off preload because we expect this next one to occur in the same
        // counting cycle.
        LL_TIM_OC_DisablePreload(state->timerHardware->tim,
                                 timerLLChannel(state->timerHardware->channel));
        // This timer is upcounting, so the correct new CCR value is 9ms
        // (max 2ms pulse + wait 7ms for tick.  Max correct tick is 5.5ms
        // (including the required 0.5ms delay), so that's plenty of
        // margin, and still allows 100Hz (10ms cycle) operation)
        *state->ccr_lo = state->nine;

        LL_TIM_OC_EnablePreload(state->timerHardware->tim,
                                timerLLChannel(state->timerHardware->channel));
        // Put the CCR register back for the next cycle (since preload is now on,
        // the 9ms we just set will not be cleared.
        *state->port->channel.ccr = saveCCR;
    }
}

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion, uint8_t intr)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if (Handle == NULL) return;
#endif

    configTimeBase(timerHardware->tim, period, hz);
    pwmOCConfig(timerHardware->tim,
                timerHardware->channel,
                value,
                inversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output
               );

#if defined(USE_HAL_DRIVER)
    if (intr) {
        dprintf("Configuring PWM for interrupt, timer = %d, nchan = %d!\r\n",
                timerGetTIMNumber(timerHardware->tim),
                timerHardware->output & TIMER_OUTPUT_N_CHANNEL);

        castleState.ccr_hi = timerChCCRHi(timerHardware);
        castleState.ccr_lo = timerChCCRLo(timerHardware);
        castleState.timerHardware = timerHardware;
        {
            float nineMs = 9e-3 * hz;
            castleState.nine = lrintf(nineMs);
        }
        dprintf("Nine = %d\r\n", castleState.nine);
        {
            // Initialize the input capture.
            TIM_IC_InitTypeDef icInit;
            // Note: Castle Link is inverted.
            icInit.ICPolarity = inversion ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
            icInit.ICSelection = TIM_ICSELECTION_INDIRECTTI; // Indirect capture
            icInit.ICPrescaler = TIM_ICPSC_DIV1;  // Every edge
            icInit.ICFilter = 0; // No filtering
            HAL_TIM_IC_ConfigChannel(Handle, &icInit, timerHardware->channel ^ TIM_CHANNEL_2);
            HAL_TIM_IC_Start(Handle, timerHardware->channel ^ TIM_CHANNEL_2);
        }

        timerChCCHandlerInit(&castleState.pwmEdgeCb, pwmEdgeCallback);
        timerChConfigCallbacks(timerHardware, &castleState.pwmEdgeCb, NULL);
        timerNVICConfigure(timerInputIrq(timerHardware->tim));

        if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL)
            HAL_TIMEx_PWMN_Start_IT(Handle, timerHardware->channel);
        else
            HAL_TIM_PWM_Start_IT(Handle, timerHardware->channel);
    } else {
        dprintf("Configuring PWM, nchan = %d!\r\n",
                timerHardware->output & TIMER_OUTPUT_N_CHANNEL);
        if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL)
            HAL_TIMEx_PWMN_Start(Handle, timerHardware->channel);
        else
            HAL_TIM_PWM_Start(Handle, timerHardware->channel);
    }
    HAL_TIM_Base_Start(Handle);
#else
    assert_param(!intr);
    TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);
#endif

    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    *channel->ccr = 0;
}


/* MOTORS */

static FAST_DATA_ZERO_INIT motorDevice_t motorPwmDevice;

static void pwmWriteUnused(uint8_t index, uint8_t mode, float value)
{
    UNUSED(index);
    UNUSED(mode);
    UNUSED(value);
}

static float pwmConvertToInternal(uint8_t index, uint8_t mode, float throttle)
{
    UNUSED(index);

    float value = motorConfig()->mincommand;

    if (mode == MOTOR_CONTROL_BIDIR) {
        if (throttle != 0)
            value = scaleRangef(throttle, -1, 1, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }
    else {
        if (throttle > 0)
            value = scaleRangef(throttle, 0, 1, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }

    return value;
}

static void pwmWriteStandard(uint8_t index, uint8_t mode, float throttle)
{
    float value = pwmConvertToInternal(index,mode,throttle);
    float pulse = value * motors[index].pulseScale + motors[index].pulseOffset;

    *motors[index].channel.ccr = lrintf(pulse);
}

void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].channel.ccr) {
            *motors[index].channel.ccr = 0;
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static motorVTable_t motorPwmVTable;

bool pwmEnableMotors(void)
{
    /* check motors can be enabled */
    return (motorPwmVTable.write != &pwmWriteUnused);
}

bool pwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}

static void pwmCompleteOneshotMotorUpdate(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        if (motors[index].forceOverflow) {
            timerForceOverflow(motors[index].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].channel.ccr = 0;
    }
}

static motorVTable_t motorPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint8_t motorCount)
{
    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;

    motorPwmDevice.vTable = motorPwmVTable;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case PWM_TYPE_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case PWM_TYPE_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case PWM_TYPE_STANDARD:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        break;
    case PWM_TYPE_CASTLE_LINK:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        break;
    }

    motorPwmDevice.vTable.write = pwmWriteStandard;
    motorPwmDevice.vTable.updateStart = motorUpdateStartNull;
    motorPwmDevice.vTable.updateComplete = useUnsyncedPwm ? motorUpdateCompleteNull : pwmCompleteOneshotMotorUpdate;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            motorPwmDevice.vTable.write = &pwmWriteUnused;
            motorPwmDevice.vTable.updateComplete = motorUpdateCompleteNull;
            /* TODO: block arming and add reason system cannot arm */
            return NULL;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);

        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        //
        // Castle link requires 5.5ms wait for tick, after a max 2ms
        // pulse, which implies an absolute maximum rate of about 133
        // Hz.  The protocol document specifies 50Hz.  This code
        // requires at least 9ms + epsilon cycle time, so we set a
        // maximum of 100Hz to be reasonably safe.
        const unsigned pwmRateHz = useUnsyncedPwm ?
                                   ((motorConfig->motorPwmProtocol == PWM_TYPE_CASTLE_LINK) ? MIN(CASTLE_PWM_HZ_MAX, motorConfig->motorPwmRate) : motorConfig->motorPwmRate) :
                                   ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        motors[motorIndex].pulseScale = (sLen * hz) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);

        if (motorConfig->motorPwmProtocol == PWM_TYPE_CASTLE_LINK) {
            castleState.port = motors + motorIndex;
        }
        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, 0, motorConfig->motorPwmProtocol == PWM_TYPE_CASTLE_LINK /*inversion*/, motorConfig->motorPwmProtocol == PWM_TYPE_CASTLE_LINK /* interrupt*/);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    return &motorPwmDevice;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

#endif // USE_PWM_OUTPUT
