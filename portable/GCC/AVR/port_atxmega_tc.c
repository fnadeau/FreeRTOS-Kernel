/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"

/*-----------------------------------------------------------
 * Timer/Counter implementation for ATxmega
 *
 * To use this file, you will need to define the following in FreeRTOSConfig.h:
 * - configUSE_TIMER_INSTANCE_LETTER
 *   This is the letter, in capital, of the counter, for TCC0, use:
 *     #define configUSE_TIMER_INSTANCE_LETTER C
 * - configUSE_TIMER_INSTANCE_INDEX
 *   This is the counter index, for TCC1, use:
 *     #define configUSE_TIMER_INSTANCE_INDEX 1
 * - configUSE_TIMER_CLKSEL
 *   This is the clock selection for timer, it can be:
 *     TC_CLKSEL_DIV1_gc
 *     TC_CLKSEL_DIV2_gc
 *     TC_CLKSEL_DIV4_gc
 *     TC_CLKSEL_DIV8_gc
 *     TC_CLKSEL_DIV64_gc
 *     TC_CLKSEL_DIV256_gc
 *     TC_CLKSEL_DIV1024_gc
 *-----------------------------------------------------------
 */

/* Macro expention helper */

#define _PRGEN_INDEX( id )            PRGEN_ ## id
#define PRGEN_INDEX( id )             _PRGEN_INDEX( id )

#define _PC_TC_bm( id )               PR_TC ## id ## _bm
#define PC_TC_bm( id )                _PC_TC_bm( id )

#define _TC_WGMODE_gm( id )           TC ## id ## _WGMODE_gm
#define TC_WGMODE_gm( id )            _TC_WGMODE_gm( id )

#define _TC( port, id )               TC ## port ## id
#define TC( port, id )                _TC( port, id )

#define _TC_t( id )                   TC ## id ## _t
#define TC_t( id )                    _TC_t( id )

#define _TC_OVFINTLVL( id, group )    TC ## id ## _OVFINTLVL_ ## group
#define TC_OVFINTLVL( id, group )     _TC_OVFINTLVL( id, group )

#define _TC_CLKSEL( id, group )       TC ## id ## _CLKSEL_ ## group
#define TC_CLKSEL( id, group )        _TC_CLKSEL( id, group )

#define _VECTOR_NAME( port, id )      TC ## port ## id ## _OVF_vect
#define VECTOR_NAME( port, id )       _VECTOR_NAME( port, id )

#define PGEN_IDX           PRGEN_INDEX(configUSE_TIMER_INSTANCE_LETTER)
#define TC_REG             TC( configUSE_TIMER_INSTANCE_LETTER, configUSE_TIMER_INSTANCE_INDEX )
#define TC_type            TC_t( configUSE_TIMER_INSTANCE_INDEX )
#define TC_OVFINTLVL_gm    TC_OVFINTLVL( configUSE_TIMER_INSTANCE_INDEX, gm )
#define TC_OVFINTLVL_gp    TC_OVFINTLVL( configUSE_TIMER_INSTANCE_INDEX, gp )
#define TC_CLKSEL_gm       TC_CLKSEL( configUSE_TIMER_INSTANCE_INDEX, gm )
#define VECTOR             VECTOR_NAME( configUSE_TIMER_INSTANCE_LETTER, configUSE_TIMER_INSTANCE_INDEX )

enum prgen_offset
{
    PRGEN_GEN,
    PRGEN_A,
    PRGEN_B,
    PRGEN_C,
    PRGEN_D,
    PRGEN_E,
    PRGEN_F
};

#if configUSE_PREEMPTION == 1

/*
 * Tick ISR for preemptive scheduler.  We can use a naked attribute as
 * the context is saved at the start of vPortYieldFromTick().  The tick
 * count is incremented after the context is saved.
 */
    ISR( VECTOR, ISR_NAKED )
    {
        /*Flag is cleared automatically when vector is executed */

        portSAVE_CONTEXT();

        if( xTaskIncrementTick() != pdFALSE )
        {
            vTaskSwitchContext();
        }

        portRESTORE_CONTEXT();

        asm volatile ( "reti" );
    }
#else /* if configUSE_PREEMPTION == 1 */

/*
 * Tick ISR for the cooperative scheduler.  All this does is increment the
 * tick count.  We don't need to switch context, this can only be done by
 * manual calls to taskYIELD();
 */
    ISR( VECTOR )
    {
        /*Flag is cleared automatically when vector is executed */
        xTaskIncrementTick();
    }
#endif /* if configUSE_PREEMPTION == 1 */


void prvSetupTimerInterrupt( void )
{
    /* Enable power */
    *( ( uint8_t * ) &PR.PRGEN + PGEN_IDX ) &= ~( PC_TC_bm( configUSE_TIMER_INSTANCE_INDEX ) | PR_HIRES_bm );

    /* Set waveforme mode */
    ( ( TC_type * ) &TC_REG )->CTRLB = ( ( ( TC_type * ) &TC_REG )->CTRLB & ~TC_WGMODE_gm( configUSE_TIMER_INSTANCE_INDEX ) ) | TC_WGMODE_NORMAL_gc;

    /* Set period */
    ( ( TC_type * ) &TC_REG )->PER = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

    /* Enable int */
    ( ( TC_type * ) &TC_REG )->INTCTRLA = ( ( TC_type * ) &TC_REG )->INTCTRLA & ~TC_OVFINTLVL_gm;
    ( ( TC_type * ) &TC_REG )->INTCTRLA = ( ( TC_type * ) &TC_REG )->INTCTRLA | ( 1 << TC_OVFINTLVL_gp );

    /* Enable timer source prescaler */
    ( ( TC_type * ) &TC_REG )->CTRLA = ( ( ( TC_type * ) &TC_REG )->CTRLA & ~TC_CLKSEL_gm ) | configUSE_TIMER_CLKSEL;
}

void disable( void )
{
    *( ( uint8_t * ) &PR.PRGEN + PGEN_IDX ) |= ( PC_TC_bm( configUSE_TIMER_INSTANCE_INDEX ) | PR_HIRES_bm );
}
