/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "os/mynewt.h"
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrfx_config_nrf9160.h"
#include "nrf_spu.h"
#include "spm/spm_internal.h"

/* For Cortex Ms */
#define AIRCR_VECT_KEY_PERMIT_WRITE 0x05FAUL

#define PERIPH(name, reg, config)                       \
    {                                                   \
        name, .id = NRFX_PERIPHERAL_ID_GET(reg), config \
    }

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CODE_FLASH_START_NS         0x00010000  
#define CODE_FLASH_SIZE_NS          0x00030000
#define CODE_FLASH_START_NSC        0x1000FE00
#define CODE_FLASH_SIZE_NSC         0x200
#define DATA_RAM_START_NS           0x20008000
#define DATA_RAM_SIZE_NS            0x0002B000
#define PERIPH_START_NS             0x40000000
#define PERIPH_SIZE_NS              0x00100000

#define FIRST_NONSECURE_ADDRESS (CODE_FLASH_START_NS)
#define LAST_SECURE_REGION_INDEX \
	((FIRST_NONSECURE_ADDRESS / FLASH_SECURE_ATTRIBUTION_REGION_SIZE) - 1)

static void
config_regions(bool ram, uint16_t start, size_t end, uint32_t perm)
{
    uint16_t i;

    for (i = start; i < end; i++) {
        if (ram) {
            NRF_SPU_S->RAMREGION[i].PERM = perm;
        } else {
            NRF_SPU_S->FLASHREGION[i].PERM = perm;
        }
    }
}

static void
spm_config_sram(void)
{
    /* Make entire RAM non-secure */
    const uint32_t nonsecure_ram_perm = SRAM_READ | SRAM_WRITE | SRAM_EXEC
 	| SRAM_LOCK | SRAM_NONSEC;


    /* Configuration for Secure RAM Regions (0 - 128 kB) */
    config_regions(true, 0, NUM_RAM_SECURE_ATTRIBUTION_REGIONS,
   		   nonsecure_ram_perm);
}

static void
spm_config_nsc_flash(void)
{
	/* Configure a single region in Secure Flash as Non-Secure Callable
	 * (NSC) area.
	 *
	 * Area to configure is dynamically decided with help from linker code.
	 *
	 * Note: Any Secure Entry functions, exposing secure services to the
	 * Non-Secure firmware, shall be located inside this NSC area.
	 *
	 * If the start address of the NSC area is hard-coded, it must follow
	 * the HW restrictions: The size must be a power of 2 between 32 and
	 * 4096, and the end address must fall on a SPU region boundary.
	 */

    NRF_SPU_S->FLASHNSC[0].REGION = FLASH_NSC_REGION_FROM_ADDR(0x10000);
    /* NRF_SPU_S->FLASHNSC[0].SIZE = FLASH_NSC_SIZE_REG(nsc_size); */
    NRF_SPU_S->FLASHNSC[0].SIZE = SPU_FLASHNSC_SIZE_SIZE_512;  /* MQ Hardcoded this value because FLASH_NSC_SIZE_REG(nsc_size) is incorrect */
}

static void
spm_config_flash(void)
{
    /* Regions of flash up to and including SPM are configured as Secure.
     * The rest of flash is configured as Non-Secure.
     */
    static const uint32_t flash_perm[] = {
    /* Configuration for Secure Regions */
        [0 ... LAST_SECURE_REGION_INDEX] =
	    FLASH_READ | FLASH_WRITE | FLASH_EXEC |
	    FLASH_LOCK | FLASH_SECURE,
	/* Configuration for Non Secure Regions */
	[(LAST_SECURE_REGION_INDEX + 1) ... 31] =
	    FLASH_READ | FLASH_WRITE | FLASH_EXEC |
	    FLASH_LOCK | FLASH_NONSEC,
	};

    /* Assign permissions */
    for (size_t i = 0; i < ARRAY_SIZE(flash_perm); i++) {

        NRF_SPU_S->FLASHREGION[i].PERM = flash_perm[i];
   }

   spm_config_nsc_flash();
}

static bool
usel_or_split(uint8_t id)
{
    const uint32_t perm = NRF_SPU->PERIPHID[id].PERM;

    /* NRF_GPIOTE1_NS needs special handling as its
     * peripheral ID for non-secure has incorrect properties
     * in the NRF_SPM->PERIPHID[id].perm register.
     */
    if (id == NRFX_PERIPHERAL_ID_GET(NRF_GPIOTE1_NS)) {
        return true;
    }

    bool present = (perm & SPU_PERIPHID_PERM_PRESENT_Msk) ==
        SPU_PERIPHID_PERM_PRESENT_Msk;

    /* User-selectable attribution */
    bool usel = (perm & SPU_PERIPHID_PERM_SECUREMAPPING_Msk) ==
        SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable;

    /* Split attribution */
    bool split = (perm & SPU_PERIPHID_PERM_SECUREMAPPING_Msk) ==
        SPU_PERIPHID_PERM_SECUREMAPPING_Split;

    return present && (usel || split);
}

static int
spm_config_peripheral(uint8_t id, bool dma_present)
{
    /* Set a peripheral to Non-Secure state, if
     * - it is present
     * - has UserSelectable/Split attribution.
     *
     * Assign DMA capabilities and lock down the attribution.
     *
     * Note: the function assumes that the peripheral ID matches
     * the IRQ line.
     */
    NVIC_DisableIRQ(id);

    if (usel_or_split(id)) {
	NRF_SPU->PERIPHID[id].PERM = PERIPH_PRESENT | PERIPH_NONSEC |
        (dma_present ? PERIPH_DMA_NOSEP : 0) | PERIPH_LOCK;
    }

    /* Even for non-present peripherals we force IRQs to be routed
     * to Non-Secure state.
     */
    NVIC_SetTargetState(id);
    return 0;
}

static void
spm_dppi_configure(uint32_t mask)
{
    NRF_SPU->DPPI[0].PERM = mask;
}

static void
spm_config_peripherals(void)
{
    int i, rc = 0;
    struct periph_cfg {
        const char* name;
 	uint8_t id;
	uint8_t nonsecure;
    };

    /* - All user peripherals are allocated to the Non-Secure domain.
     * - All GPIOs are allocated to the Non-Secure domain.
     */
    static const struct periph_cfg periph[] = {
#ifdef NRF_P0
        PERIPH("NRF_P0", NRF_P0_NS, 1),
#endif
#ifdef NRF_CLOCK
        PERIPH("NRF_CLOCK", NRF_CLOCK_NS, 1),
#endif
#ifdef NRF_RTC0
        PERIPH("NRF_RTC0", NRF_RTC0_NS, 1),
#endif
#ifdef NRF_RTC1
        PERIPH("NRF_RTC1", NRF_RTC1_NS, 1),
#endif
#ifdef NRF_NVMC
        PERIPH("NRF_NVMC", NRF_NVMC_NS, 1),
#endif
#ifdef NRF_UARTE1
        PERIPH("NRF_UARTE1", NRF_UARTE1_NS, 1),
#endif
#ifdef NRF_UARTE2
        PERIPH("NRF_UARTE2", NRF_UARTE2_NS, 1),
#endif
#ifdef NRF_TWIM2
        PERIPH("NRF_TWIM2", NRF_TWIM2_NS, 1),
#endif
#ifdef NRF_SPIM3
        PERIPH("NRF_SPIM3", NRF_SPIM3_NS, 1),
#endif
#ifdef NRF_TIMER0
        PERIPH("NRF_TIMER0", NRF_TIMER0_NS, 1),
#endif
#ifdef NRF_TIMER1
        PERIPH("NRF_TIMER1", NRF_TIMER1_NS, 1),
#endif
#ifdef NRF_TIMER2
        PERIPH("NRF_TIMER2", NRF_TIMER2_NS, 1),
#endif
#ifdef NRF_SAADC
        PERIPH("NRF_SAADC", NRF_SAADC_NS, 1),
#endif
#ifdef NRF_PWM0
        PERIPH("NRF_PWM0", NRF_PWM0_NS, 1),
#endif
#ifdef NRF_PWM1
        PERIPH("NRF_PWM1", NRF_PWM1_NS, 1),
#endif
#ifdef NRF_PWM2
        PERIPH("NRF_PWM2", NRF_PWM2_NS, 1),
#endif
#ifdef NRF_PWM3
        PERIPH("NRF_PWM3", NRF_PWM3_NS, 1),
#endif
#ifdef NRF_WDT
        PERIPH("NRF_WDT", NRF_WDT_NS, 1),
#endif
		/* There is no DTS node for the peripherals below,
		 * so address them using nrfx macros directly.
		 */
        PERIPH("NRF_IPC", NRF_IPC_S, 1),
        PERIPH("NRF_VMC", NRF_VMC_S, 1),
        PERIPH("NRF_FPU", NRF_FPU_S, 1),
        PERIPH("NRF_EGU1", NRF_EGU1_S, 1),
        PERIPH("NRF_EGU2", NRF_EGU2_S, 1),
        PERIPH("NRF_DPPIC", NRF_DPPIC_S, 1),

        PERIPH("NRF_GPIOTE1", NRF_GPIOTE1_NS, 1),
        PERIPH("NRF_REGULATORS", NRF_REGULATORS_S, 1),
    };

    spm_dppi_configure(0x0000000);

// XXX #if MYNEWT_VAL(SPM_NRF_P0_NS)
    /* Configure GPIO pins to be Non-Secure */
    NRF_SPU->GPIOPORT[0].PERM = 0;
//#endif
    for (i = 0; i < ARRAY_SIZE(periph); i++) {

        if (!periph[i].nonsecure) {
	    continue;
        }

        rc = spm_config_peripheral(periph[i].id, false);
        if (rc) {
            assert(0);
        }
    }
}

static void
tz_nonsecure_exception_prio_config(int secure_boost)
{
    uint32_t aircr_payload = SCB->AIRCR & (~(SCB_AIRCR_VECTKEY_Msk));
    if (secure_boost) {
        aircr_payload |= SCB_AIRCR_PRIS_Msk;
    } else {
        aircr_payload &= ~(SCB_AIRCR_PRIS_Msk);
    }
    SCB->AIRCR = ((AIRCR_VECT_KEY_PERMIT_WRITE << SCB_AIRCR_VECTKEY_Pos)
                 & SCB_AIRCR_VECTKEY_Msk) | aircr_payload;
}

static void
tz_nbanked_exception_target_state_set(int secure_state)
{
    uint32_t aircr_payload = SCB->AIRCR & (~(SCB_AIRCR_VECTKEY_Msk));
    if (secure_state) {
        aircr_payload &= ~(SCB_AIRCR_BFHFNMINS_Msk);
    } else {
        aircr_payload |= SCB_AIRCR_BFHFNMINS_Msk;
    }
    SCB->AIRCR = ((AIRCR_VECT_KEY_PERMIT_WRITE << SCB_AIRCR_VECTKEY_Pos)
                 & SCB_AIRCR_VECTKEY_Msk) | aircr_payload;
}

static void
tz_nonsecure_system_reset_req_block(int block)
{
    uint32_t aircr_payload = SCB->AIRCR & (~(SCB_AIRCR_VECTKEY_Msk));
    if (block) {
        aircr_payload |= SCB_AIRCR_SYSRESETREQS_Msk;
    } else {
        aircr_payload &= ~(SCB_AIRCR_SYSRESETREQS_Msk);
    }

    SCB->AIRCR = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)
                 & SCB_AIRCR_VECTKEY_Msk) | aircr_payload;
}

#if MYNEWT_VAL(HARDFLOAT)
void tz_nonsecure_fpu_access_enable(void)
{
    SCB->NSACR |=
        (1UL << SCBACR_CP10_Pos) | (1UL << SCBACR_CP11_Pos);
}
#endif

static void
tz_sau_configure(int enable, int allns)
{
    if (enable) {
        TZ_SAU_Enable();
    } else {
        TZ_SAU_Disable();
        if (allns) {
            SAU->CTRL |= SAU_CTRL_ALLNS_Msk;
        } else {
            SAU->CTRL &= ~(SAU_CTRL_ALLNS_Msk);
        }
    }
}

static void
spm_configure_ns(void)
{
/* XXX Configure core register block for Non-Secure state
 * tz_nonsecure_state_setup(spm_ns_conf); 
 * Make changes to VTOR, PSP, MSP, CONTROL to use non-secure
 */

/* Prioritize Secure exceptions over Non-Secure */
    tz_nonsecure_exception_prio_config(1);

/* Set non-banked exceptions to target Non-Secure */
    tz_nbanked_exception_target_state_set(0);

/* Configure if Non-Secure firmware should be allowed to issue System
 * reset. If not it could be enabled through a secure service.
 */
    tz_nonsecure_system_reset_req_block(1);

/* Allow SPU to have precedence over (non-existing) ARMv8-M SAU. */
    tz_sau_configure(0, 1);

//#if defined(CONFIG_ARMV7_M_ARMV8_M_FP) && defined(CONFIG_SPM_NRF_FPU_NS)
#if MYNEWT_VAL(HARDFLOAT)
/* Allow Non-Secure firmware to use the FPU */
    tz_nonsecure_fpu_access_enable();
#endif
}


void spm_jump(void)
{
#if 0
	/* Extract initial MSP of the Non-Secure firmware image.
	 * The assumption is that the MSP is located at VTOR_NS[0].
	 */
	u32_t *vtor_ns = (u32_t *)NON_SECURE_APP_ADDRESS;

	PRINT("SPM: NS image at 0x%x\n", (u32_t)vtor_ns);
	PRINT("SPM: NS MSP at 0x%x\n", vtor_ns[0]);
	PRINT("SPM: NS reset vector at 0x%x\n", vtor_ns[1]);

	/* Configure Non-Secure stack */
	tz_nonsecure_setup_conf_t spm_ns_conf = {
		.vtor_ns = (u32_t)vtor_ns,
		.msp_ns = vtor_ns[0],
		.psp_ns = 0,
		.control_ns.npriv = 0, /* Privileged mode*/
		.control_ns.spsel = 0 /* Use MSP in Thread mode */
	};
#endif
    spm_configure_ns();
#if 0
	/* Generate function pointer for Non-Secure function call. */
	TZ_NONSECURE_FUNC_PTR_DECLARE(reset_ns);
	reset_ns = TZ_NONSECURE_FUNC_PTR_CREATE(vtor_ns[1]);

	if (TZ_NONSECURE_FUNC_PTR_IS_NS(reset_ns)) {
		PRINT("SPM: prepare to jump to Non-Secure image.\n");

		/* Note: Move UARTE0 before jumping, if it is
		 * to be used on the Non-Secure domain.
		 */
#endif
		/* Configure UARTE0 as non-secure */
    spm_config_peripheral(NRFX_PERIPHERAL_ID_GET(NRF_UARTE0), 0);

    __DSB();
    __ISB();
#if 0
		/* Jump to Non-Secure firmware */
		reset_ns();

		CODE_UNREACHABLE;

	} else {
		PRINT("SPM: wrong pointer type: 0x%x\n",
		      (u32_t)reset_ns);
	}
#endif
}

void spm_config(void)
{
    tz_sau_configure(0, 1);
    spm_config_flash();
    spm_config_sram();
    spm_config_peripherals();
    //spm_configure_ns();
    /* Force memory writes before continuing */
    //__DSB();
    /* Flush and refill pipeline updated permissions */
    //__ISB();
}
