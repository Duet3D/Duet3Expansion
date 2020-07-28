#include "RepRapFirmware.h"

#if SAME5x

static void InitClocks();

extern "C" void AppInit() noexcept
{
	NVIC_DisableIRQ(CAN0_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);
	CAN0->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN0->ILE.reg = 0;
	CAN1->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN1->ILE.reg = 0;

	// We don't know which bootloader we entered from, and they use different clocks, so configure the clocks.
	// First reset the generic clock generator. This sets all clock generators to default values and the CPU clock to the 48MHz DFLL output.
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }

	// Disable DPLL0 so that we can reprogram it
	OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.ENABLE) { }

	// Also disable DPLL1
	OSCCTRL->Dpll[1].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.ENABLE) { }

	// Now it's safe to configure the clocks
	InitClocks();

	// All done
	SystemCoreClock = SystemCoreClockFreq;
}

// Initialise the clocks. The clock configuration is:
// XOSC1	12MHz crystal oscillator
// FDPLL0	Divides XOSC1 down to 3MHz then multiples it up to 120MHz
// FDPLL1	not used
// DFLL48M	closed loop mode frequency locked to XOSC1 via GCLK1, 48MHz
// GCLK0	FDPLL0 output, 120MHz (for CPU and fast peripherals)
// GCLK1	XOSC1 output divided by 366 to get 32765.4Hz, used as the slow clock and as a reference for DFLL48M
// GCLK2	not used
// GCLK3	FDPLL0 divided by 2, 60MHz (for slower peripherals)
// GCLK4	DFLL, 48MHz for CAN and step timer

static void InitClocks()
{
	hri_nvmctrl_set_CTRLA_RWS_bf(NVMCTRL, 0);				// need 6 wait states @ 120MHz
//	hri_nvmctrl_set_CTRLA_RWS_bf(NVMCTRL, 6);				// need 6 wait states @ 120MHz
//	hri_nvmctrl_clear_CTRLA_AUTOWS_bit(NVMCTRL);			// clear the auto WS bit

	// Initialise 32kHz oscillator
	const uint16_t calib = hri_osc32kctrl_read_OSCULP32K_CALIB_bf(OSC32KCTRL);
	hri_osc32kctrl_write_OSCULP32K_reg(OSC32KCTRL, OSC32KCTRL_OSCULP32K_CALIB(calib));

	// Initialise XOSC0
	hri_oscctrl_write_XOSCCTRL_reg(OSCCTRL, 0,
			  OSCCTRL_XOSCCTRL_CFDPRESC(3)
			| OSCCTRL_XOSCCTRL_STARTUP(0)
			| (0 << OSCCTRL_XOSCCTRL_SWBEN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_CFDEN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_ENALC_Pos)
			| OSCCTRL_XOSCCTRL_IMULT(4)						// 12MHz
			| OSCCTRL_XOSCCTRL_IPTAT(3)
			| (0 << OSCCTRL_XOSCCTRL_LOWBUFGAIN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_ONDEMAND_Pos)
			| (0 << OSCCTRL_XOSCCTRL_RUNSTDBY_Pos)
			| (1 << OSCCTRL_XOSCCTRL_XTALEN_Pos)
			| (1 << OSCCTRL_XOSCCTRL_ENABLE_Pos));

	while (!hri_oscctrl_get_STATUS_XOSCRDY0_bit(OSCCTRL)) { }

	// Initialise MCLK
	hri_mclk_write_CPUDIV_reg(MCLK, MCLK_CPUDIV_DIV(MCLK_CPUDIV_DIV_DIV1_Val));

	// Initialise FDPLL0. (12MHz / 4) * 40 = 120MHz
	hri_oscctrl_write_DPLLRATIO_reg(OSCCTRL, 0,
			  OSCCTRL_DPLLRATIO_LDRFRAC(0)
			| OSCCTRL_DPLLRATIO_LDR(39));			// multiply by 40
	hri_oscctrl_write_DPLLCTRLB_reg(OSCCTRL, 0,
			  OSCCTRL_DPLLCTRLB_DIV(1)				// divide by 4
			| (0 << OSCCTRL_DPLLCTRLB_DCOEN_Pos)
			| OSCCTRL_DPLLCTRLB_DCOFILTER(0)
			| (0 << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
			| OSCCTRL_DPLLCTRLB_LTIME(0)
			| OSCCTRL_DPLLCTRLB_REFCLK(2)			// source is XOSC0
			| (0 << OSCCTRL_DPLLCTRLB_WUF_Pos)
			| OSCCTRL_DPLLCTRLB_FILTER(0));
	hri_oscctrl_write_DPLLCTRLA_reg(OSCCTRL, 0,
			  (0 << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
			| (1 << OSCCTRL_DPLLCTRLA_ENABLE_Pos));

	while (!(hri_oscctrl_get_DPLLSTATUS_LOCK_bit(OSCCTRL, 0) || hri_oscctrl_get_DPLLSTATUS_CLKRDY_bit(OSCCTRL, 0))) { }

	// We must initialise GCLKs 0 and 1 before we touch the DFLL:
	// - GCLK0 is the CPU clock and defaults to the DFLL
	// - GCLK1 is used as the reference when we reprogram the DFLL

	// GCLK0: from FDPLL0 direct
	hri_gclk_write_GENCTRL_reg(GCLK, 0,
			  GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 7);

	// GCLK1: XOSC0 divided by 366 to give 32786.9Hz
	hri_gclk_write_GENCTRL_reg(GCLK, 1,
			  GCLK_GENCTRL_DIV(366) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 0);

	// Initialise DFLL48M in closed loop mode
	hri_gclk_write_PCHCTRL_reg(GCLK, OSCCTRL_GCLK_ID_DFLL48, GCLK_PCHCTRL_GEN(1) | GCLK_PCHCTRL_CHEN);		// set GCLK1 as DFLL reference

	hri_oscctrl_write_DFLLCTRLA_reg(OSCCTRL, 0);

	hri_oscctrl_write_DFLLMUL_reg(OSCCTRL, OSCCTRL_DFLLMUL_CSTEP(4) | OSCCTRL_DFLLMUL_FSTEP(4) | OSCCTRL_DFLLMUL_MUL(1464));
	while (hri_oscctrl_get_DFLLSYNC_DFLLMUL_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLCTRLB_reg(OSCCTRL, 0);
	while (hri_oscctrl_get_DFLLSYNC_DFLLCTRLB_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLCTRLA_reg(OSCCTRL, (0 << OSCCTRL_DFLLCTRLA_RUNSTDBY_Pos) | OSCCTRL_DFLLCTRLA_ENABLE);
	while (hri_oscctrl_get_DFLLSYNC_ENABLE_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLVAL_reg(OSCCTRL, hri_oscctrl_read_DFLLVAL_reg(OSCCTRL));
	while (hri_oscctrl_get_DFLLSYNC_DFLLVAL_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLCTRLB_reg(OSCCTRL,
			  (0 << OSCCTRL_DFLLCTRLB_WAITLOCK_Pos) | (0 << OSCCTRL_DFLLCTRLB_BPLCKC_Pos)
			| (0 << OSCCTRL_DFLLCTRLB_QLDIS_Pos) | (0 << OSCCTRL_DFLLCTRLB_CCDIS_Pos)
			| (0 << OSCCTRL_DFLLCTRLB_USBCRM_Pos) | (0 << OSCCTRL_DFLLCTRLB_LLAW_Pos)
			| (0 << OSCCTRL_DFLLCTRLB_STABLE_Pos) | (1u << OSCCTRL_DFLLCTRLB_MODE_Pos));
	while (hri_oscctrl_get_DFLLSYNC_DFLLCTRLB_bit(OSCCTRL)) { }

	// Initialise the other GCLKs
	// GCLK3: FDPLL0 divided by 2, 60MHz for peripherals that need less than 120MHz
	hri_gclk_write_GENCTRL_reg(GCLK, 3,
			  GCLK_GENCTRL_DIV(2) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 7);

	// GCLK4: DFLL48M for CAN and step timer
	hri_gclk_write_GENCTRL_reg(GCLK, 4,
			  GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 6);
}

// Return a pointer to the pin description entry. Declared in and called from CoreN2G.
const PinDescriptionBase *GetPinDescription(Pin p) noexcept
{
	return (p < ARRAY_SIZE(PinTable)) ? &PinTable[p] : nullptr;
}

// Define replacement standard library functions
#include <syscalls.h>

#endif

// End
