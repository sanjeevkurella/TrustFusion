#include "stm32f401xc.h"
#include "system_stm32f4xx.h"

/* System clock frequency */
uint32_t SystemCoreClock = 16000000U;

/* AHB prescaler table */
const uint8_t AHBPrescTable[16] =
{
    0, 0, 0, 0, 0, 0, 1, 2,
    3, 4, 6, 7, 8, 9, 10, 11
};

/* APB prescaler table */
const uint8_t APBPrescTable[8] =
{
    0, 0, 0, 0, 1, 2, 3, 4
};


/**
 * @brief  Initialize the system clock.
 *
 * Clock configuration:
 *
 * HSI = 16 MHz
 *
 * PLL:
 *   PLLM = 16
 *   PLLN = 336
 *   PLLP = 4
 *
 * Therefore:
 *
 * VCO input  = 16 MHz / 16 = 1 MHz
 * VCO output = 1 MHz × 336 = 336 MHz
 * SYSCLK     = 336 MHz / 4 = 84 MHz
 */
void SystemInit(void)
{
    /* Reset RCC clock configuration to default state */

    /* Enable HSI */
    RCC->CR |= RCC_CR_HSION;

    /* Wait until HSI is ready */
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U)
    {
    }

    /* Reset CFGR */
    RCC->CFGR = 0x00000000U;

    /* Reset PLL configuration */
    RCC->PLLCFGR = 0x24003010U;

    /* Disable HSE, CSS and PLL */
    RCC->CR &= ~(RCC_CR_HSEON |
                 RCC_CR_CSSON |
                 RCC_CR_PLLON);

    /* Reset HSEBYP */
    RCC->CR &= ~RCC_CR_HSEBYP;

    /* Reset all RCC interrupt flags */
    RCC->CIR = 0x00000000U;

    /*
     * Configure Flash:
     * - Instruction cache
     * - Data cache
     * - Prefetch
     * - 2 wait states for 84 MHz operation
     */
    FLASH->ACR = FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN |
                 FLASH_ACR_PRFTEN |
                 FLASH_ACR_LATENCY_2WS;

    /*
     * PLL configuration:
     *
     * PLLSRC = HSI
     * PLLM   = 16
     * PLLN   = 336
     * PLLP   = 4
     */
    RCC->PLLCFGR =
        (16U << RCC_PLLCFGR_PLLM_Pos) |
        (336U << RCC_PLLCFGR_PLLN_Pos) |
        (1U << RCC_PLLCFGR_PLLP_Pos);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait until PLL is ready */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
    {
    }

    /*
     * Configure bus clocks:
     *
     * AHB  = SYSCLK / 1  = 84 MHz
     * APB1 = HCLK / 2    = 42 MHz
     * APB2 = HCLK / 1    = 84 MHz
     */
    RCC->CFGR =
        RCC_CFGR_HPRE_DIV1 |
        RCC_CFGR_PPRE1_DIV2 |
        RCC_CFGR_PPRE2_DIV1;

    /* Select PLL as system clock */
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait until PLL becomes system clock */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
    }

    /* Update SystemCoreClock variable */
    SystemCoreClock = 84000000U;
}


/**
 * @brief Update SystemCoreClock variable.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp;
    uint32_t pllvco;
    uint32_t pllp;
    uint32_t pllm;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
        case RCC_CFGR_SWS_HSI:
            SystemCoreClock = 16000000U;
            break;

        case RCC_CFGR_SWS_HSE:
            SystemCoreClock = 8000000U;
            break;

        case RCC_CFGR_SWS_PLL:

            pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

            pllvco = (16000000U / pllm) *
                     ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >>
                      RCC_PLLCFGR_PLLN_Pos);

            pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>
                     RCC_PLLCFGR_PLLP_Pos) + 1U) * 2U;

            SystemCoreClock = pllvco / pllp;
            break;

        default:
            SystemCoreClock = 16000000U;
            break;
    }

    /* Apply AHB prescaler */
    tmp = (RCC->CFGR & RCC_CFGR_HPRE) >>
          RCC_CFGR_HPRE_Pos;

    SystemCoreClock >>= AHBPrescTable[tmp];
}
