#include <stdint.h>
#include "stm32f401xc.h"

/* Latest ADC value */
volatile uint16_t ecg_adc_value;

volatile uint8_t ecg_half_ready = 0;
volatile uint8_t ecg_full_ready = 0;

volatile uint32_t dma_half_count = 0;
volatile uint32_t dma_full_count = 0;

volatile uint8_t lead_off_lo_minus = 0;
volatile uint8_t lead_off_lo_plus  = 0;
volatile uint8_t lead_off_detected = 0;

#define ECG_BUFFER_SIZE 256

volatile uint16_t ecg_buffer[ECG_BUFFER_SIZE];

float ecg_centered[128];

float ecg_filtered[128];

void LeadOff_GPIO_Init(void)
{
    /* Enable GPIOB clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB0 and PB1 as input */
    GPIOB->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)));

    /* No pull-up / pull-down */
    GPIOB->PUPDR &= ~((3U << (0 * 2)) | (3U << (1 * 2)));
}

void ECG_BandpassFilter(float *input, float *output)
{
    /* Biquad Section 1 */
    const float b10 = 0.04514067f;
    const float b11 = 0.09028134f;
    const float b12 = 0.04514067f;
    const float a11 = -1.31913863f;
    const float a12 = 0.50059036f;

    /* Biquad Section 2 */
    const float b20 = 1.0f;
    const float b21 = -2.0f;
    const float b22 = 1.0f;
    const float a21 = -1.99111877f;
    const float a22 = 0.99115903f;

    /* State for Section 1 */
    static float x1_1 = 0.0f;
    static float x2_1 = 0.0f;
    static float y1_1 = 0.0f;
    static float y2_1 = 0.0f;

    /* State for Section 2 */
    static float x1_2 = 0.0f;
    static float x2_2 = 0.0f;
    static float y1_2 = 0.0f;
    static float y2_2 = 0.0f;

    for (int i = 0; i < 128; i++)
    {
        float x = input[i];

        /* Section 1 */
        float y1 = b10 * x
                 + b11 * x1_1
                 + b12 * x2_1
                 - a11 * y1_1
                 - a12 * y2_1;

        x2_1 = x1_1;
        x1_1 = x;
        y2_1 = y1_1;
        y1_1 = y1;

        /* Section 2 */
        float y2 = b20 * y1
                 + b21 * x1_2
                 + b22 * x2_2
                 - a21 * y1_2
                 - a22 * y2_2;

        x2_2 = x1_2;
        x1_2 = y1;
        y2_2 = y1_2;
        y1_2 = y2;

        output[i] = y2;
    }
}

void ECG_RemoveDC(uint16_t *input, float *output)
{
    float sum = 0.0f;
    float mean;

    /* Calculate mean of 128 ECG samples */
    for (int i = 0; i < 128; i++)
    {
        sum += (float)input[i];
    }

    mean = sum / 128.0f;

    /* Remove DC component */
    for (int i = 0; i < 128; i++)
    {
        output[i] = (float)input[i] - mean;
    }
}
/* =========================================================
   Simple delay
   Only for this initial ADC + UART test.
   We will remove this when Timer + DMA are implemented.
   ========================================================= */



/* =========================================================
   USART2 initialization
   PA2 = USART2_TX
   Baud rate = 115200
   APB1 clock = 42 MHz
   ========================================================= */



void USART2_Init(void)
{
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* Enable USART2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;


    /* -----------------------------------------------------
       PA2 -> Alternate Function mode
       ----------------------------------------------------- */

    /* PA2 MODER = 10 (Alternate Function) */
    GPIOA->MODER &= ~(3U << (2 * 2));
    GPIOA->MODER |=  (2U << (2 * 2));

    /* No pull-up / pull-down */
    GPIOA->PUPDR &= ~(3U << (2 * 2));


    /* -----------------------------------------------------
       PA2 -> AF7 = USART2
       AFR[0] controls PA0 - PA7
       ----------------------------------------------------- */

    GPIOA->AFR[0] &= ~(0xFU << (4 * 2));
    GPIOA->AFR[0] |=  (7U << (4 * 2));


    /* -----------------------------------------------------
       USART2 configuration
       ----------------------------------------------------- */

    /* Disable USART before configuration */
    USART2->CR1 &= ~USART_CR1_UE;

    /*
       115200 baud
       APB1 = 42 MHz

       USARTDIV = 42,000,000 / (16 * 115200)
               ≈ 22.786

       BRR ≈ 0x016D
    */
    USART2->BRR = 0x016D;

    /* 8 data bits */
    USART2->CR1 &= ~USART_CR1_M;

    /* 1 stop bit */
    USART2->CR2 &= ~USART_CR2_STOP;

    /* No parity */
    USART2->CR1 &= ~USART_CR1_PCE;

    /* Enable transmitter */
    USART2->CR1 |= USART_CR1_TE;

    /* Enable USART */
    USART2->CR1 |= USART_CR1_UE;
}


/* =========================================================
   Send one character through USART2
   ========================================================= */
void USART2_SendChar(int c)
{
    /* Wait until transmit data register is empty */
    while (!(USART2->SR & USART_SR_TXE))
    {
    }

    USART2->DR = (uint8_t)c;
}


/* =========================================================
   Send string
   ========================================================= */
void USART2_SendString(const char *str)
{
    while (*str)
    {
        USART2_SendChar(*str++);
    }
}


/* =========================================================
   Send unsigned integer
   ========================================================= */
void USART2_SendUInt(uint32_t value)
{
    char buffer[10];
    int i = 0;

    /* Special case: zero */
    if (value == 0)
    {
        USART2_SendChar('0');
        return;
    }

    /* Convert number to ASCII */
    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }

    /* Send in reverse order */
    while (i > 0)
    {
        USART2_SendChar(buffer[--i]);
    }
}


/* =========================================================
   ADC1 initialization

   AD8232 OUTPUT -> PA1
   PA1 -> ADC1_IN1

   12-bit ADC
   ========================================================= */
/* =========================================================
   TIM2 initialization

   TIM2 clock = 84 MHz

   84 MHz / (8399 + 1) = 10 kHz
   10 kHz / (19 + 1)   = 500 Hz

   TIM2 TRGO -> ADC1 external trigger
   ========================================================= */


void TIM2_Init(void)
{
    /* Enable TIM2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Prescaler */
    TIM2->PSC = 8399;

    /* Auto-reload */
    TIM2->ARR = 19;

    /*
     * Generate an update event.
     * This loads PSC/ARR and initializes the timer.
     */
    TIM2->EGR = TIM_EGR_UG;

    /*
     * Master Mode Selection:
     *
     * MMS = 010
     * TRGO = Update Event
     */
    TIM2->CR2 &= ~TIM_CR2_MMS;
    TIM2->CR2 |= (2U << TIM_CR2_MMS_Pos);

    /* Start TIM2 */
    TIM2->CR1 |= TIM_CR1_CEN;
}
void ADC1_Init(void)
{
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* -----------------------------------------------------
       PA1 -> Analog mode
       PA1 = ADC1_IN1
       ----------------------------------------------------- */

    GPIOA->MODER &= ~(3U << (2 * 1));
    GPIOA->MODER |=  (3U << (2 * 1));

    /* No pull-up / pull-down */
    GPIOA->PUPDR &= ~(3U << (2 * 1));


    /* -----------------------------------------------------
       Enable ADC1 clock
       ----------------------------------------------------- */

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;


    /* Disable ADC before configuration */
    ADC1->CR2 &= ~ADC_CR2_ADON;


    /* -----------------------------------------------------
       12-bit resolution
       ----------------------------------------------------- */

    ADC1->CR1 &= ~(3U << ADC_CR1_RES_Pos);


    /* Right alignment */
    ADC1->CR2 &= ~ADC_CR2_ALIGN;


    /* Single conversion mode */
    ADC1->CR2 &= ~ADC_CR2_CONT;


    /* -----------------------------------------------------
       TIM2 TRGO -> ADC external trigger
       ----------------------------------------------------- */

    /* Clear external trigger selection */
    ADC1->CR2 &= ~ADC_CR2_EXTSEL;

    /*
       STM32F401:
       EXTSEL = 0110 -> TIM2 TRGO
    */
    ADC1->CR2 |= (6U << ADC_CR2_EXTSEL_Pos);

    /*
       External trigger enabled
       Rising edge
    */
    ADC1->CR2 &= ~ADC_CR2_EXTEN;
    ADC1->CR2 |= ADC_CR2_EXTEN_0;


    /* -----------------------------------------------------
       Select channel 1
       PA1 = ADC1_IN1
       ----------------------------------------------------- */

    /* One conversion in regular sequence */
    ADC1->SQR1 &= ~(0xFU << ADC_SQR1_L_Pos);

    /* First conversion = channel 1 */
    ADC1->SQR3 &= ~(0x1FU << ADC_SQR3_SQ1_Pos);
    ADC1->SQR3 |=  (1U << ADC_SQR3_SQ1_Pos);


    /* -----------------------------------------------------
       ADC sample time
       Channel 1 = 84 cycles
       ----------------------------------------------------- */

    ADC1->SMPR2 &= ~(7U << ADC_SMPR2_SMP1_Pos);
    ADC1->SMPR2 |=  (6U << ADC_SMPR2_SMP1_Pos);


    /* Enable ADC -> DMA requests */
    ADC1->CR2 |= ADC_CR2_DMA;

    /* Enable ADC */
    ADC1->CR2 |= ADC_CR2_ADON;
}

/* =========================================================
   DMA2 Stream 0 initialization for ADC1

   ADC1 -> DMA2 Stream 0 Channel 0
   Peripheral: ADC1->DR
   Memory:     ecg_buffer[256]
   Mode:       Circular
   Data size:  16-bit
   ========================================================= */
void DMA2_ADC1_Init(void)
{
    /* Enable DMA2 clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* Disable DMA2 Stream 0 before configuration */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;

    /* Wait until stream is actually disabled */
    while (DMA2_Stream0->CR & DMA_SxCR_EN)
    {
    }

    /* Clear all interrupt flags for Stream 0 */
    DMA2->LIFCR =
          DMA_LIFCR_CFEIF0
        | DMA_LIFCR_CDMEIF0
        | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CHTIF0
        | DMA_LIFCR_CTCIF0;

    /* -----------------------------------------------------
       Peripheral address = ADC1 data register
       ----------------------------------------------------- */
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;

    /* -----------------------------------------------------
       Memory address = ECG buffer
       ----------------------------------------------------- */
    DMA2_Stream0->M0AR = (uint32_t)ecg_buffer;

    /* Number of transfers */
    DMA2_Stream0->NDTR = ECG_BUFFER_SIZE;

    /* -----------------------------------------------------
       Stream configuration

       Channel 0 -> ADC1
       Peripheral-to-memory
       Circular mode
       Memory increment
       16-bit peripheral
       16-bit memory
       ----------------------------------------------------- */

    DMA2_Stream0->CR = 0;

    /* Channel selection = Channel 0 */
    DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL;

    /* Peripheral-to-memory
       DIR = 00
    */
    DMA2_Stream0->CR &= ~DMA_SxCR_DIR;

    /* Circular mode */
    DMA2_Stream0->CR |= DMA_SxCR_CIRC;

    /* Memory increment */
    DMA2_Stream0->CR |= DMA_SxCR_MINC;

    /* Peripheral size = 16 bits */
    DMA2_Stream0->CR &= ~DMA_SxCR_PSIZE;
    DMA2_Stream0->CR |= (1U << DMA_SxCR_PSIZE_Pos);

    /* Memory size = 16 bits */
    DMA2_Stream0->CR &= ~DMA_SxCR_MSIZE;
    DMA2_Stream0->CR |= (1U << DMA_SxCR_MSIZE_Pos);

    /* Peripheral increment disabled */
    DMA2_Stream0->CR &= ~DMA_SxCR_PINC;

    /* Direct mode */
    DMA2_Stream0->FCR &= ~DMA_SxFCR_DMDIS;

    /* High priority */
    DMA2_Stream0->CR &= ~DMA_SxCR_PL;
    DMA2_Stream0->CR |= (2U << DMA_SxCR_PL_Pos);

    /* Enable half-transfer interrupt */
    DMA2_Stream0->CR |= DMA_SxCR_HTIE;

    /* Enable transfer-complete interrupt */
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;

    /* Enable DMA2 Stream 0 interrupt in NVIC */
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    /* Enable DMA stream */
    DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void DMA2_Stream0_IRQHandler(void)
{
    /*
     * Half-transfer:
     * ecg_buffer[0] to ecg_buffer[127] ready
     */
    if (DMA2->LISR & DMA_LISR_HTIF0)
    {
        DMA2->LIFCR = DMA_LIFCR_CHTIF0;
        ecg_half_ready = 1;
        dma_half_count++;
    }

    /*
     * Transfer complete:
     * ecg_buffer[128] to ecg_buffer[255] ready
     */
    if (DMA2->LISR & DMA_LISR_TCIF0)
    {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;

        ecg_full_ready = 1;
        dma_full_count++;
    }
}
/* =========================================================
   MAIN
   ========================================================= */
int main(void)
{
    /* -----------------------------------------------------
       Enable Cortex-M4 FPU
       ----------------------------------------------------- */

    SCB->CPACR |= (0xFU << 20);

    __DSB();
    __ISB();


    /* -----------------------------------------------------
       Initialize peripherals
       ----------------------------------------------------- */

    USART2_Init();
    LeadOff_GPIO_Init();
    DMA2_ADC1_Init();
    ADC1_Init();
    TIM2_Init();


    /* -----------------------------------------------------
       Startup message
       ----------------------------------------------------- */

    USART2_SendString("\r\n");
    USART2_SendString("TrustFusion ECG ADC Test\r\n");
    USART2_SendString("STM32F401CCU6\r\n");
    USART2_SendString("ADC1 PA1\r\n");
    USART2_SendString("AD8232 OUTPUT -> PA1\r\n");
    USART2_SendString("USART2 PA2\r\n");
    USART2_SendString("----------------------\r\n");


    /* -----------------------------------------------------
       Main loop
       ----------------------------------------------------- */

    while (1)
    {
    	lead_off_lo_minus = (GPIOB->IDR >> 0) & 1U;
    	lead_off_lo_plus  = (GPIOB->IDR >> 1) & 1U;

    	lead_off_detected = lead_off_lo_minus | lead_off_lo_plus;
    	if (ecg_half_ready)
    	{
    	    ecg_half_ready = 0;

    	    ECG_RemoveDC((uint16_t *)ecg_buffer, ecg_centered);

    	    ECG_BandpassFilter(ecg_centered, ecg_filtered);

    	    USART2_SendString("HALF BUFFER READY\r\n");
    	}

    	if (ecg_full_ready)
    	{
    	    ecg_full_ready = 0;

    	    ECG_RemoveDC((uint16_t *)&ecg_buffer[128], ecg_centered);

    	    ECG_BandpassFilter(ecg_centered, ecg_filtered);

    	    USART2_SendString("FULL BUFFER READY\r\n");
    	}
    }
}
