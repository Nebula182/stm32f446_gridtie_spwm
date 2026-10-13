#include <math.h>
#include "stm32f446xx.h"

void SystemClock_Config(void);

#define PWM_ARR      4500  // Частота ШІМ = 20 кГц (при 180 МГц у Center-aligned)
#define LUT_SIZE     400   // Розмір буфера ШІМ для одного періоду 50 Гц (400 * 50мкс = 20мс)
#define TS           0.000050f // Період дискретизації = 50 мікросекунд (20 кГц)

/* ================= БУФЕРИ ШІМ ================= */
uint16_t bufA[LUT_SIZE * 2];
uint16_t bufB[LUT_SIZE * 2];
volatile uint8_t active_A = 1;
volatile uint8_t dma_ready_to_update = 0;

/* ================= СТРУКТУРА ТА ЗМІННІ SOGI-PLL ================= */
typedef struct {
    float osg_k;    // Коефіцієнт підсилення SOGI (зазвичай 1.0 ... 1.4)
    float osg_w0;   // Базова частота (2 * pi * 50 = 314.159f)
    float x;        // Внутрішній стан інсталятора 1
    float y;        // Внутрішній стан інсталятора 2
    float v_d;      // Пряма компонента напруги
    float v_q;      // Квадратурна компонента напруги (має прагнути до 0)
    
    // PI регулятор петлі PLL
    float pi_kp;
    float pi_ki;
    float pi_i;
    
    float omega;    // Розрахована частота мережі (рад/с)
    float theta;    // Вихідний чистий кут фази мережі (0 ... 2*PI)
} SOGI_PLL_t;

volatile SOGI_PLL_t pll;
volatile float Vgrid_raw_f = 0.0f; // Оцифрована напруга з АЦП
volatile float k_mod = 0.8f;       // Коефіцієнт модуляції інвертора
volatile uint8_t grid_present = 0;

/* ================= ІНІЦІАЛІЗАЦІЯ SOGI-PLL ================= */
void sogi_pll_init(void)
{
    pll.osg_k  = 1.4142f;
    pll.osg_w0 = 2.0f * M_PI * 50.0f; // 314.1592f
    pll.x = 0.0f;
    pll.y = 0.0f;
    
    // Налаштування PI-регулятора для частоти 20 кГц
    pll.pi_kp = 40.0f;
    pll.pi_ki = 1400.0f;
    pll.pi_i  = 0.0f;
    
    pll.omega = pll.osg_w0;
    pll.theta = 0.0f;
}

/* ================= ОБЧИСЛЕННЯ SOGI-PLL (Викликається на 20 кГц) ================= */
static inline void sogi_pll_update(float v_grid_sample)
{
    // 1. Алгоритм SOGI (генерація ортогональних сигналів через дискретні інтегратори)
    float error = v_grid_sample - pll.v_d;
    
    // Інтегратор 1 (метод Ейлера або трапецій, спрощено для швидкості за 1 такт FPU)
    pll.x += TS * pll.omega * (pll.osg_k * error - pll.y);
    pll.v_d = pll.x;
    
    // Інтегратор 2
    pll.y += TS * pll.omega * pll.v_d;
    pll.v_q = pll.y;
    
    // 2. Блок детектора фази (Park transformation спрощений для однофазної системи)
    float cos_t = cosf(pll.theta);
    float sin_t = sinf(pll.theta);
    
    // Виділяємо помилку синхронізації по осі Q
    float eq = -pll.v_d * sin_t + pll.v_q * cos_t;
    
    // 3. PI-регулятор петлі PLL
    pll.pi_i += pll.pi_ki * TS * eq;
    // Обмеження інтегральної частини
    if (pll.pi_i > 50.0f)  pll.pi_i = 50.0f;
    if (pll.pi_i < -50.0f) pll.pi_i = -50.0f;
    
    pll.omega = pll.osg_w0 + pll.pi_kp * eq + pll.pi_i;
    
    // Обмеження частоти захисту (дозволяємо мережі коливатися в межах 45...55 Гц)
    if (pll.omega > 345.5f) pll.omega = 345.5f;
    if (pll.omega < 282.7f) pll.omega = 282.7f;
    
    // 4. Інтегрування кута фази
    pll.theta += pll.omega * TS;
    if (pll.theta >= 2.0f * M_PI) {
        pll.theta -= 2.0f * M_PI;
    }
}

/* ================= TIM1 ШІМ + BURST ================= */
void tim1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC = 0;
    TIM1->ARR = PWM_ARR;
    TIM1->CR1 |= TIM_CR1_CMS_0; // Center-aligned mode 1

    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
    
    // Мертвий час для 180 МГц (значення 150 дає ~830 нс паузи)
    TIM1->BDTR = TIM_BDTR_MOE | (150 << TIM_BDTR_DTG_Pos); 
    
    TIM1->DIER |= TIM_DIER_UDE | TIM_DIER_UIE; // Дозволяємо переривання ШІМ (UIE) та DMA (UDE)

    // Burst: стартуємо з CCR1, оновлюємо 2 регістри (CCR1 та CCR2)
    TIM1->DCR = (13 << TIM_DCR_DBA_Pos) | (1 << TIM_DCR_DBL_Pos); 

    TIM1->CR1 |= TIM_CR1_CEN;
    
    // Дозволяємо переривання таймера в NVIC для прорахунку PLL кожні 50 мкс
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0); // Найвищий пріоритет математики фази
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* ================= ПЕРЕРИВАННЯ ТАЙМЕРА (ПРОРАХУНОК PLL НА 20 кГц) ================= */
volatile float V_grid_amp = 0.0f; // Поточна обчислена амплітуда мережі
#define V_GRID_THRESHOLD  0.2f    // Поріг аварії (якщо напруга впала нижче 20% від норми)

void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR = ~TIM_SR_UIF; 

        // 1. Оцифрування
        int16_t adc_val = (int16_t)ADC1->DR - 2048;
        Vgrid_raw_f = (float)adc_val / 2048.0f;

        // 2. Розрахунок поточної амплітуди мережі через компоненти SOGI
        // (Використовуємо швидку апаратну функцію sqrtf за 1 такт FPU)
        V_grid_amp = sqrtf((pll.v_d * pll.v_d) + (pll.v_q * pll.v_q));

        // 3. Перевірка наявності мережі
        if (V_grid_amp > V_GRID_THRESHOLD)
        {
            // МЕРЕЖА Є: Працює програмний PLL, підлаштовуючи кут під розетку
            grid_present = 1;
            sogi_pll_update(Vgrid_raw_f);
        }
        else
        {
            // МЕРЕЖІ НЕМАЄ (Аварія / Відключення)
            grid_present = 0;
            
            // Фіксуємо частоту строго на 50.00 Гц (314.1592f рад/с)
            pll.omega = 2.0f * M_PI * 50.0f;
            
            // Просто інкрементуємо кут самостійно, перетворюючи інвертор на автономний UPS
            pll.theta += pll.omega * TS;
            if (pll.theta >= 2.0f * M_PI) {
                pll.theta -= 2.0f * M_PI;
            }
            
            // Тут обов'язково повинен бути вихід на піни керування реле, 
            // щоб ФІЗИЧНО відрізати будинок від зовнішньої лінії РЕМ!
            // GPIOA->BSRR = GPIO_BSRR_BR1; // Наприклад, розімкнути реле на PA1
        }
    }
}

/* ================= DMA ШІМ (DMA2, Stream 5, Channel 6) ================= */
void dma_pwm_init(uint16_t *buf)
{
    RCC->AHBENR |= RCC_AHBENR_DMA2EN; // Всі запити TIM1 висять суворо на DMA2!

    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream5->CR & DMA_SxCR_EN);

    DMA2_Stream5->PAR  = (uint32_t)&TIM1->DMAR;
    DMA2_Stream5->M0AR = (uint32_t)buf;
    DMA2_Stream5->NDTR = LUT_SIZE * 2;

    // Конфігурація для F446: Channel 6, 16 біт пам'ять/периферія, інкремент пам'яті, з пам'яті в пер., переривання TC
    DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) | 
                       DMA_SxCR_MINC | 
                       (1 << DMA_SxCR_DIR_Pos) | 
                       (1 << DMA_SxCR_PSIZE_Pos) | // 16 біт
                       (1 << DMA_SxCR_MSIZE_Pos) | // 16 біт
                       DMA_SxCR_TCIE;              // Переривання по закінченню масиву

    NVIC_SetPriority(DMA2_Stream5_IRQn, 1);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    DMA2_Stream5->CR |= DMA_SxCR_EN;
}

void DMA2_Stream5_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF5)
    {
        DMA2->LIFCR = DMA_LIFCR_CTCIF5; 

        // Логіка підготовки подвійного буфера в Normal Mode
        if (active_A) active_A = 0;
        else           active_A = 1;

        dma_ready_to_update = 1; 

        // Перезапуск потоку DMA
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        if (active_A) DMA2_Stream5->M0AR = (uint32_t)bufA;
        else          DMA2_Stream5->M0AR = (uint32_t)bufB;
        DMA2_Stream5->NDTR = LUT_SIZE * 2;
        DMA2_Stream5->CR |= DMA_SxCR_EN;
    }
}

/* ================= ДИНАМІЧНИЙ РОЗРАХУНОК СИНУСА «НА ЛЬОТУ» ЗАВДЯКИ FPU ================= */
void build_buf(uint16_t *buf)
{
    float k;
    float start_theta = pll.theta; 

    if (grid_present)
    {
        // Режим GRID-TIE (підмішування): беремо базовий k_mod
        k = k_mod; 
    }
    else
    {
        // Режим OFF-GRID (UPS): переходимо на жорстке утримання стабільної амплітуди напруги 
        // (Наприклад, тримаємо фіксовано 85% від максимального заповнення для отримання 220 В)
        k = 0.85f; 
        
        // Тут також можна задіяти PI-регулятор напруги зі зворотним зв'язком по АЦП, 
        // як у вашому найпершому варіанті коду.
    }

    for (int i = 0; i < LUT_SIZE; i++)
    {
        float step_angle = start_theta + ((2.0f * M_PI * i) / LUT_SIZE);
        float s = (sinf(step_angle) + 1.0f) * 0.5f;

        buf[i * 2]     = (uint16_t)(s * k * PWM_ARR);
        buf[i * 2 + 1] = (uint16_t)((1.0f - s) * k * PWM_ARR);
    }
}

/* ================= GPIO ІНІЦІАЛІЗАЦІЯ ДЛЯ NUCLEO-F446RE ================= */
void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // 1. PA0 -> Налаштування як Analog для АЦП (Вхід синусоїди мережі)
    GPIOA->MODER |= (3 << GPIO_MODER_MODER0_Pos);

    // 2. PA8 -> TIM1_CH1, PA9 -> TIM1_CH2 (Alternate Function)
    GPIOA->MODER &= ~((3 << GPIO_MODER_MODER8_Pos) | (3 << GPIO_MODER_MODER9_Pos));
    GPIOA->MODER |=  ((2 << GPIO_MODER_MODER8_Pos) | (2 << GPIO_MODER_MODER9_Pos));
    
    // На Nucleo F446 комплементарні виходи зручно вивести на PB13 (CH1N) та PB14 (CH2N)
    GPIOB->MODER &= ~((3 << Bailey_MODER_MODER13_Pos) | (3 << GPIO_MODER_MODER14_Pos));
    GPIOB->MODER |=  ((2 << GPIO_MODER_MODER13_Pos) | (2 << GPIO_MODER_MODER14_Pos));

    // Запис номерів AF (Для TIM1 в серії F4 альтернативна функція — це AF1)
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos) | (1 << GPIO_AFRH_AFSEL9_Pos); 
    GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL13_Pos) | (1 << GPIO_AFRH_AFSEL14_Pos); 
}

/* ================= АЦП (ШВИДКИЙ ОДНОРАЗОВИЙ ЗАПУСК ДЛЯ ТАЙМЕРА) ================= */
void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    ADC1->CR2 |= ADC_CR2_ADON; // Увімкнення АЦП
    ADC->CCR  |= (1 << ADC_CCR_ADCPRE_Pos); // Дільник частоти АЦП (APB2 / 4)
    
    ADC1->SQR3 = 0; // Обираємо канал ADC1_IN0 (ніжка PA0)
}

/* ================= MAIN FUNCTION ================= */
int main(void)
{
    SystemClock_Config(); // Повний розгін до 180 МГц
    
    // Вмикаємо апаратну підтримку FPU копроцесора (Обов'язково для роботи sinf!)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2)); 
    
    gpio_init();
    sogi_pll_init();

    build_buf(bufA);
    build_buf(bufB);

    adc_init();
    tim1_init();
    dma_pwm_init(bufA);

    active_A = 1;

    while (1)
    {
        if (dma_ready_to_update)
        {
            dma_ready_to_update = 0;

            // Завдяки шаленій швидкості ядра, main спокійно перераховує синусоїду
            if (active_A == 0) build_buf(bufA);
            else               build_buf(bufB);
        }
    }
}

/* ================= КОНФІГУРАЦІЯ CLOCK СЕРІЇ F4 НА 180 МГц ВІД КВАРЦУ NUCLEO (8 МГц) ================= */
void SystemClock_Config(void)
{
    // 1. Вмикаємо внутрішній генератор HSI (16 МГц)
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // 2. Вмикаємо тактування модуля керування живленням (Power Interface)
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    // Налаштовуємо внутрішній регулятор надійності на Regulator Voltage Scaling (Mode 1, max частота)
    PWR->CR |= PWR_CR_VOS;

    // 3. Конфігуруємо дільники для периферійних шин:
    // AHB Prescaler  = 1  -> System Clock (180 МГц)
    // APB1 Prescaler = 4  -> 45 МГц (для периферії типу TIM2..7, таймери множаться на 2 = 90 МГц)
    // APB2 Prescaler = 2  -> 90 МГц (для периферії типу TIM1, ADC, таймери множаться на 2 = 180 МГц)
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    // 4. Налаштовуємо головний PLL (від HSI = 16 МГц):
    // Формула: F_VCO = (F_INPUT / M) * N
    // F_SYS = F_VCO / P
    // Обираємо: M = 16 (16MHz / 16 = 1MHz на вході компаратора PLL)
    //           N = 360 (1MHz * 360 = 360MHz для VCO)
    //           P = 2 (360MHz / 2 = 180MHz системна частота) [1.1]
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | 
                   (16  << RCC_PLLCFGR_PLLM_Pos) | 
                   (360 << RCC_PLLCFGR_PLLN_Pos) | 
                   (0   << RCC_PLLCFGR_PLLP_Pos); // 0 відповідає діленню на 2

    // 5. Вмикаємо PLL та чекаємо на його стабілізацію
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // 6. АКТИВАЦІЯ РЕЖИМУ OVER-DRIVE (Обов'язково для F446 на частотах > 168 МГц) [1.1]
    // Вмикаємо Over-drive біт
    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));
    
    // Перемикаємо живлення на роботу з Over-drive
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    // 7. Налаштовуємо Flash Latency (для 180 МГц при 3.3 В потрібно 5 циклів очікування - 5WS)
    // Також вмикаємо апаратний кеш інструкцій та даних (ICEN, DCEN) та Prefetch (PRFTEN)
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    // 8. Перемикаємо джерело системного тактування (SYSCLK) на вихід PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    
    // Чекаємо, поки система підтвердить перехід на PLL
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
