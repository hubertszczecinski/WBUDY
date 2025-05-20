#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_timer.h"
#include "oled.h"
#include "temp.h"
#include "pca9532.h"
#include "light.h"
#include "joystick.h"
#include "rgb.h"


#define PWM_FREQUENCY 1000
#define LIGHT_THRESHOLD 300
#define TIMEPRESCALE (25000-1)
uint8_t night_mode = 0xFF;

/*******************
funkcjonalności:
1. oled
2. spiu i/f oled
3. pwm (motor)
4. i2c i/f
5. trimpot przetw. ADC
6. temperatura czujnik max
7. czujnik światła lux
8. timer
9.gpio przycisk dioda on/off

*/



/*!
 *  @brief    Inicjalizuje moduł PWM (Pulse Width Modulation) dla kanału 1 na mikrokontrolerze LPC17xx.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            Konfiguruje pin P2.0 jako wyjście PWM, ustawia częstotliwość PWM na 250 Hz i włącza moduł PWM.
 */

void PWM_Init(void) {
    LPC_PINCON->PINSEL4 |= (1 << 0);
    LPC_PWM1->MR0 = SystemCoreClock / PWM_FREQUENCY;
    LPC_PWM1->MR1 = 0;
    LPC_PWM1->MCR |= (1 << 1);
    LPC_PWM1->LER |= (1 << 0) | (1 << 1);
    LPC_PWM1->PCR |= (1 << 9);
    LPC_PWM1->TCR = (1 << 0) | (1 << 3);
}

/*!
 *  @brief    Inicjalizuje przetwornik analogowo-cyfrowy (ADC) dla kanału 0 na pinie P0.23.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            Konfiguruje pin P0.23 jako wejście analogowe, ustawia częstotliwość próbkowania ADC na 200 kHz i włącza kanał 0.
 */

void adc_init(void) {
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 23;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, DISABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

/*!
 *  @brief    Odczytuje wartość z potencjometru podłączonego do kanału 0 ADC.
 *  @param    void
 *  @returns  uint16_t
 *            Wartość ADC w zakresie 0-4095.
 *  @side effects:
 *            Uruchamia konwersję ADC i czeka na jej zakończenie, wprowadzając opóźnienie 200 ms.
 */

uint16_t Read_Potentiometer(void) {
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while(!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE))){
    	delayms(200);
    	return ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
    }
}

/*!
 *  @brief    Inicjalizuje Timer0 do generowania opóźnień w milisekundach.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            Konfiguruje Timer0 z preskalerem dla częstotliwości 25 MHz i resetuje licznik.
 */

static void Timer0_Init(void){
	LPC_TIM0->CTCR=0x0;
	LPC_TIM0->PR= TIMEPRESCALE;
	LPC_TIM0->TCR= 0x02;
}

/*!
 *  @brief    Generuje opóźnienie w milisekundach przy użyciu Timer0.
 *  @param    milliseconds
 *            Czas opóźnienia w milisekundach.
 *  @returns  void
 *  @side effects:
 *            Blokuje wykonanie programu na zadany czas, resetując i zatrzymując Timer0 po zakończeniu.
 */

void delayms(unsigned int milliseconds){
	LPC_TIM0-> TCR = 0x02;
	LPC_TIM0-> TCR = 0x01;
	while(LPC_TIM0-> TC < milliseconds){}
	LPC_TIM0-> TCR= 0x00;
}

static uint32_t msTicks = 0;
static uint8_t buf[10];

/*!
 *  @brief    Obsługuje przerwanie SysTick, zwiększając licznik milisekund.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            Inkrementuje globalną zmienną msTicks, używaną do śledzenia czasu.
 */

void SysTick_Handler(void) {
    msTicks++;
}

/*!
 *  @brief    Zwraca aktualną wartość licznika milisekund.
 *  @param    void
 *  @returns  uint32_t
 *            Aktualna wartość zmiennej msTicks.
 *  @side effects:
 *            Brak.
 */

static uint32_t getTicks(void) {
    return msTicks;
}

/*!
 *  @brief    Inicjalizuje interfejs SSP (SPI) dla komunikacji z wyświetlaczem OLED.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            Konfiguruje piny P0.7, P0.8, P0.9 jako SSP oraz P2.2 jako GPIO, inicjalizuje moduł SSP1 i włącza go.
 */

static void init_ssp(void) {
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 7;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 8;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 9;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Funcnum = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    SSP_ConfigStructInit(&SSP_ConfigStruct);
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
    SSP_Cmd(LPC_SSP1, ENABLE);
}

/*!
 *  @brief    Inicjalizuje interfejs I2C dla komunikacji z czujnikami i ekspanderem PCA9532.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            Konfiguruje piny P0.10 i P0.11 jako SCL2 i SDA2, ustawia częstotliwość I2C na 100 kHz i włącza moduł I2C2.
 */

static void init_i2c(void) {
    PINSEL_CFG_Type PinCfg;

    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    I2C_Init(LPC_I2C2, 100000);
    I2C_Cmd(LPC_I2C2, ENABLE);
}

/*!
 *  @brief    Konwertuje liczbę całkowitą na ciąg znaków w podanej bazie.
 *  @param    value
 *            Liczba do konwersji.
 *  @param    pBuf
 *            Bufor na wynikowy ciąg znaków.
 *  @param    len
 *            Maksymalna długość bufora.
 *  @param    base
 *            Baza systemu liczbowego (2-36).
 *  @returns  void
 *  @side effects:
 *            Zapisuje ciąg znaków do bufora pBuf, obsługuje liczby ujemne i kończy ciąg znakiem null.
 */

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base) {
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    if (pBuf == NULL || len < 2) return;
    if (base < 2 || base > 36) return;

    if (value < 0) {
        tmpValue = -tmpValue;
        value    = -value;
        pBuf[pos++] = '-';
    }

    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);

    if (pos > len) return;

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = pAscii[value % base];
        value /= base;
    } while(value > 0);
}

/*!
 *  @brief    Aktualizuje wyświetlacz OLED, pokazując temperaturę i natężenie światła.
 *  @param    temp
 *            Wartość temperatury do wyświetlenia.
 *  @param    light
 *            Wartość natężenia światła do wyświetlenia.
 *  @param    light_value
 *            Wartość natężenia światła do określenia trybu ekranu (dzień/noc).
 *  @returns  void
 *  @side effects:
 *            Czyści obszary wyświetlacza i wyświetla temperaturę oraz światło w odpowiednich kolorach w zależności od progu LIGHT_THRESHOLD.
 */

void update_display(int32_t temp, uint32_t light, uint32_t light_value) {
    // Czyszczenie obszarów wyświetlania
    oled_fillRect((1+6*6), 20, 80, 28,
                 light_value > LIGHT_THRESHOLD ? OLED_COLOR_BLACK : OLED_COLOR_WHITE);
    oled_fillRect((1+7*6), 30, 80, 38,
                 light_value > LIGHT_THRESHOLD ? OLED_COLOR_BLACK : OLED_COLOR_WHITE);

    // Wyświetlanie temperatury
    intToString(temp, buf, 10, 10);
    oled_putString((1+6*6), 20, buf,
                  light_value > LIGHT_THRESHOLD ? OLED_COLOR_WHITE : OLED_COLOR_BLACK,
                  light_value > LIGHT_THRESHOLD ? OLED_COLOR_BLACK : OLED_COLOR_WHITE);

    // Wyświetlanie światła
    intToString(light, buf, 10, 10);
    oled_putString((1+7*6), 30, buf,
                  light_value > LIGHT_THRESHOLD ? OLED_COLOR_WHITE : OLED_COLOR_BLACK,
                  light_value > LIGHT_THRESHOLD ? OLED_COLOR_BLACK : OLED_COLOR_WHITE);
}

/*!
 *  @brief    Obsługuje joystick do sterowania diodami RGB.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            W zależności od pozycji joysticka włącza lub wyłącza diody RGB (czerwona, zielona, niebieska).
 */

void joystic_colors(){
	 uint8_t r = 0;
	   uint8_t g = 0;
	   uint8_t b = 0;

	   uint8_t joy = 0;

	  joy = joystick_read();

	        if ((joy & JOYSTICK_CENTER) != 0) {
	            rgb_setLeds(RGB_RED|RGB_GREEN|RGB_BLUE);
	            r = RGB_RED;
	            b = RGB_BLUE;
	            g = RGB_GREEN;
	        }

	        if ((joy & JOYSTICK_DOWN) != 0) {
	            rgb_setLeds(0);
	            r = g = b = 0;
	        }

	        if ((joy & JOYSTICK_LEFT) != 0) {
	            r = !r;
	            rgb_setLeds(r|g|b);
	        }

	        if ((joy & JOYSTICK_UP) != 0) {
	            if(g)
	                g = 0;
	            else
	                g = RGB_GREEN;
	            rgb_setLeds(r|g|b);
	        }

	        if ((joy & JOYSTICK_RIGHT) != 0) {
	            if(b)
	                b = 0;
	            else
	                b = RGB_BLUE;
	            rgb_setLeds(r|g|b);
	        }
}

/*!
 *  @brief    Inicjalizuje ekran OLED, czujniki i inne peryferia systemu.
 *  @param    void
 *  @returns  void
 *  @side effects:
 *            Inicjalizuje interfejsy I2C i SSP, czujnik światła, PCA9532, temperaturę, wyświetlacz OLED oraz konfiguruje SysTick dla timera.
 */

void screen_init(void){
    init_i2c();
    init_ssp();

    light_setRange(LIGHT_RANGE_4000);

    oled_init();
    temp_init(&getTicks);
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);
    }

    oled_clearScreen(OLED_COLOR_WHITE);
    oled_putString(1, 1, (uint8_t*)"       A05 ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1, 10, (uint8_t*)"Klimatyzator ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1, 20, (uint8_t*)"Temp: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1, 30, (uint8_t*)"Light: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

/*!
 *  @brief    Aktualizuje tryb wyświetlania ekranu OLED (dzień/noc).
 *  @param    mode
 *            0 - tryb dzienny, 1 - tryb nocny.
 *  @returns  void
 *  @side effects:
 *            Czyści ekran i wyświetla tekst nagłówka oraz etykiety temperatury w odpowiednich kolorach (biały/czarny).
 */

void update_screen_mode(uint8_t mode) {
    // mode: 0 - dzień, 1 - noc
	oled_color_t text_color = mode ? OLED_COLOR_WHITE : OLED_COLOR_BLACK;
	oled_color_t bg_color = mode ? OLED_COLOR_BLACK : OLED_COLOR_WHITE;

    oled_clearScreen(bg_color);
    oled_putString(1, 1,  (uint8_t*)"       A05 ", text_color, bg_color);
    oled_putString(1, 10, (uint8_t*)"Klimatyzator ", text_color, bg_color);
    oled_putString(1, 20, (uint8_t*)"Temperatura: ", text_color, bg_color);
}

/*!
 *  @brief    Główna pętla programu sterującego klimatyzatorem.
 *  @param    void
 *  @returns  int
 *            Zwraca 0 (nigdy nie osiągane z powodu nieskończonej pętli).
 *  @side effects:
 *            Inicjalizuje peryferia, odczytuje dane z potencjometru, czujnika światła i temperatury,
 *            steruje PWM, diodami RGB, LED i ekranem OLED w zależności od warunków oświetlenia.
 */

int main(void) {
    PWM_Init();
    adc_init();
    screen_init();
    Timer0_Init();
    rgb_init();
    light_enable();
    joystick_init();
    int32_t t = 0;
    uint32_t lux = 0;


    while(1) {
        uint16_t adc_value = Read_Potentiometer();
        LPC_PWM1->MR1 = (adc_value * LPC_PWM1->MR0) / 4095;
        LPC_PWM1->LER |= (1 << 1);

        lux = light_read();
        joystic_colors();
        uint8_t current_mode = (lux < 500) ? 1 : 0;
        if (current_mode != night_mode) {
        				night_mode = current_mode;
        				update_screen_mode(night_mode);
        			}

        			t = temp_read();
        			intToString(t, buf, 10, 10);

        			// Aktualizacja tekstu temperatury z odpowiednim kolorem
        			oled_color_t txt_color = night_mode ? OLED_COLOR_WHITE : OLED_COLOR_BLACK;
        			oled_color_t bg_color  = night_mode ? OLED_COLOR_BLACK : OLED_COLOR_WHITE;

        			oled_fillRect((1+12*6), 20, 80, 8, bg_color);
        			oled_putString((1+12*6), 20, buf, txt_color, bg_color);

        			delayms(200);

    }
}
