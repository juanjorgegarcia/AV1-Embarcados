/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"
#include "stdio.h"


struct ili9488_opt_t g_ili9488_display_opt;

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

#define BUT1_PIO_ID			  ID_PIOD
#define BUT1_PIO				  PIOD
#define BUT1_PIN				  28
#define BUT1_PIN_MASK			  (1 << BUT1_PIN)
#define BUT1_DEBOUNCING_VALUE  79

#define BUT2_PIO_ID			  ID_PIOC
#define BUT2_PIO				  PIOC
#define BUT2_PIN				  31
#define BUT2_PIN_MASK			  (1 << BUT2_PIN)
#define BUT2s_DEBOUNCING_VALUE  20




/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0



#define sizePneu 0.325 // raio do pneu em cm;

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile Bool f_rtt_alarme = false;

volatile uint32_t rotations = 0;

volatile uint32_t speed = 0;

volatile uint32_t distance = 0;

volatile uint32_t hour = 0;
volatile uint32_t minute = 0;
volatile uint32_t second = 0;


volatile uint8_t running = 1;






/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void configure_lcd(void);


/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
		if (running){
			speed = (float) (((2*3.14*rotations)/4)*sizePneu)*3.6;
			distance += (float) 2*3.14*sizePneu*rotations;

			rotations = 0;
		}

		
	}
}

/**
*  Handle Interrupcao botao 1
*/
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	if(running){
		rotations+=1;
	}

}

/**
*  Handle Interrupcao botao 1
*/
static void Button2_Handler(uint32_t id, uint32_t mask)
{
	running = !running;
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
/**
* @Brief Inicializa o pino do BUT
*/
void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO,BUT1_PIN_MASK,BUT2s_DEBOUNCING_VALUE);


	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup??o */
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);





	/* habilita interrup?c?o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 3);
	
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO,BUT2_PIN_MASK,BUT2s_DEBOUNCING_VALUE);


	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup??o */
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);





	/* habilita interrup?c?o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 3);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);

	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_FALL_EDGE, Button2_Handler);


};



void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}

void draw_display() {
	      /*
       * CLEAR FLAG
       */
	

	font_draw_text(&calibri_36, "Velocidade:", 50, 0, 1);
	char speed_buffer[32];
	sprintf(speed_buffer,"%d",speed);
	font_draw_text(&calibri_36, speed_buffer, 50, 50, 1);
	font_draw_text(&calibri_36, "Distancia:", 50, 100, 1);

	char distance_buffer[32];
	sprintf(distance_buffer,"%d",distance);
	font_draw_text(&calibri_36, distance_buffer, 50, 150, 2);
	
	char hours_buffer[32];
	sprintf(hours_buffer,"%d",hour);
	char minutes_buffer[32];
	sprintf(minutes_buffer,"%d",minute);
	char seconds_buffer[32];
	sprintf(seconds_buffer,"%d",second);
	font_draw_text(&calibri_36, "Horas:", 50, 200, 2);
	font_draw_text(&calibri_36, hours_buffer, 50, 250, 2);
	font_draw_text(&calibri_36, "Minutos:", 50, 300, 2);
	font_draw_text(&calibri_36, minutes_buffer, 50, 350, 2);
	font_draw_text(&calibri_36, "Segundos:", 50, 400, 2);
	font_draw_text(&calibri_36, seconds_buffer, 50, 450, 2);
		
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}


/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
 			rtc_get_time(RTC,&hour,&minute,&second);

			if (running){
			

 			if (second >= 59){
	 			minute += 1;
	 			second = 0;
	 			}else{
	 			second += 1;
 			}
			if(minute>=59){
				hour+=1;
				minute=0;
			}

 			rtc_set_time_alarm(RTC, 1, hour, 1, minute, 1, second);
			hour -= HOUR;
			minute -= MINUTE;
			second -= SECOND;


		 }
		 configure_lcd();





	}
	
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

int main(void) {
	board_init();
	sysclk_init();	
	
	//font_draw_text(&sourcecodepro_28, "TEST", 50, 50, 1);

	//font_draw_text(&calibri_36, "Oi Mundo! #$!@", 50, 100, 1);
	//font_draw_text(&arial_72, "102456", 50, 200, 2);
	
	// Desliga watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
		
	sysclk_init();
	io_init();
	
	/* Configura os bot?es */
	BUT_init();
	
	RTC_init();

	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE, 1, SECOND+1);	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
	
	rotations = 0;
	distance = 0;
	second = 0;
	running = 1;

	// super loop
	// aplicacoes embarcadas n�o devem sair do while(1).
  while (1){
    if (f_rtt_alarme){
      
      /*
       * O clock base do RTT � 32678Hz
       * Para gerar outra base de tempo � necess�rio
       * usar o PLL pre scale, que divide o clock base.
       *
       * Nesse exemplo, estamos operando com um clock base
       * de pllPreScale = 32768/32768/2 = 2Hz
       *
       * Quanto maior a frequ�ncia maior a resolu��o, por�m
       * menor o tempo m�ximo que conseguimos contar.
       *
       * Podemos configurar uma IRQ para acontecer quando 
       * o contador do RTT atingir um determinado valor
       * aqui usamos o irqRTTvalue para isso.
       * 
       * Nesse exemplo o irqRTTvalue = 8, causando uma
       * interrup��o a cada 2 segundos (lembre que usamos o 
       * pllPreScale, cada incremento do RTT leva 500ms (2Hz).
       */
      uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
      uint32_t irqRTTvalue  = 8;
      
      // reinicia RTT para gerar um novo IRQ
      RTT_init(pllPreScale, irqRTTvalue);         
      
     /*
      * caso queira ler o valor atual do RTT, basta usar a funcao
      *   rtt_read_timer_value()
      */
      
		
      f_rtt_alarme = false;
	  
    }
	
	draw_display();
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);


  }  
}