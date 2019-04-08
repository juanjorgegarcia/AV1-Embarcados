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
#define BUT3s_DEBOUNCING_VALUE  79




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



#define sizePneu 325 // raio do pneu em cm;

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile Bool f_rtt_alarme = false;

volatile uint32_t rotations = 0;


/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

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
		
	}
}

/**
*  Handle Interrupcao botao 1
*/
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	rotations+=1;

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
	

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup??o */
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);


	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);



	/* habilita interrup?c?o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 1);

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

int main(void) {
	board_init();
	sysclk_init();	
	configure_lcd();
	
	//font_draw_text(&sourcecodepro_28, "TEST", 50, 50, 1);

	font_draw_text(&calibri_36, "Oi Mundo! #$!@", 50, 100, 1);
	font_draw_text(&arial_72, "102456", 50, 200, 2);
	
	// Desliga watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
		
	sysclk_init();
	io_init();
	
	/* Configura os bot?es */
	BUT_init();
		
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
		
	// super loop
	// aplicacoes embarcadas não devem sair do while(1).
  // aplicacoes embarcadas não devem sair do while(1).
  while (1){
    if (f_rtt_alarme){
      
      /*
       * O clock base do RTT é 32678Hz
       * Para gerar outra base de tempo é necessário
       * usar o PLL pre scale, que divide o clock base.
       *
       * Nesse exemplo, estamos operando com um clock base
       * de pllPreScale = 32768/32768/2 = 2Hz
       *
       * Quanto maior a frequência maior a resolução, porém
       * menor o tempo máximo que conseguimos contar.
       *
       * Podemos configurar uma IRQ para acontecer quando 
       * o contador do RTT atingir um determinado valor
       * aqui usamos o irqRTTvalue para isso.
       * 
       * Nesse exemplo o irqRTTvalue = 8, causando uma
       * interrupção a cada 2 segundos (lembre que usamos o 
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
      
      /*
       * CLEAR FLAG
       */
	  	char buffer[32];
	  	sprintf(buffer,"%d",rotations);
	  	font_draw_text(&calibri_36, buffer, 50, 50, 1);

      f_rtt_alarme = false;
	  
    }
  }  
}