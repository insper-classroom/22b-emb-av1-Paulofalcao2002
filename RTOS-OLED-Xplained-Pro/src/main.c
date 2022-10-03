#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

// Bottoes do OLED
#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

// PINOS do MOTOR
#define IN_1_PIO PIOD
#define IN_1_PIO_ID ID_PIOD
#define IN_1_IDX 30
#define IN_1_IDX_MASK (1u << IN_1_IDX)

#define IN_2_PIO PIOA
#define IN_2_PIO_ID ID_PIOA
#define IN_2_IDX 6
#define IN_2_IDX_MASK (1u << IN_2_IDX)

#define IN_3_PIO PIOC
#define IN_3_PIO_ID ID_PIOC
#define IN_3_IDX 19
#define IN_3_IDX_MASK (1u << IN_3_IDX)

#define IN_4_PIO PIOA
#define IN_4_PIO_ID ID_PIOA
#define IN_4_IDX 2
#define IN_4_IDX_MASK (1u << IN_4_IDX)

/** RTOS  */
#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;
SemaphoreHandle_t xSemaphoreRTT;

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
}

void but1_callback(void) {
  printf("Apertei b1\n");
  int angulo = 45;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}

void but2_callback(void) { 
  printf("Apertei b2\n");
  int angulo = 90;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}

void but3_callback(void) {
  printf("Apertei b3\n");
  int angulo = 180;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}

void RTT_Handler(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
      xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
   }  
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("MOTOR", 0, 0, &sysfont);

	io_init();

	int angulo;
	for (;;)  {
		if (xQueueReceive(xQueueModo, &angulo, 100)) {
			printf("Recebi angulo %d\n", angulo);

			int passos = angulo / 0.17578125;
			printf("Passos %d\n", passos);
			xQueueSend(xQueueSteps, (void *)&passos, 10);

			if (angulo == 45) {
				gfx_mono_draw_filled_rect(0, 20, 128, 10, GFX_PIXEL_CLR);
				gfx_mono_draw_string("MODO 1 (45)", 0, 20, &sysfont);
			} else if (angulo == 90) {
				gfx_mono_draw_filled_rect(0, 20, 128, 10, GFX_PIXEL_CLR);
				gfx_mono_draw_string("MODO 2 (90)", 0, 20, &sysfont);
			} else if (angulo == 180) {
				gfx_mono_draw_filled_rect(0, 20, 128, 10, GFX_PIXEL_CLR);
				gfx_mono_draw_string("MODO 3 (180)", 0, 20, &sysfont);
			}
			
		}

	}
}

static void task_motor(void *pvParameters) {
	int passos;
	int passo = 1;
	int acende = 1;
	for (;;)  {
		if (xQueueReceive(xQueueSteps, &passos, 1)) {
			printf("Recebi %d passos no motor\n", passos);
			RTT_init(1000, 5, RTT_MR_ALMIEN);
		}
		if (xSemaphoreTake(xSemaphoreRTT, 1)) {
			if (passo <= passos) {
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if (acende == 1) {
					pio_clear(IN_1_PIO, IN_1_IDX_MASK);
					pio_set(IN_4_PIO, IN_4_IDX_MASK);

					acende++;
				} else if (acende == 2) {
					pio_set(IN_3_PIO, IN_3_IDX_MASK);
					pio_clear(IN_4_PIO, IN_4_IDX_MASK);

					acende++;
				} else if (acende == 3) {
					pio_set(IN_2_PIO, IN_2_IDX_MASK);
					pio_clear(IN_3_PIO, IN_3_IDX_MASK);
				
					acende++;
				} else if (acende == 4) {
					pio_set(IN_1_PIO, IN_1_IDX_MASK);
					pio_clear(IN_2_PIO, IN_2_IDX_MASK);
					
					acende = 1;
				}
				passo++;
			} else {
				passo = 0;
				acende = 1;
			}
		}

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

void io_init(void) {
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);

	pmc_enable_periph_clk(IN_1_PIO_ID);
	pmc_enable_periph_clk(IN_2_PIO_ID);
	pmc_enable_periph_clk(IN_3_PIO_ID);
	pmc_enable_periph_clk(IN_4_PIO_ID);

	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

	pio_configure(IN_1_PIO, PIO_OUTPUT_0, IN_1_IDX_MASK, PIO_DEFAULT);
	pio_configure(IN_2_PIO, PIO_OUTPUT_0, IN_2_IDX_MASK, PIO_DEFAULT);
	pio_configure(IN_3_PIO, PIO_OUTPUT_0, IN_3_IDX_MASK, PIO_DEFAULT);
	pio_configure(IN_4_PIO, PIO_OUTPUT_0, IN_4_IDX_MASK, PIO_DEFAULT);

	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
	but1_callback);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
	but3_callback);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
	but2_callback);

	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

	pio_get_interrupt_status(BUT_1_PIO);
	pio_get_interrupt_status(BUT_2_PIO);
	pio_get_interrupt_status(BUT_3_PIO);

	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
		
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
		rtt_enable_interrupt(RTT, rttIRQSource);
	else
		rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	xSemaphoreRTT = xSemaphoreCreateBinary();
	if (xSemaphoreRTT == NULL)
		printf("falha em criar o semaforo \n");

	xQueueModo = xQueueCreate(100, sizeof(uint32_t));
  	if (xQueueModo == NULL)
    	printf("falha em criar a queue xQueueModo \n");

	xQueueSteps = xQueueCreate(100, sizeof(uint32_t));
  	if (xQueueSteps == NULL)
    	printf("falha em criar a queue xQueueSteps \n");

	/* Create task to control oled */
	if (xTaskCreate(task_modo, "modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
	  	printf("Failed to create modo task\r\n");
	}

	if (xTaskCreate(task_motor, "motor", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
	  	printf("Failed to create modo task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
