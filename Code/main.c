/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/* Tasks periods */ 
#define Button1_Task_Period          50
#define Button2_Task_Period          50
#define TX_Task_Period               100
#define RX_Task_Period               20
#define LOAD1_Task_Period            10
#define LOAD2_Task_Period            100
/* Queue message datatypes */
typedef enum 
{
	Button_1,
	Button_2,
	Transmitter
}TaskID_t;

typedef enum 
{
	Button_Falling=-1,
	Button_NoChange,
	Button_Rising
}ButtonStatus_t;

/* Queue message struct */
typedef struct 
{
	TaskID_t TaskID;
	ButtonStatus_t Button_Status;
	char String[sizeof("Monitoring buttons...!")];
}Message_t;


/* Variables for calculations */
int TasksInTime[6]={0,0,0,0,0,0}, 
		TasksOutTime[6]={0,0,0,0,0,0}, 
		TasksTotalTime[6]={0,0,0,0,0,0},
		Task_Loads[6]={0,0,0,0,0,0};
		
int System_Time   =0,CPU_Load       =0;

/* Tasks handles */
TaskHandle_t Button1_TaskHandle = NULL;
TaskHandle_t Button2_TaskHandle = NULL;
TaskHandle_t TX_TaskHandle = NULL;
TaskHandle_t RX_TaskHandle = NULL;
TaskHandle_t Load1_TaskHandle = NULL;
TaskHandle_t Load2_TaskHandle = NULL;

/* Queue handles */
QueueHandle_t Consumer_Queue=NULL;

/* Tick hook */
void vApplicationTickHook( void )
{
      GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
			GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
}

/* Idle task hook */
void vApplicationIdleHook( void )
{
      GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
}

/* Tasks prototypes */
/*  Button 1 monitoring task */
void Button_1_Monitor( void * pvParameters );

/*  Button 2 monitoring task */
void Button_2_Monitor( void * pvParameters );

/*  UART transmitter task */
void Task_Transmitter( void * pvParameters );
	
/*  UART receiver task */
void Uart_Receiver( void * pvParameters );
	
/* Load task 1 with 5 ms execution time */
void Load_1_Simulation( void * pvParameters );

/* Load task 2 with 12 ms execution time */
void Load_2_Simulation( void * pvParameters );

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	/* Queues creation */
	Consumer_Queue=xQueueCreate(
													 /* The number of items the queue can hold. */
													 200,
													 /* Size of each item is big enough to hold the
													 whole structure. */
													 sizeof( Message_t ) );
	
  /* Create Tasks here */

	xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button1_TaskHandle,
										Button1_Task_Period/*period task */ );      /* Used to pass out the created task's handle. */					
			
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button2_TaskHandle,
										Button2_Task_Period /*period task */);      /* Used to pass out the created task's handle. */											
											
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Task_Transmitter,       /* Function that implements the task. */
                    "Transmitter",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &TX_TaskHandle,
										TX_Task_Period /*period task */  );      /* Used to pass out the created task's handle. */		
										
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "uart",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &RX_TaskHandle,
										RX_Task_Period /*period task */  );      /* Used to pass out the created task's handle. */		
										
                    /* Create the task, storing the handle. */
  xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load1_TaskHandle,
										LOAD1_Task_Period /*period task */);      /* Used to pass out the created task's handle. */	
										
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load2_TaskHandle,	
										LOAD2_Task_Period /*period task */ );      /* Used to pass out the created task's handle. */			

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

/*  Button 1 monitoring task */
void Button_1_Monitor( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Create message struct */
	Message_t B1_StateMsg;
	/* Create state variables and set initial values */
	pinState_t B1_State=PIN_IS_LOW;
	pinState_t B1_PrevState = GPIO_read(PORT_0, PIN0);
	/* Assign task ID to message struct */
	B1_StateMsg.TaskID=Button_1;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 20 );
	for( ;; )
	{
		/* Read GPIO pin and insert status into message*/
	  B1_State = GPIO_read(PORT_0, PIN0);
		B1_StateMsg.Button_Status=(ButtonStatus_t)((uint8_t)B1_State-(uint8_t)B1_PrevState);
		/* Send message to Consumer_Queue */
		if( B1_StateMsg.Button_Status != Button_NoChange && Consumer_Queue != NULL)
   {
      xQueueSend( Consumer_Queue,(void * ) &B1_StateMsg,(TickType_t) 0 );
   }
	 B1_PrevState=B1_State;
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , Button1_Task_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	}
}

/*  Button 2 monitoring task */
void Button_2_Monitor( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Create message struct */
	Message_t B2_StateMsg;
	/* Create state variables and set initial values */
	pinState_t B2_State=PIN_IS_LOW;
	pinState_t B2_PrevState = GPIO_read(PORT_0, PIN1);
	/* Assign task ID to message struct */
	B2_StateMsg.TaskID=Button_2;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 21 );
	for( ;; )
	{
		/* Read GPIO pin and insert status into message*/
	  B2_State = GPIO_read(PORT_0, PIN1);
		B2_StateMsg.Button_Status=(ButtonStatus_t)((uint8_t)B2_State-(uint8_t)B2_PrevState);
		/* Send message to Consumer_Queue */
		if( B2_StateMsg.Button_Status != Button_NoChange && Consumer_Queue != NULL )
   {
      xQueueSend( Consumer_Queue,(void * ) &B2_StateMsg,(TickType_t) 0 );
   }
	 B2_PrevState=B2_State;
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , Button2_Task_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	}
}

/*  UART transmitter task */
void Task_Transmitter( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Create message struct */
	Message_t TX_Msg;
	/* Periodic string */
	volatile char PeriodicMsg[]="\nMonitoring buttons...!";
	/* Assign string to message struct */
	strcpy( TX_Msg.String, PeriodicMsg );
	/* Assign button ID to message struct */
	TX_Msg.TaskID=Transmitter;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 22 );
 for( ;; )
	{
		/* Send message to Consumer_Queue */
		if( Consumer_Queue != NULL )
		 {
				xQueueSend( Consumer_Queue,(void * ) &TX_Msg,(TickType_t) 0 );
		 }
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , TX_Task_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	}
}

/*  UART receiver task */
void Uart_Receiver( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Create message struct */
	Message_t RX_Msg;
	/* Last UART msg flag */
	TaskID_t SentFlag;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 23 );
	for( ;; )
	{
	 /* Send message to Consumer_Queue */
	 if( Consumer_Queue != NULL )
	 {
			xQueueReceive( Consumer_Queue, &RX_Msg,(TickType_t) 0 );
	 }
	 /* Check received message task ID */
	 if(RX_Msg.TaskID==Transmitter && SentFlag != Transmitter)
	 {
		 vSerialPutString((const signed char * const)RX_Msg.String,sizeof(RX_Msg.String));
		 SentFlag = Transmitter;
	 }
	 else if (RX_Msg.TaskID==Button_1 && SentFlag != Button_1)
	 {
		 if (RX_Msg.Button_Status==Button_Rising)
		 {
			vSerialPutString((const signed char * const)"\nButton 1 is ON!",sizeof("\nButton 1 is OFF!"));
		 }
		 else if (RX_Msg.Button_Status==Button_Falling)
		 {
			vSerialPutString((const signed char * const)"\nButton 1 is OFF!",sizeof("\nButton 1 is OFF!"));
		 }
		 SentFlag = Button_1;
	 }
	 else if (RX_Msg.TaskID==Button_2 && SentFlag != Button_2)
	 {
		 if (RX_Msg.Button_Status==Button_Rising)
		 {
			vSerialPutString((const signed char * const)"\nButton 2 is ON!",sizeof("\nButton 2 is OFF!"));
		 }
		 else if (RX_Msg.Button_Status==Button_Falling)
		 {
			vSerialPutString((const signed char * const)"\nButton 2 is OFF!",sizeof("\nButton 2 is OFF!"));
		 }
		 SentFlag = Button_2;
	 }
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , RX_Task_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	}
}
	
/* Load task 1 with 5 ms execution time */
void Load_1_Simulation( void * pvParameters )
{		
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* calculate delay time using 
	(XTAL / 1000U)*time_in_ms  */
	uint32_t delay = 12000*5;  
	/* For loop iterator */
	uint32_t i=0;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 24 );
	for( ;; )
	{
		for(i=0;i<=delay;i++)
		{
			/* Do something to avoid 
			removing for loop for optimization */
			//i=i;
		}
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , LOAD1_Task_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	}
}

/* Load task 2 with 12 ms execution time */
void Load_2_Simulation( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* calculate delay time using equation
	(XTAL / 1000U)*time_in_ms  */
	uint32_t delay = 12000*12;  
	/* For loop iterator */
	uint32_t i=0;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 25 );
	for( ;; )
	{
		for(i=0;i<=delay;i++)
		{
			/* Do something to avoid 
			removing for loop for optimization */
			//i=i;
		}
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , LOAD2_Task_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	}
}

