/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

/*
*********************************************************************************************************
*
*                                             EXAMPLE MAIN
*
* File : main.c
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*********************************************************************************************************
*/

#include  <bsp_os.h>
#include  "bsp.h"

#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>


#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_core.h"
#include "em_acmp.h"


#include "capsense.h"
#include "app.h"
#include "fifo.h"
#include "display.h"
#include "glib.h"



/*
*********************************************************************************************************
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*********************************************************************************************************
*/

#define  MAIN_START_TASK_PRIO              		20u
#define  MAIN_FORCE_TASK_PRIO       	        21u
#define  MAIN_PHYSICS_TASK_PRIO    				22u
#define  MAIN_GAIN_TASK_PRIO	    			23u
#define  MAIN_LCD_DISPLAY_TASK_PRIO    			24u
#define  MAIN_IDLE_TASK_PRIO              		25u

#define  MAIN_START_TASK_STK_SIZE         		512u
#define  MAIN_PHYSICS_TASK_STK_SIZE			 	512u
#define  MAIN_FORCE_TASK_STK_SIZE	    		512u
#define  MAIN_GAIN_TASK_STK_SIZE				512u
#define  MAIN_LCD_DISPLAY_TASK_STK_SIZE	    	512u
#define  MAIN_IDLE_TASK_STK_SIZE				512u
/*
*********************************************************************************************************
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/

/* Task Stacks.                                    */
static  CPU_STK  MainStartTaskStk[MAIN_START_TASK_STK_SIZE];
/* Task TCBs.                                      */
static  OS_TCB   MainStartTaskTCB;

// Idle tasks
static  CPU_STK  MainIdleTaskStk[MAIN_IDLE_TASK_STK_SIZE];
static  OS_TCB   MainIdleTaskTCB;

//Display Task
static  CPU_STK  MainLcdDisplayTaskStk[MAIN_LCD_DISPLAY_TASK_STK_SIZE];
static  OS_TCB   MainLcdDisplayTaskTCB;

//Vehicle Task
static  CPU_STK  MainGainTaskStk[MAIN_GAIN_TASK_STK_SIZE];
static  OS_TCB   MainGainTaskTCB;
static  CPU_STK  MainForceTaskStk[MAIN_FORCE_TASK_STK_SIZE];
static  OS_TCB   MainForceTaskTCB;
static  CPU_STK  MainPhysicsTaskStk[MAIN_PHYSICS_TASK_STK_SIZE];
static  OS_TCB   MainPhysicsTaskTCB;



//  OS Task Flags
static OS_FLAG_GRP	input_flag;
static OS_FLAG_GRP	output_flag;

//  Semaphores
static OS_SEM		timer_sem;
static OS_SEM		fifo_sem;

//  Mutexes
static OS_MUTEX  	gain_mutex;
static OS_MUTEX  	physics_mutex;

//  Timer
static OS_TMR		tmr;

//  Message Queue
static OS_Q			queue;
//static OS_MSG_SIZE  msg_size = sizeof(int);


//  Enum to store directional states
enum force {
	none,
	far_left,
	left,
	right,
	far_right,
};

//  Button states
#define FIFO_LEN	10
uint8_t btn0_fifo[FIFO_LEN];
uint8_t btn1_fifo[FIFO_LEN];
uint8_t btn0_fifo_rd;
uint8_t btn0_fifo_wr;
uint8_t btn1_fifo_rd;
uint8_t btn1_fifo_wr;

// gain and force
uint32_t gain_data;
double force_data;

// physics ints
struct physics_data_struct {
	double gravity;
	double bob;
	double cart;
	double len;
	uint32_t graph_lim;
	uint32_t xmin;
	uint32_t xmax;
	double time;
	double theta;
	double x_pos;
	double w;
	double v;
} physics_data;

/* Global glib context */
GLIB_Context_t gc;

//  time
static volatile uint32_t msTicks; /* counts 1ms timeTicks */
static volatile uint32_t start = 0; /* counts 1ms timeTicks */




/*
*********************************************************************************************************
*********************************************************************************************************
*                                       LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*********************************************************************************************************
*/

// task main functions
static  void  MainStartTask (void  *p_arg);
static  void  MainIdleTask (void  *p_arg);
static  void  MainPhysicsTask (void  *p_arg);
static  void  MainForceTask (void  *p_arg);
static  void  MainGainTask (void  *p_arg);
static  void  MainLcdDisplayTask (void  *p_arg);
// callback function for OSTimer
static void MyCallback(OS_TMR p_tmr, void *p_arg);


/*
*********************************************************************************************************
*********************************************************************************************************
*                                          GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C applications. It is assumed that your code will
*               call main() once you have performed all necessary initialization.
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

double dthetadt(double w) {
    return w;
}

double dxdt(double v) {
    return v;
}

double dwdt(double F, double theta, double w, double L, double M, double m, double g) {
    return ( 1/((M+m) - m*cos(theta)*cos(theta)) * (F*cos(theta)/L + (g/L)*((M+m)*sin(theta)) - m*w*w*sin(theta)*cos(theta)) );
}

double dvdt(double F, double theta, double w, double L, double M, double m, double g) {
    return ( 1/( cos(theta)*cos(theta) - (M+m)/(m)) * ( (w*w*L*sin(theta) - F/(m) - g*sin(theta)*cos(theta)) ) );
}

int  main (void)
{
    RTOS_ERR  err;

    BSP_SystemInit();                                           /* Initialize System.                                   */
    CPU_Init();                                                 /* Initialize CPU.                                      */

    OS_TRACE_INIT();
    OSInit(&err);                                               /* Initialize the Kernel.                               */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSTaskCreate(&MainStartTaskTCB,                          /* Create the Start Task.                               */
                 "Main Start Task",
                  MainStartTask,
                  DEF_NULL,
                  MAIN_START_TASK_PRIO,
                 &MainStartTaskStk[0],
                 (MAIN_START_TASK_STK_SIZE / 10u),
                  MAIN_START_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

/*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSStart(&err);                                              /* Start the kernel.                                    */
/*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    return (1);
}


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/





RTOS_ERR  err;
static  void  MainStartTask (void  *p_arg)
{

    PP_UNUSED_PARAM(p_arg);        /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    Common_Init(&err);                                          /* Call common module initialization example.           */
    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

    BSP_OS_Init();                                              /* Initialize the BSP */

    // initialize common tasks
        cmu_open();
        gpio_open();
        /* Create Idle Task.                               */
        OSTaskCreate(&MainIdleTaskTCB,
                     "Main Idle Task",
                      MainIdleTask,
                      DEF_NULL,
                      MAIN_IDLE_TASK_PRIO,
                     &MainIdleTaskStk[0],
                     (MAIN_IDLE_TASK_STK_SIZE / 10u),
                      MAIN_IDLE_TASK_STK_SIZE,
                      0u,
                      0u,
                      DEF_NULL,
                     (OS_OPT_TASK_STK_CLR),
                     &err);
        /* create Speed Setpoint                               */
        OSTaskCreate(&MainPhysicsTaskTCB,
                     "Main Physics Task",
                      MainPhysicsTask,
                      DEF_NULL,
                      MAIN_PHYSICS_TASK_PRIO,
                     &MainPhysicsTaskStk[0],
                     (MAIN_PHYSICS_TASK_STK_SIZE / 10u),
                      MAIN_PHYSICS_TASK_STK_SIZE,
                      0u,
                      0u,
                      DEF_NULL,
                     (OS_OPT_TASK_STK_CLR),
                     &err);
        /* create Vehicle Direction                               */
        OSTaskCreate(&MainForceTaskTCB,
                     "Main Force Task",
                      MainForceTask,
                      DEF_NULL,
                      MAIN_FORCE_TASK_PRIO,
                     &MainForceTaskStk[0],
                     (MAIN_FORCE_TASK_STK_SIZE / 10u),
                      MAIN_FORCE_TASK_STK_SIZE,
                      0u,
                      0u,
                      DEF_NULL,
                     (OS_OPT_TASK_STK_CLR),
                     &err);
        /* create Vehicle Monitor                               */
        OSTaskCreate(&MainGainTaskTCB,
                     "Main Gain Task",
                      MainGainTask,
                      DEF_NULL,
                      MAIN_GAIN_TASK_PRIO,
                     &MainGainTaskStk[0],
                     (MAIN_GAIN_TASK_STK_SIZE / 10u),
                      MAIN_GAIN_TASK_STK_SIZE,
                      0u,
                      0u,
                      DEF_NULL,
                     (OS_OPT_TASK_STK_CLR),
                     &err);
        /* create LCD Display                               */
        OSTaskCreate(&MainLcdDisplayTaskTCB,
                     "Main LCD Display Task",
                      MainLcdDisplayTask,
                      DEF_NULL,
                      MAIN_LCD_DISPLAY_TASK_PRIO,
                     &MainLcdDisplayTaskStk[0],
                     (MAIN_LCD_DISPLAY_TASK_STK_SIZE / 10u),
                      MAIN_LCD_DISPLAY_TASK_STK_SIZE,
                      0u,
                      0u,
                      DEF_NULL,
                     (OS_OPT_TASK_STK_CLR),
                     &err);
        // create event flags
        OSFlagCreate((OS_FLAG_GRP *)
        		&input_flag,
    			(CPU_CHAR *) "input flag",
    			(OS_FLAGS) 0,
    			(RTOS_ERR *)&err);
        OSFlagCreate((OS_FLAG_GRP *)
        		&output_flag,
    			(CPU_CHAR *) "output flag",
    			(OS_FLAGS) 0,
    			(RTOS_ERR *)&err);
        // Creates semaphores
        OSSemCreate ((OS_SEM *) &timer_sem,
                (CPU_CHAR *) "Slider Timer Semaphore",
                (OS_SEM_CTR) 0,
                (RTOS_ERR *) &err);
        OSSemCreate ((OS_SEM *) &fifo_sem,
                (CPU_CHAR *) "FIFO Semaphore",
                (OS_SEM_CTR) 0,
                (RTOS_ERR *) &err);
        // Create OSTimer
        OSTmrCreate ((OS_TMR *) &tmr,
    			(CPU_CHAR *) "OS Timer",
    			(OS_TICK) 5,
    			(OS_TICK) 5,
    			(OS_OPT) OS_OPT_TMR_PERIODIC,
    			(OS_TMR_CALLBACK_PTR) &MyCallback,
    			(void *) 0,
    			(RTOS_ERR *) &err);
        // Create mutexes
        OSMutexCreate ((OS_MUTEX *) &physics_mutex,
    			(CPU_CHAR *) "Physics Mutex",
    			(RTOS_ERR *) &err);
        OSMutexCreate ((OS_MUTEX *) &gain_mutex,
    			(CPU_CHAR *) "Gain Mutex",
    			(RTOS_ERR *) &err);
        // Create message queue
        OSQCreate((OS_Q	*)
        		&queue,
    			(CPU_CHAR *)"Message Queue",
    			(OS_MSG_QTY)4,
    			(RTOS_ERR *)&err);
        // FIFOs
        btn0_fifo_rd = 0;
        btn0_fifo_wr = 0;
        btn1_fifo_rd = 0;
        btn1_fifo_wr = 0;

        // Start timer
        OSTmrStart ((OS_TMR *) &tmr, (RTOS_ERR *) &err);

        /* 1 msec interrupts  */
        if (SysTick_Config(SystemCoreClockGet() / 1000)) {
          while (1) ;
        }

        // suspend start when init done
        OSTaskSuspend (&MainStartTaskTCB, &err);

        while (DEF_ON) {
            OSTimeDly( 1000, OS_OPT_TIME_DLY, &err);
            APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
        }
    }

static  void  MainIdleTask (void  *p_arg)
{
    RTOS_ERR  err;

    /* Prevent compiler warning.                            */
    PP_UNUSED_PARAM(p_arg);

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif


    while (DEF_ON) {
    	EMU_EnterEM1();
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}



/***************************************************************************//**
 * @brief
 *   Speed Setpoint
 *
 * @details
 * 	 determines button values
 *
 *
 ******************************************************************************/

static  void  MainPhysicsTask (void  *p_arg)
{
    RTOS_ERR  err;
    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */
#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */

    uint32_t start_time = 0;
    // set the initial conditions
	physics_data.gravity = 9.81;
	physics_data.bob = 20;
	physics_data.cart = 20;
	physics_data.len = 50;
	physics_data.graph_lim = 100;
	physics_data.xmin = 0;
	physics_data.xmax = 125;
	physics_data.time = 0;
	physics_data.theta = 0.01;
	physics_data.x_pos = 0;
	physics_data.w = 0;
	physics_data.v = 0;
    uint8_t game_reset = 0;
    double v_int;
    double w_int;
    double x_int;
    double theta_int;
    uint32_t x_min;
    uint32_t x_max;

    while (DEF_ON) {
    	if (!game_reset) {
    		uint32_t end_time = msTicks;
    		double dt = (double)(end_time-start_time)/1000;
        	double F = force_data*gain_data;

        	// get physics data
        	OSMutexPend (&physics_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        	double x = physics_data.x_pos;
        	double v = physics_data.v;
        	double theta = physics_data.theta;
        	double w = physics_data.w;
        	double L = physics_data.len;
        	double M = physics_data.cart;
        	double m = physics_data.bob;
        	double g = physics_data.gravity;
        	x_min = physics_data.xmin;
        	x_max = physics_data.xmax;
        	// release physics data
        	OSMutexPost (&physics_mutex, OS_OPT_POST_NONE, &err);

        	//  Equations
            v_int = v + dt*dvdt(F, theta, w, L, M, m, g);
            x_int = x + dt*dxdt(v_int);
            w_int = w + dt*dwdt(F, theta, w, L, M, m, g);
            theta_int = theta + dt*dthetadt(w_int);
            physics_data.time += dt;

        	// get physics data
        	OSMutexPend (&physics_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        	physics_data.time += dt;
        	physics_data.x_pos = x_int;
        	physics_data.v = v_int;
        	physics_data.w = w_int;
        	physics_data.theta = theta_int;
        	// release physics data
        	OSMutexPost (&physics_mutex, OS_OPT_POST_NONE, &err);


            start_time = msTicks;

            if (fabs(theta_int) > 1.57) {
            	game_reset = 1;
            }
            if (((int) (66 + x_int - sin(theta_int)*50)) < x_min||
            		((int) (66 + x_int - sin(theta_int)*50)) > x_max) {
            	game_reset = 1;
            }
    	}
    	else {
        	GPIO_PinOutSet(LED0_port, LED0_pin);
    	}

		OSTimeDly(8, OS_OPT_TIME_DLY, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Vehicle Direction
 *
 * @details
 * 	  vehicle direction using the slider
 *
 *
 ******************************************************************************/

static  void  MainForceTask (void  *p_arg)
{
    RTOS_ERR  err;
    uint8_t slider_pos;
    double nominal_force = 100.0;

    PP_UNUSED_PARAM(p_arg);

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    slider_setup();
    while (DEF_ON) {
    	OSSemPend (&timer_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        // slider status
        slider_position(&slider_pos);

        switch (slider_pos) {
        case NONE:
        	force_data = 0;
        	break;
        case FAR_LEFT:
        	force_data = -1.5*nominal_force;
        	break;
        case LEFT:
        	force_data = -1*nominal_force;
        	break;
        case RIGHT:
        	force_data = nominal_force;
        	break;
        case FAR_RIGHT:
        	force_data = 1.5*nominal_force;
        	break;
        default:
        	break;
        }

        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Vehicle Monitor
 *
 * @details
 * 	 check vehicle send to display and alert LED
 *
 *
 ******************************************************************************/

static  void  MainGainTask (void  *p_arg)
{
    RTOS_ERR  err;
    gain_data = 0;
    uint8_t btn0_status = 0;
    uint8_t btn1_status = 0;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
    buttons_setup();                                           /* ... the platform manager at this moment.             */
    gain_data = 6;
    while (DEF_ON) {
    	// pend on semaphore
    	OSSemPend (&fifo_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    	CORE_DECLARE_IRQ_STATE;           // Storage for saving IRQ state prior
    	CORE_ENTER_ATOMIC();
    	if (!fifo_isempty(btn0_fifo, &btn0_fifo_wr, &btn0_fifo_rd)) {
    		btn0_status = fifo_pop(btn0_fifo, &btn0_fifo_rd, FIFO_LEN);
    	}
    	if (!fifo_isempty(btn1_fifo, &btn1_fifo_wr, &btn1_fifo_rd)) {
    		btn1_status = fifo_pop(btn1_fifo, &btn1_fifo_rd, FIFO_LEN);
    	}
		CORE_EXIT_ATOMIC();

		if (btn0_status && (gain_data > 0)) {
			gain_data-=2;
		}
		else if (btn1_status && (gain_data < 20)) {
			gain_data+=2;
		}

		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   LCD Display
 *
 * @details
 * 	 display environment on LCD
 *
 *
 ******************************************************************************/

static  void  MainLcdDisplayTask (void  *p_arg)
{
    RTOS_ERR  err;
    EMSTATUS status;
    PP_UNUSED_PARAM(p_arg);

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    /* Initialize display module */
    status = DISPLAY_Init();
    if (DISPLAY_EMSTATUS_OK != status) {
      while (true)
        ;
    }

    status = DMD_init(0);
    if (DMD_OK != status) {
      while (true)
        ;
    }
    status = GLIB_contextInit(&gc);
    if (GLIB_OK != status) {
      while (true)
        ;
    }
    GLIB_Rectangle_t pRect;
    double len = 50;
    char time[4];
    double x_pos;
    double phy_time;
    double theta;
    while (DEF_ON) {
    	GLIB_clear(&gc);
    	GLIB_setFont(&gc, (GLIB_Font_t *)&GLIB_FontNarrow6x8);
        gc.backgroundColor = Black;
        gc.foregroundColor = White;

    	// get physics data
    	OSMutexPend (&physics_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    	x_pos = physics_data.x_pos;
    	theta = physics_data.theta;
    	phy_time = physics_data.time;
    	// release physics data
    	OSMutexPost (&physics_mutex, OS_OPT_POST_NONE, &err);

    	//  LCD elements
        int startx = 66 + x_pos;
        int starty = 66;
        int endx = (int) (startx - sin(theta)*len);
        int endy = (int) (starty - cos(theta)*len);
        pRect.xMax = startx+10;
        pRect.xMin = startx-10;
        pRect.yMax = starty+5;
        pRect.yMin = starty;
        // LCD shapes
	    GLIB_drawLine(&gc, startx, starty, endx, endy);
	    GLIB_drawCircleFilled(&gc, endx, endy, 4);
	    GLIB_drawRectFilled(&gc, &pRect);
	    GLIB_drawLine(&gc, 0, 72, 125, 72);
	    sprintf(time, "%d", (int) phy_time);
	    GLIB_setFont(&gc, (GLIB_Font_t *)&GLIB_FontNumber16x20);
	    GLIB_drawString(&gc, time, 4, 52, 90, 0);
	    /* Update */
	    DMD_updateDisplay();
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   kernel timer.
 *
 * @details
 * 	 semaphore shared with slider input not
 * 	 called until the OSTimer reaches 10 ms timeout then called
 * 	 every 10 ms signaling slider to execute
 *
 * @note
 *   called every time the OSTimer counts 10 ms.
 *
 ******************************************************************************/
static void MyCallback (OS_TMR p_tmr, void *p_arg) {
	RTOS_ERR  err;
	OSSemPost (&timer_sem, OS_OPT_POST_ALL, &err);
}



/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Even pins module.
 *
 *
 * @details
 * 	 PB0_status to polls value of pushbutton 0
 *
 * @note
 *   called every time PB0 is pressed.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	RTOS_ERR  err;
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	bool PB0_status;
	poll_PB0(&PB0_status);
	if (PB0_status) {
	    fifo_push(btn0_fifo, &btn0_fifo_wr, FIFO_LEN, 1);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	else {
	    fifo_push(btn0_fifo, &btn0_fifo_wr, FIFO_LEN, 0);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	__enable_irq();
}

/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Odd pins module.
 *
 *
 * @details
 * 	 PB1_status polls the value of pushbutton 1
 *
 * @note
 *   called every time PB1 is pressed.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	RTOS_ERR  err;
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	bool PB1_status;
	poll_PB1(&PB1_status);
	if (PB1_status) {
	    fifo_push(btn1_fifo, &btn1_fifo_wr, FIFO_LEN, 1);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	else {
	    fifo_push(btn1_fifo, &btn1_fifo_wr, FIFO_LEN, 0);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}

	__enable_irq();
}


/***************************************************************************//**
 * @brief
 *   Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;
  if (msTicks%20 == 0) {
	  start = msTicks;
  }
  if (msTicks < start + gain_data) {
	  GPIO_PinOutSet(LED1_port, LED1_pin);
  }
  else{
	  GPIO_PinOutClear(LED1_port, LED1_pin);
  }
}

