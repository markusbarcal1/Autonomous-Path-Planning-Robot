#include "MKL46Z4.h"
#include <stdint.h>
#include <stdbool.h>

#include "motor.h"

/* ================= CALIBRATION ================= */
#define SERVO_LEFT_US        2350U
#define SERVO_CENTER_US      1325U
#define SERVO_RIGHT_US        350U

#define FRONT_STOP_CM        18.0f
#define SIDE_CLEAR_CM        20.0f

#define START_DELAY_MS       2000U

/* Motor calibration */
#define BASE_LEFT_DUTY       85U
#define BASE_RIGHT_DUTY      90U

/* Timing */
#define FORWARD_BURST_MS     350U
#define POST_TURN_FORWARD_MS 250U
#define CORNER_ADVANCE_MS    500U
#define WRAP_FORWARD_MS     1000U
#define RIGHT_TURN_90_MS    1000U
#define LEFT_TURN_90_MS     1300U
#define UTURN_MS            1300U
#define REVERSE_MS           180U

#define SERVO_SETTLE_MS      200U

/* ================= PIN MAP ================= */
/* Servo + ultrasonic */
#define SERVO_PIN            12U    /* PTA12 */
#define TRIG_PIN              2U    /* PTD2 */
#define ECHO_PIN             13U    /* PTA13 */

/* Start button */
#define START_BTN_PIN         3U    /* PTC3 */

/* ================= STATE ================= */
typedef enum
{
    STATE_IDLE = 0,
    STATE_WAIT_2S,
    STATE_NAVIGATE
} robot_state_t;

typedef enum
{
    FOLLOW_LEFT = 0,
    FOLLOW_RIGHT
} wall_mode_t;

volatile robot_state_t g_state = STATE_IDLE;
volatile bool g_startRequest = false;

static wall_mode_t g_wallMode = FOLLOW_LEFT;
static bool g_wallModeLocked = false;

/* ================= FUNCTION PROTOTYPES ================= */
static void systick_free_run_init(void);
static void delay_us(uint32_t us);
static void delay_ms(uint32_t ms);

static void gpio_init_sensor_servo(void);
static void switch_init(void);

static int start_button_pressed(void);

static void servo_pulse_us(uint32_t pulse_us);
static void servo_hold_position_us(uint32_t pulse_us, uint32_t hold_ms);

static uint32_t measure_echo_high_us(void);
static float measure_distance_cm(void);
static float scan_at_us(uint32_t servo_us);
static float scan_left(void);
static float scan_center(void);
static float scan_right(void);

static void robot_stop(void);
static void robot_forward(void);
static void robot_reverse(void);
static void robot_turn_left(void);
static void robot_turn_right(void);

static void forward_for_ms(uint32_t ms);
static void reverse_for_ms(uint32_t ms);
static void turn_left_for_ms(uint32_t ms);
static void turn_right_for_ms(uint32_t ms);

static void navigate_step(void);

static void led_init(void);
static void led_green_on(void);
static void led_green_off(void);

/* ================= MAIN ================= */
int main(void)
{
    SystemCoreClockUpdate();

    led_init();
    led_green_on();

    systick_free_run_init();

    gpio_init_sensor_servo();
    switch_init();

    Motor_Init();
    Motor_StopAll();

    servo_hold_position_us(SERVO_CENTER_US, 500U);

    while (1)
    {
        switch (g_state)
        {
            case STATE_IDLE:
                Motor_StopAll();
                servo_hold_position_us(SERVO_CENTER_US, 200U);

                if (g_startRequest)
                {
                    g_startRequest = false;
                    g_state = STATE_WAIT_2S;
                }
                break;

            case STATE_WAIT_2S:
                delay_ms(START_DELAY_MS);
                g_wallModeLocked = false;
                g_state = STATE_NAVIGATE;
                break;

            case STATE_NAVIGATE:
                navigate_step();
                break;

            default:
                Motor_StopAll();
                g_state = STATE_IDLE;
                break;
        }
    }
}

/* ================= NAVIGATION ================= */
static void navigate_step(void)
{
    float dist_front;
    float dist_left;
    float dist_right;

    /* Choose which wall to follow once at startup */
    if (!g_wallModeLocked)
    {
        dist_left  = scan_left();
        dist_right = scan_right();

        if ((dist_left <= SIDE_CLEAR_CM) && (dist_right > SIDE_CLEAR_CM))
        {
            g_wallMode = FOLLOW_LEFT;
        }
        else if ((dist_right <= SIDE_CLEAR_CM) && (dist_left > SIDE_CLEAR_CM))
        {
            g_wallMode = FOLLOW_RIGHT;
        }
        else
        {
            g_wallMode = FOLLOW_LEFT;
        }

        g_wallModeLocked = true;
    }

    if (g_wallMode == FOLLOW_LEFT)
    {
        dist_left  = scan_left();
        dist_front = scan_center();

        /* Left wall disappeared -> advance past corner, then turn left */
        if ((dist_left > SIDE_CLEAR_CM) && (dist_front > FRONT_STOP_CM))
        {
            forward_for_ms(CORNER_ADVANCE_MS);
            turn_left_for_ms(LEFT_TURN_90_MS);
            forward_for_ms(WRAP_FORWARD_MS);
            return;
        }

        /* Wall ahead -> turn away from wall */
        if (dist_front <= FRONT_STOP_CM)
        {
            turn_right_for_ms(RIGHT_TURN_90_MS);
            forward_for_ms(POST_TURN_FORWARD_MS);
            return;
        }

        /* Normal wall-follow forward motion */
        forward_for_ms(FORWARD_BURST_MS);
        return;
    }
    else /* FOLLOW_RIGHT */
    {
        dist_right = scan_right();
        dist_front = scan_center();

        /* Right wall disappeared -> advance past corner, then turn right */
        if ((dist_right > SIDE_CLEAR_CM) && (dist_front > FRONT_STOP_CM))
        {
            forward_for_ms(CORNER_ADVANCE_MS);
            turn_right_for_ms(RIGHT_TURN_90_MS);
            forward_for_ms(WRAP_FORWARD_MS);
            return;
        }

        /* Wall ahead -> turn away from wall */
        if (dist_front <= FRONT_STOP_CM)
        {
            turn_left_for_ms(LEFT_TURN_90_MS);
            forward_for_ms(POST_TURN_FORWARD_MS);
            return;
        }

        /* Normal wall-follow forward motion */
        forward_for_ms(FORWARD_BURST_MS);
        return;
    }
}

/* ================= MOTOR WRAPPERS ================= */
static void robot_stop(void)
{
    Motor_StopAll();
}

static void robot_forward(void)
{
    Motor_LeftForward();
    Motor_RightForward();
    Motor_SetBothDutyPercent(BASE_LEFT_DUTY, BASE_RIGHT_DUTY);
}

static void robot_reverse(void)
{
    Motor_LeftReverse();
    Motor_RightForward();
    Motor_SetBothDutyPercent(BASE_LEFT_DUTY, BASE_RIGHT_DUTY);
}

static void robot_turn_left(void)
{
    /* swing left: right wheel drives, left wheel off */
    Motor_LeftForward();
    Motor_RightForward();
    Motor_SetLeftDutyPercent(0U);
    Motor_SetRightDutyPercent(BASE_RIGHT_DUTY);
}

static void robot_turn_right(void)
{
    /* swing right: left wheel drives, right wheel off */
    Motor_LeftForward();
    Motor_RightForward();
    Motor_SetLeftDutyPercent(BASE_LEFT_DUTY);
    Motor_SetRightDutyPercent(0U);
}

static void forward_for_ms(uint32_t ms)
{
    robot_forward();
    delay_ms(ms);
    robot_stop();
    delay_ms(40U);
}

static void reverse_for_ms(uint32_t ms)
{
    robot_reverse();
    delay_ms(ms);
    robot_stop();
    delay_ms(40U);
}

static void turn_left_for_ms(uint32_t ms)
{
    robot_turn_left();
    delay_ms(ms);
    robot_stop();
    delay_ms(80U);
}

static void turn_right_for_ms(uint32_t ms)
{
    robot_turn_right();
    delay_ms(ms);
    robot_stop();
    delay_ms(80U);
}

/* ================= SCANNING ================= */
static float scan_left(void)
{
    return scan_at_us(SERVO_LEFT_US);
}

static float scan_center(void)
{
    return scan_at_us(SERVO_CENTER_US);
}

static float scan_right(void)
{
    return scan_at_us(SERVO_RIGHT_US);
}

static float scan_at_us(uint32_t servo_us)
{
    float d1, d2, d3;

    servo_hold_position_us(servo_us, 400U);
    delay_ms(SERVO_SETTLE_MS);

    d1 = measure_distance_cm();
    delay_ms(25U);
    d2 = measure_distance_cm();
    delay_ms(25U);
    d3 = measure_distance_cm();

    return (d1 + d2 + d3) / 3.0f;
}

/* ================= ULTRASONIC ================= */
static uint32_t measure_echo_high_us(void)
{
    uint32_t timeout;
    uint32_t start;
    uint32_t end;
    uint32_t cycles;
    uint32_t ticks_per_us = SystemCoreClock / 1000000U;

    GPIOD->PCOR = (1U << TRIG_PIN);
    delay_us(2U);

    GPIOD->PSOR = (1U << TRIG_PIN);
    delay_us(10U);
    GPIOD->PCOR = (1U << TRIG_PIN);

    timeout = 30000U;
    while (((GPIOA->PDIR & (1U << ECHO_PIN)) == 0U) && timeout--)
    {
        delay_us(1U);
    }
    if (timeout == 0U) return 0U;

    start = SysTick->VAL;

    timeout = 30000U;
    while (((GPIOA->PDIR & (1U << ECHO_PIN)) != 0U) && timeout--)
    {
        delay_us(1U);
    }
    if (timeout == 0U) return 0U;

    end = SysTick->VAL;

    if (start >= end)
        cycles = start - end;
    else
        cycles = start + (SysTick->LOAD + 1U) - end;

    return cycles / ticks_per_us;
}

static float measure_distance_cm(void)
{
    uint32_t echo_us = measure_echo_high_us();

    if (echo_us == 0U)
        return 400.0f;   /* timeout as far away */

    return ((float)echo_us) / 58.0f;
}

/* ================= SERVO ================= */
static void servo_pulse_us(uint32_t pulse_us)
{
    GPIOA->PSOR = (1U << SERVO_PIN);
    delay_us(pulse_us);
    GPIOA->PCOR = (1U << SERVO_PIN);
    delay_us(20000U - pulse_us);
}

static void servo_hold_position_us(uint32_t pulse_us, uint32_t hold_ms)
{
    uint32_t pulses = hold_ms / 20U;
    uint32_t i;

    for (i = 0U; i < pulses; i++)
    {
        servo_pulse_us(pulse_us);
    }
}

/* ================= START SWITCH ================= */
static void switch_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[START_BTN_PIN] =
        PORT_PCR_MUX(1) |
        PORT_PCR_PE_MASK |
        PORT_PCR_PS_MASK |
        PORT_PCR_IRQC(0x0A);   /* falling edge */

    PTC->PDDR &= ~(1U << START_BTN_PIN);

    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

static int start_button_pressed(void)
{
    return ((GPIOC->PDIR & (1U << START_BTN_PIN)) == 0U);
}

/* ================= LED ================= */
static void led_init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    PORTD->PCR[5] = PORT_PCR_MUX(1);
    PTD->PDDR |= (1U << 5);

    PTD->PSOR = (1U << 5);   /* off */
}

static void led_green_on(void)
{
    PTD->PCOR = (1U << 5);   /* active-low */
}

static void led_green_off(void)
{
    PTD->PSOR = (1U << 5);
}

void PORTC_PORTD_IRQHandler(void)
{
    if (PORTC->ISFR & (1U << START_BTN_PIN))
    {
        if (g_state == STATE_IDLE)
        {
            g_startRequest = true;
        }
        else
        {
            /* kill switch back to idle */
            g_state = STATE_IDLE;
        }

        PORTC->ISFR = (1U << START_BTN_PIN);
    }
}

/* ================= GPIO INIT ================= */
static void gpio_init_sensor_servo(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;

    /* Servo PTA12 */
    PORTA->PCR[SERVO_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[SERVO_PIN] |= PORT_PCR_MUX(1);
    GPIOA->PDDR |= (1U << SERVO_PIN);
    GPIOA->PCOR = (1U << SERVO_PIN);

    /* TRIG PTD2 */
    PORTD->PCR[TRIG_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[TRIG_PIN] |= PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1U << TRIG_PIN);
    GPIOD->PCOR = (1U << TRIG_PIN);

    /* ECHO PTA13 */
    PORTA->PCR[ECHO_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[ECHO_PIN] |= PORT_PCR_MUX(1);
    GPIOA->PDDR &= ~(1U << ECHO_PIN);
}

/* ================= TIMING ================= */
static void systick_free_run_init(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t ticks_per_us = SystemCoreClock / 1000000U;
    uint32_t needed = us * ticks_per_us;
    uint32_t elapsed = 0;
    uint32_t prev = SysTick->VAL;

    while (elapsed < needed)
    {
        uint32_t now = SysTick->VAL;

        if (prev >= now)
            elapsed += (prev - now);
        else
            elapsed += (prev + (SysTick->LOAD + 1U) - now);

        prev = now;
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000U);
    }
}
