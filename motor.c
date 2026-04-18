#include "motor.h"
#include "MKL46Z4.h"

#define PWM_MOD_VALUE 999U

void Motor_Init(void)
{
    /* enable clocks */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK;

    /* select TPM clock source */
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    /* direction pins as GPIO */
    PORTB->PCR[0] = PORT_PCR_MUX(1);   /* AI1 */
    PORTB->PCR[1] = PORT_PCR_MUX(1);   /* AI2 */
    PORTC->PCR[2] = PORT_PCR_MUX(1);   /* BI1 */
    PORTB->PCR[3] = PORT_PCR_MUX(1);   /* BI2 */

    PTB->PDDR |= (1U << 0) | (1U << 1) | (1U << 3);
    PTC->PDDR |= (1U << 2);

    /* PWM pins to TPM */
    PORTB->PCR[2] = PORT_PCR_MUX(3);   /* PTB2 -> TPM2_CH0 */
    PORTC->PCR[1] = PORT_PCR_MUX(4);   /* PTC1 -> TPM0_CH0 */

    /* TPM2 CH0 on PTB2 */
    TPM2->SC = 0;
    TPM2->MOD = PWM_MOD_VALUE;
    TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->CONTROLS[0].CnV = 0;
    TPM2->SC = TPM_SC_PS(4) | TPM_SC_CMOD(1);

    /* TPM0 CH0 on PTC1 */
    TPM0->SC = 0;
    TPM0->MOD = PWM_MOD_VALUE;
    TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM0->CONTROLS[0].CnV = 0;
    TPM0->SC = TPM_SC_PS(4) | TPM_SC_CMOD(1);

    /* safe default: motors off */
    PTB->PCOR = (1U << 0) | (1U << 1) | (1U << 3);
    PTC->PCOR = (1U << 2);
}

void Motor_LeftForward(void)
{
    PTB->PSOR = (1U << 0);
    PTB->PCOR = (1U << 1);
}

void Motor_LeftReverse(void)
{
    PTB->PCOR = (1U << 0);
    PTB->PSOR = (1U << 1);
}

void Motor_RightForward(void)
{
    PTC->PCOR = (1U << 2);
    PTB->PSOR = (1U << 3);
}

void Motor_RightReverse(void)
{
    PTC->PSOR = (1U << 2);
    PTB->PCOR = (1U << 3);
}

void Motor_StopAll(void)
{
    TPM2->CONTROLS[0].CnV = 0;
    TPM0->CONTROLS[0].CnV = 0;

    PTB->PCOR = (1U << 0) | (1U << 1) | (1U << 3);
    PTC->PCOR = (1U << 2);
}

void Motor_SetLeftDutyPercent(uint8_t percent)
{
    if (percent >= 100U)
        TPM2->CONTROLS[0].CnV = PWM_MOD_VALUE;
    else
        TPM2->CONTROLS[0].CnV = (PWM_MOD_VALUE * percent) / 100U;
}

void Motor_SetRightDutyPercent(uint8_t percent)
{
    if (percent >= 100U)
        TPM0->CONTROLS[0].CnV = PWM_MOD_VALUE;
    else
        TPM0->CONTROLS[0].CnV = (PWM_MOD_VALUE * percent) / 100U;
}

void Motor_SetBothDutyPercent(uint8_t leftPercent, uint8_t rightPercent)
{
    Motor_SetLeftDutyPercent(leftPercent);
    Motor_SetRightDutyPercent(rightPercent);
}
