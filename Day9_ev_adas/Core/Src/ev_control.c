/**
 * @file    ev_control.c
 * @brief   Electric Vehicle dynamics model
 *
 * Computes:
 *   â€¢ Motor torque from accelerator pedal + drive-mode scale
 *   â€¢ Regenerative braking torque from brake pedal
 *   â€¢ Vehicle speed via inertia model (simple Euler integration)
 *   â€¢ Instantaneous power (kW)
 *   â€¢ State-of-Charge via energy integration
 *   â€¢ Estimated range from SOC and drive-mode efficiency
 *
 * ADC channels (12-bit, 0â€“4095 â†’ 0â€“100 %):
 *   PA0 = accel pedal, PA1 = brake pedal,
 *   PA2 = SOC (initial), PA3 = motor temperature
 */

#include "ev_control.h"

/* â”€â”€â”€ Drive-mode torque scaling table â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static const float TORQUE_MAP[3] = {
    0.6f,   /* ECO    â€” 60 % max torque */
    1.0f,   /* NORMAL */
    1.3f,   /* SPORT  â€” 130 % boost     */
};

/* â”€â”€â”€ ADC rank â†’ channel mapping (matches CubeMX Injected/Regular config) â”€â”€ */
#define ADC_RANK_ACCEL  0
#define ADC_RANK_BRAKE  1
#define ADC_RANK_SOC    2
#define ADC_RANK_TEMP   3
#define ADC_CHANNELS    4

static uint32_t adc_buf[ADC_CHANNELS];

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

/**
 * @brief  Initialise EV handle to safe defaults (PARKED cold state).
 */
void EV_Init(EV_HandleTypeDef *ev)
{
    memset(ev, 0, sizeof(*ev));
    ev->soc        = 100.0f;
    ev->drive_mode = DRIVE_MODE_NORMAL;
    ev->motor_temp = 25.0f;    /* ambient */
    ev->speed_kmh = 10;
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
uint16_t Read_ADC_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

  return HAL_ADC_GetValue(&hadc1);
}

/**
 * @brief  Read four ADC channels (polling DMA-filled buffer) and map to
 *         pedal/temp values.  Call once per EV_Update cycle.
 *
 *  Conversion:
 *    pedal_pct  = (adc / 4095.0) Ã— 100.0
 *    temp_c     = (adc / 4095.0) Ã— 120.0   (NTC linearised, 0â€“120 Â°C)
 */
void EV_ReadADC(EV_HandleTypeDef *ev)
{
    /* Start ADC conversion (all 4 channels, scan mode) */

	adc_buf[ADC_RANK_ACCEL] = Read_ADC_Channel(ADC_CHANNEL_0);
	adc_buf[ADC_RANK_BRAKE] = Read_ADC_Channel(ADC_CHANNEL_1);
	adc_buf[ADC_RANK_TEMP]  = Read_ADC_Channel(ADC_CHANNEL_3);

    ev->accel_pedal = CLAMP((adc_buf[ADC_RANK_ACCEL] / 4095.0f) * 100.0f,
                             0.0f, 100.0f);
    ev->brake_pedal = CLAMP((adc_buf[ADC_RANK_BRAKE] / 4095.0f) * 100.0f,
                             0.0f, 100.0f);
    ev->motor_temp  = CLAMP((adc_buf[ADC_RANK_TEMP]  / 4095.0f) * 120.0f,
                             0.0f, 120.0f);

    /* PA2 (SOC ADC) is only used on first boot to seed the SOC value.
       After that, SOC is tracked by energy integration â€” do NOT overwrite
       ev->soc from ADC every cycle. */
}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

/**
 * @brief  Core EV model update â€” call at 100 Hz (dt = 0.01 s).
 *
 * @param  ev   Pointer to EV handle (already populated by EV_ReadADC).
 * @param  dt   Time step in seconds (nominally 0.01).
 */
void EV_Update(EV_HandleTypeDef *ev, float dt)
{
    float mode_scale = TORQUE_MAP[ev->drive_mode];

    /* â”€â”€ 1. Motor torque from accelerator â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
        //ev->motor_torque = ev->accel_pedal * EV_MAX_TORQUE_NM * mode_scale;
        ev->motor_torque = (ev->accel_pedal / 100.0f) * EV_MAX_TORQUE_NM * mode_scale;

        /* â”€â”€ 2. Regenerative braking â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
        if (ev->brake_pedal > EV_REGEN_THRESHOLD_PCT) {
              /* 0â€“70 % scale */
            ev->regen_level  = ev->brake_pedal * 0.7f;
            ev->motor_torque = -(ev->regen_level / 100.0f) * EV_REGEN_TORQUE_MAX_NM;
            /* Simplified: torque = -regen_pct * max_regen_Nm */

        }

        /* â”€â”€ 3. Speed â€” simple inertia model (Euler) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
        /*    accel (m/sÂ²) = (net_torque - drag) / mass_factor                    */
        /* Lumped drag in Nm (proportional to speed proxy) */
        float speed_ms = ev->speed_kmh / 3.6f;
        float drag_Nm  = speed_ms * EV_DRAG_COEFF;          /* Nm, e.g. coeff = 2.0 */
        float accel    = (ev->motor_torque - drag_Nm) / EV_MASS_FACTOR;  /* m/sÂ² proxy */
        ev->speed_kmh  = CLAMP(ev->speed_kmh + accel * dt * 3.6f, 0.0f, EV_MAX_SPEED_KMH);

        /* â”€â”€ 4. Instantaneous power â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
        /* Mechanical power: P_mech (kW) = T (Nm) Ã— v (m/s) / 1000              */
        float v_ms = ev->speed_kmh / 3.6f;
        float p_mech_kw = ev->motor_torque * v_ms / 1000.0f;

        /* Motor copper losses (IÂ²R): proportional to torqueÂ², non-zero at       */
        /* standstill. This is the current draw even when the vehicle is stopped. */
        /* Without this term, power_kw = 0 at v=0 â†’ SOC never changes at rest.  */
        float torque_ratio = ev->motor_torque / EV_MAX_TORQUE_NM;
        float p_loss_kw    = torque_ratio * torque_ratio * 5.0f;  /* up to 5 kW  */

        ev->power_kw = p_mech_kw + p_loss_kw;

        /* â”€â”€ 5. SOC integration â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

        float delta_soc = (ev->power_kw * dt)
                          / (EV_BATTERY_CAPACITY_KWH * 3600.0f)
                          * 100.0f
                          * EV_SIM_SCALE;
        ev->soc = CLAMP(ev->soc - delta_soc, 0.0f, 100.0f);

        /* â”€â”€ 6. Estimated range â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
        float eff_whpkm = (ev->drive_mode == DRIVE_MODE_ECO)
                           ? EV_EFFICIENCY_ECO
                           : EV_EFFICIENCY_OTHER;
        /* remaining Wh = soc% Ã— capacity_kWh Ã— 1000 */
        float remaining_wh = (ev->soc / 100.0f) * EV_BATTERY_CAPACITY_KWH * 1000.0f;
        ev->range_km = remaining_wh / eff_whpkm;

        /* â”€â”€ 7. Motor thermal model (simple warm-up/cool-down) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
        float thermal_power = fabsf(ev->power_kw) * 0.05f;  /* 5 % loss = heat */
        float cooling       = (ev->motor_temp - 25.0f) * 0.01f;
        ev->motor_temp += (thermal_power - cooling) * dt;
        ev->motor_temp  = CLAMP(ev->motor_temp, 25.0f, 130.0f);


}

/* â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

void EV_SetDriveMode(EV_HandleTypeDef *ev, uint8_t mode)
{
    if (mode <= DRIVE_MODE_SPORT)
        ev->drive_mode = mode;
}

void EV_InjectSpeed(EV_HandleTypeDef *ev, float speed_kmh)
{
    ev->speed_kmh = CLAMP(speed_kmh, 0.0f, EV_MAX_SPEED_KMH);
}

void EV_InjectSOC(EV_HandleTypeDef *ev, float soc_pct)
{
    ev->soc = CLAMP(soc_pct, 0.0f, 100.0f);
}

void EV_InjectMotorTemp(EV_HandleTypeDef *ev, float temp_c)
{
    ev->motor_temp = CLAMP(temp_c, 0.0f, 130.0f);
}
