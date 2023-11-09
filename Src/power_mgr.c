#include "power_mgr.h"
#include "controller_board.h"
#include "adapter_board.h"
#include "config.h"

#define INA230_ADDR 	0x80U

// Refers to INA230 schematic to know how to calculate this value
#define INA230_CALIB	(uint16_t)0x0200U

PinInfo PWR_EN_Pin = {
	.pin = GPIO_PIN_15,
	.port = GPIOA
};

PinInfo BOARD_PERI_RST_Pin = {
	.pin = GPIO_PIN_11,
	.port = GPIOB
};

PinInfo IMX6_POR_CTRL_Pin = {
	.pin = GPIO_PIN_1,
	.port = GPIOA
};

#define DEFAULT_VDDA 3.3f

static float gVDDA = DEFAULT_VDDA;
static float gAdcResolution = DEFAULT_VDDA / 4095.0f;

bool PM_ReadVoltage(const TestPointSpec *spec, float *val, bool raw)
{
	bool success = false;
	ADC_ChannelConfTypeDef channelConf;

	channelConf.Rank = 1;
	channelConf.SamplingTime = CONFIG_ADC_SAMPLE_TIME;
	channelConf.Channel = spec->adcChannel;

	if (HAL_ADC_ConfigChannel(&hadc, &channelConf) != HAL_OK)
	{
		return false;
	}

	HAL_ADC_Start(&hadc);
	HAL_StatusTypeDef rc = HAL_ADC_PollForConversion(&hadc, 500);

	if (rc == HAL_OK)
	{
		uint32_t res = HAL_ADC_GetValue(&hadc);

		#if CONFIG_CALIB_MODE == 1
				COM_Trace("Raw %s=%d", spec->name, res);
		#endif

		if (!raw && spec->isHalf)
		{
			*val = (float)res * gAdcResolution * 2;
		}
		else
		{
			*val = (float)res * gAdcResolution;
		}
	}
	else
	{
		goto out;
	}

	*val += *val * spec->testPointVolLinearCalib;
	success = true;
out:
	channelConf.Rank = ADC_RANK_NONE;
	HAL_ADC_ConfigChannel(&hadc, &channelConf);
	HAL_ADC_Stop(&hadc);
	HAL_Delay(1000);

	return success;
}

static bool CheckShort(TestPointSpec *specs, uint8_t sz)
{
	if (!CONFIG_CHECK_SHORT_ENABLE)
	{
		return true;
	}

	bool result = true;

	for (uint8_t i = 0; i < sz; ++i)
	{
		if (!specs[i].activated || !specs[i].chkShortActivated)
		{
			continue;
		}

		float voltage;

		if (!PM_ReadVoltage(&specs[i], &voltage, true) ||
				voltage < specs[i].chkShortVolTolPercent * specs[i].chkShortVol)
		{
			COM_Trace("Shorted point: %s", specs[i].name);
			COM_TraceFloat(voltage, "With %s = %sV", specs[i].name, "%.3f");
			result = false;
		} else
		{
			COM_TraceFloat(voltage, "CK %s = %sV", specs[i].name, "%.3f");
		}
	}

	return result;
}

static void InitTargetAdcPins()
{
	AdapterBoard *adapter = GetAdapterBoard();

	for (uint8_t i = 0; i < adapter->testPointSpecsSz; ++i)
	{
		TestPointSpec *spec = &adapter->testPointSpecs[i];
		if (!spec->activated)
		{
			continue;
		}

		GPIO_InitTypeDef adcPin = {0};

		adcPin.Pin = (uint32_t)spec->adcPin.pin;
		adcPin.Mode = GPIO_MODE_ANALOG;
		adcPin.Pull = GPIO_NOPULL;

		HAL_GPIO_Init(spec->adcPin.port, &adcPin);
	}
}

void PM_Init()
{
	/* CHECK_SHORT_EN initialization */
	GPIO_InitTypeDef pwrEn = {0};

	pwrEn.Pin = PWR_EN_Pin.pin;
	pwrEn.Mode = GPIO_MODE_OUTPUT_PP;
	pwrEn.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_WritePin(PWR_EN_Pin.port, PWR_EN_Pin.pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(PWR_EN_Pin.port, &pwrEn);

	/* BOARD_PERI_RST initialization */
	GPIO_InitTypeDef boardPeriRst = {0};

	boardPeriRst.Pin = BOARD_PERI_RST_Pin.pin;
	boardPeriRst.Mode = GPIO_MODE_OUTPUT_PP;
	boardPeriRst.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_WritePin(BOARD_PERI_RST_Pin.port, BOARD_PERI_RST_Pin.pin, GPIO_PIN_SET);
	HAL_GPIO_Init(BOARD_PERI_RST_Pin.port, &boardPeriRst);

	/* IMX6_POR_CTRL initialization */
	GPIO_InitTypeDef imx6PorCtrl = {0};

	imx6PorCtrl.Pin = IMX6_POR_CTRL_Pin.pin;
	imx6PorCtrl.Mode = GPIO_MODE_OUTPUT_PP;
	imx6PorCtrl.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_WritePin(IMX6_POR_CTRL_Pin.port, IMX6_POR_CTRL_Pin.pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(IMX6_POR_CTRL_Pin.port, &imx6PorCtrl);

	InitTargetAdcPins();
}

void PM_ResetPeri()
{
	HAL_GPIO_WritePin(BOARD_PERI_RST_Pin.port, BOARD_PERI_RST_Pin.pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BOARD_PERI_RST_Pin.port, BOARD_PERI_RST_Pin.pin, GPIO_PIN_SET);
}

void PM_SetChipPOR(bool on)
{
	if (on)
	{
		HAL_GPIO_WritePin(IMX6_POR_CTRL_Pin.port, IMX6_POR_CTRL_Pin.pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(IMX6_POR_CTRL_Pin.port, IMX6_POR_CTRL_Pin.pin, GPIO_PIN_RESET);
	}
}

PowerStatus PM_SetDevicePower(bool on)
{
	AdapterBoard *adapter = GetAdapterBoard();

	if (on)
	{
		if (!CB_IsDeviceConnected() && !CONFIG_ALLOW_POWER_WITHOUT_TARGET)
		{
			COM_Trace("Device not attached");
			return POWER_DEVICE_NOT_ATTACHED;
		}

		if (CheckShort(adapter->testPointSpecs, adapter->testPointSpecsSz))
		{
			COM_Trace("Powered up");
			HAL_GPIO_WritePin(PWR_EN_Pin.port, PWR_EN_Pin.pin, GPIO_PIN_SET);
		}
		else
		{
			COM_Trace("Short circuit detected");
			return POWER_SHORT_DETECTED;
		}

		MBZ_PWR_OK_SET;
		return POWER_OK;
	}
	else
	{
		COM_Trace("Powered down");
		HAL_GPIO_WritePin(PWR_EN_Pin.port, PWR_EN_Pin.pin, GPIO_PIN_RESET);

		MBZ_PWR_OK_RESET;
		return POWER_OK;
	}
}

void PM_VerifyVoltages(bool *passed, bool *accepted)
{
	AdapterBoard *adapter = GetAdapterBoard();
	int nPassed = 0;
	int nAccepted = 0;
	int nActivated = 0;

	for (uint8_t i = 0; i < adapter->testPointSpecsSz; ++i)
	{
		float voltage;
		const TestPointSpec *spec = &adapter->testPointSpecs[i];

		if (!spec->activated)
		{
			continue;
		}

		nActivated++;

		if (!PM_ReadVoltage(spec, &voltage, false))
		{
			COM_Trace("Can't read %s", spec->name);
			continue;
		}

		if ((voltage > spec->testPointVol - spec->testPointVolTol)
				&& (voltage < spec->testPointVol + spec->testPointVolTol))
		{
			nPassed++;
			nAccepted++;
			COM_TraceFloat(voltage, "%s = %sV [P]", spec->name, "%.3f");
		}
		else // failed
		{
			// Workaround for VSYS 3V8 MobiZ LTE
			// Failed but accepted due to hardware limitations
			if (spec->failureAccepted)
			{
				COM_TraceFloat(voltage, "%s = %sV [A]", spec->name, "%.3f");
				nAccepted++;
			}
			else
			{
				COM_TraceFloat(voltage, "%s = %sV [F]", spec->name, "%.3f");
			}
		}
	}

	*passed = (nPassed == nActivated);
	*accepted = (nAccepted == nActivated);
}

bool PM_IsDevicePowered()
{
	return HAL_GPIO_ReadPin(PWR_EN_Pin.port, PWR_EN_Pin.pin) == GPIO_PIN_SET;
}

void PM_SetVDDA(float vdda)
{
	gVDDA = vdda;
	gAdcResolution = vdda / 4095.0f;
}

float PM_GetVDDA()
{
	return gVDDA;
}

bool PM_ReportPower(uint16_t *vbus, uint16_t *vshut, uint16_t *current, uint16_t *power)
{
	if (!CB_IsDeviceConnected())
	{
		COM_Trace("Device not attached!");
		return false;
	}

	if (!PM_IsDevicePowered())
	{
		COM_Trace("Device not powered!");
		return false;
	}

	return true;
}

//void PM_Test()
//{
//	float val;
//	TestPointSpec *spec = &(GetAdapterBoard()->testPointSpecs[9]);
//	PM_ReadVoltage(spec, &val);
//	COM_TraceFloat(val, "TEST %s = %sV", spec->name, "%.3f");
//	HAL_Delay(1500);
//}
