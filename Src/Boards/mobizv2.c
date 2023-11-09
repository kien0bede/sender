#include "adapter_board.h"
#include "controller_board.h"

static PinInfo GND_DETECT_Pin = {
	.pin = GPIO_PIN_1,
	.port = GPIOB
};

static PinInfo BOOT0_SEL_Pin = {
	.pin = GPIO_PIN_2,
	.port = GPIOB,
};

static PinInfo BOOT1_SEL_Pin = {
	.pin = GPIO_PIN_10,
	.port = GPIOB,
};

static IRQn_Type g_MobizV2_IrqList[1] = { EXTI4_15_IRQn };

/* Refer to power_mgr.h to know how to provide test point specs */
static TestPointSpec g_testPointSpecs[] = {
	TEST_POINT_SPEC(3, "VSYS_1V8", ADC_INPUT_3, ADC_CHANNEL_3, false, false, 1.2f, 0.6f, 1.8f, 0.1f, 0.15),
	TEST_POINT_SPEC(4, "VDD_ARM", ADC_INPUT_4, ADC_CHANNEL_4, false, false, 1.2f, 0.6f, 1.4f, 0.05f, -0.1),
	TEST_POINT_SPEC(5, "DCDC_3V3", ADC_INPUT_5, ADC_CHANNEL_5, true, false, 3.3f, 0.6f, 3.3f, 0.15f, 0.08),
	TEST_POINT_SPEC(6, "PMIC_ON_REQ", ADC_INPUT_6, ADC_CHANNEL_6, true, false, 3.3f, 0.3f, 3.3f, 0.15f, 0.08),
	TEST_POINT_SPEC(7, "VDD_DRAM", ADC_INPUT_7, ADC_CHANNEL_7, false, false, 1.2f, 0.3f, 1.35f, 0.05f, -0.15),
	TEST_POINT_SPEC(8, "LS_3V3", ADC_INPUT_8, ADC_CHANNEL_8, true, false, 3.3f, 0.6f, 3.3f, 0.15f, 0.08),
};

static void MobizV2_Init()
{
	/* Boot select initialization */
	GPIO_InitTypeDef bootSel = {0};

	bootSel.Pin = BOOT0_SEL_Pin.pin | BOOT1_SEL_Pin.pin;
	bootSel.Mode = GPIO_MODE_OUTPUT_PP;
	bootSel.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(BOOT0_SEL_Pin.port, &bootSel);
}

static void MobizV2_EnterNewMode(OperationMode mode)
{
	switch (mode)
	{
	case OPMODE_AUTO:
		break;
	case OPMODE_CONTROL:
		break;
	default:
		break;
	}
}

static bool MobizV2_Handle_HID_OutReport(HID_OutReport *outRp)
{
	switch (outRp->type)
	{
	default:
		return false;
	}

	return true; // command handled
}

static bool MobizV2_SwitchBootMode(BootMode mode)
{
	switch (mode)
	{
	case BOOTMODE_SERIAL:
		HAL_GPIO_WritePin(BOOT0_SEL_Pin.port, BOOT0_SEL_Pin.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BOOT1_SEL_Pin.port, BOOT1_SEL_Pin.pin, GPIO_PIN_RESET);
		break;
	case BOOTMODE_DEVELOPMENT:
		HAL_GPIO_WritePin(BOOT0_SEL_Pin.port, BOOT0_SEL_Pin.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BOOT1_SEL_Pin.port, BOOT1_SEL_Pin.pin, GPIO_PIN_SET);
		break;
	case BOOTMODE_FUSE:
		HAL_GPIO_WritePin(BOOT0_SEL_Pin.port, BOOT0_SEL_Pin.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BOOT1_SEL_Pin.port, BOOT1_SEL_Pin.pin, GPIO_PIN_RESET);
		break;
	default:
		return false;
	}

	CB_OnBootModeChanged(mode);
	return true;
}

static AdapterBoard __board = {
	.boardName = "MobiZ V2",
	.irqList = g_MobizV2_IrqList,
	.irqListSz = sizeof(g_MobizV2_IrqList) / sizeof(g_MobizV2_IrqList[0]),
	.testPointSpecs = g_testPointSpecs,
	.testPointSpecsSz = sizeof(g_testPointSpecs) / sizeof(g_testPointSpecs[0]),
	.gndDetect = &GND_DETECT_Pin,
	.Init = MobizV2_Init,
	.EnterNewMode = MobizV2_EnterNewMode,
	.Handle_HID_OutReport = MobizV2_Handle_HID_OutReport,
	.SwitchBootMode = MobizV2_SwitchBootMode,
};

AdapterBoard *GetAdapterBoard(void)
{
	return &__board;
}
