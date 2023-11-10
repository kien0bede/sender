#include "controller_board.h"
#include "adapter_board.h"
#include "ringbuffer.h"
#include "commgr.h"
#include "buzzer_engine.h"
#include "power_mgr.h"
#include "config.h"
#include "command.h"

PinInfo USER_BUTTON_Pin = {
	.pin = GPIO_PIN_0,
	.port = GPIOA
};

#define GND_DETECT_ASSERTED			(0x1U)
#define PRESS_INTERVAL 2000

static struct ring_buffer_t g_rbuf;

static OperationMode g_opMode = OPMODE_INVALID;
static volatile OperationMode g_stagingMode = OPMODE_INVALID;

/* Global variables using in auto mode */
static uint8_t g_gndDetectFlags;
static uint8_t g_lastGndDetectFlags;
static uint8_t btn_pressed = 0;
static uint32_t press_time = 0;
static uint8_t short_press_triggered = 0;
static uint32_t g_deadlineToPowerDevice;
static uint32_t g_deadlineToRunQaCheck;

static BuzzerScript g_qaFailedBuzzerScript = {
	.interval = 100,
	.breakTime = 50,
	.count = 2
};

static BuzzerScript g_qaPassedBuzzerScript = {
	.interval = 1000,
	.breakTime = 0,
	.count = 1
};

static BuzzerScript g_shortedBuzzerScript = {
	.interval = 100,
	.breakTime = 50,
	.count = 2
};

static void HandleOutReport(HID_OutReport *outRp);
static void EnterNewMode(OperationMode mode);
static void AdapterBoardDelegateInit();
static bool GndDetect_Asserted();
static void AutoModeProcess(bool gndDetectChanged);
static bool GndDetectPoll();
static QaStatus RunQaCheck();

/***** Public APIs *****/
/***********************/

void CB_Init()
{
	/* USER_BUTTON initialization */
	GPIO_InitTypeDef userBtn = {0};

	userBtn.Pin = USER_BUTTON_Pin.pin;
	userBtn.Mode = GPIO_MODE_INPUT;
	userBtn.Pull = GPIO_PULLUP;

	HAL_GPIO_Init(USER_BUTTON_Pin.port, &userBtn);

	ring_buffer_init(&g_rbuf);
	AdapterBoardDelegateInit();
}

void CB_Yeild()
{
	OperationMode stgMode = g_stagingMode;
	char buf[sizeof(HID_OutReport)];
	ring_buffer_size_t n;
	bool changed;

	if (stgMode != g_opMode)
	{
		EnterNewMode(stgMode);
		g_opMode = stgMode;
	}

	changed = GndDetectPoll();

	__NVIC_DisableIRQ(USB_IRQn);
	n = ring_buffer_dequeue_arr(&g_rbuf, buf, sizeof(buf));
	__NVIC_EnableIRQ(USB_IRQn);

	if (n == sizeof(buf))
	{
		HID_OutReport *outRp = (HID_OutReport *)buf;

		if (outRp->id == HID_OUT_REPORT_ID)
		{
			if (g_opMode == OPMODE_AUTO)
			{
				g_opMode = OPMODE_CONTROL;
				g_stagingMode = OPMODE_CONTROL; // Bear in mind to change this staging state as well
				EnterNewMode(OPMODE_CONTROL);
			}

			HandleOutReport(outRp);
		}
	}

	if (g_opMode == OPMODE_AUTO)
	{
		AutoModeProcess(changed);
	}
}

void CB_BUTTON() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
		if (!btn_pressed) {
			press_time = HAL_GetTick();
			btn_pressed = 1;
			short_press_triggered = 0;
		}
		if ((HAL_GetTick() - press_time) >= PRESS_INTERVAL) {
			if (g_opMode < OPMODE_NUM - 1)
			{
				CB_SwitchOperationMode(g_opMode + 1);
			}
			else
			{
				CB_SwitchOperationMode(OPMODE_INVALID + 1);
			}
			LED_STT_G_SET;
			HAL_Delay(1000);
			LED_STT_G_RESET;
			HAL_Delay(10);
			btn_pressed = 0;
		}
	} else {
		if (btn_pressed) {
			if (!short_press_triggered) {
				CB_QaCheck();
				LED_STT_B_SET;
				HAL_Delay(1000);
				LED_STT_B_RESET;
				HAL_Delay(10);
				short_press_triggered = 1;
			}
		}
		btn_pressed = 0;
	  }
}

void CB_IRQ_HID_DataOut(uint8_t *buf)
{
	ring_buffer_queue_arr(&g_rbuf, (char*)buf, sizeof(HID_OutReport));
}

void CB_IRQ_OnTargetReset(bool asserted)
{
	// It's safe to call this function in IRQ mode
	PM_SetChipPOR(!asserted);
}

void CB_OnBootModeChanged(BootMode mode)
{
	switch (mode)
	{
	case BOOTMODE_SERIAL:
		BOOT_SERIAL_SET;
		BOOT_DEV_RESET;
		BOOT_FUSE_RESET;
		break;
	case BOOTMODE_DEVELOPMENT:
		BOOT_SERIAL_RESET;
		BOOT_DEV_SET;
		BOOT_FUSE_RESET;
		break;
	case BOOTMODE_FUSE:
		BOOT_SERIAL_RESET;
		BOOT_DEV_RESET;
		BOOT_FUSE_SET;
		break;
	default:
		break;
	}
}

OperationMode CB_GetOperationMode()
{
	return g_opMode;
}

bool CB_IsDeviceConnected()
{
	return GndDetect_Asserted();
}

void CB_SwitchOperationMode(OperationMode mode)
{
	g_stagingMode = mode;
}

QaStatus CB_QaCheck()
{
	QaStatus stat = RunQaCheck();

	if (stat == QA_PASSED)
	{
		COM_Trace("QA Check PASSED!");
		MBZ_STT_R_RESET;
		MBZ_STT_G_SET;
		BZ_Play(g_qaPassedBuzzerScript);
	}
	else if (stat == QA_VOLTAGES_FAILED_BUT_ACCEPTED)
	{
		COM_Trace("QA Check ACCEPTED!");
		MBZ_STT_R_SET;
		MBZ_STT_G_SET;
		BZ_Play(g_qaPassedBuzzerScript);
	}
	else
	{
		switch (stat)
		{
		case QA_DEVICE_NOT_FOUND:
			COM_Trace("Device not attached");
			break;
		case QA_DEVICE_NOT_POWERED:
			COM_Trace("Device not powered");
			break;
		case QA_VOLTAGES_FAILED:
			COM_Trace("Voltages checking failed");
			break;
		case QA_POWER_FAILED:
			COM_Trace("Power checking failed");
			break;
		default:
			COM_Trace("Unknown failure reason");
			break;
		}

		COM_Trace("QA Check FAILED!");
		MBZ_STT_R_SET;
		MBZ_STT_G_RESET;
		BZ_Play(g_qaFailedBuzzerScript);
	}

	return stat;
}

/***** Private APIs, utilities *****/
/***********************************/

static bool GndDetect_Asserted()
{
	AdapterBoard *adapter = GetAdapterBoard();
	return adapter->gndDetect == NULL
				|| (HAL_GPIO_ReadPin(adapter->gndDetect->port, adapter->gndDetect->pin) == GPIO_PIN_RESET);
}

static void AdapterBoardDelegateInit()
{
	AdapterBoard *adapter = GetAdapterBoard();

	/* GND Detect initialization */
	GPIO_InitTypeDef gndDetect = {0};

	gndDetect.Mode = GPIO_MODE_INPUT;
	gndDetect.Pull = GPIO_NOPULL;

	if (adapter->gndDetect != NULL) {
		gndDetect.Pin = adapter->gndDetect->pin;
		HAL_GPIO_Init(adapter->gndDetect->port, &gndDetect);
	}
}

static void HandleOutReport(HID_OutReport *outRp)
{
	switch (outRp->type)
	{
	case CMD_PING:
		CMD_Ping(outRp);
		break;
	case CMD_INFO:
		CMD_Info(outRp);
		break;
	case CMD_PLAY_BUZZER:
		CMD_PlayBuzzer(outRp);
		break;
	case CMD_PERI_RESET:
		CMD_PeriReset(outRp);
		break;
	case CMD_CHIP_RESET:
		CMD_ChipReset(outRp);
		break;
	case CMD_DEVICE_POWER_CTRL:
		CMD_DevicePowerControl(outRp);
		break;
	case CMD_SWITCH_BOOT_MODE:
		CMD_SwitchBootMode(outRp);
		break;
	case CMD_RUN_QA:
		CMD_RunQa(outRp);
		break;
	case CMD_REPORT_POWER:
		CMD_ReportPower(outRp);
		break;
	case CMD_WAIT_FOR_TARGET:
		CMD_WaitForTarget(outRp);
		break;
	case CMD_LIST_TP:
		CMD_ListTP(outRp);
		break;
	case CMD_READ_TP:
		CMD_ReadTP(outRp);
		break;
	case CMD_SWITCH_AUTO:
		CMD_SwitchToAuto(outRp);
		break;
	default:
		if (!GetAdapterBoard()->Handle_HID_OutReport(outRp))
		{
			HID_InReport inRp = {0};
			inRp.id = HID_IN_REPORT_ID;
			inRp.token = outRp->token;
			inRp.flags = HID_IN_FLAG_DONE | HID_IN_FLAG_ERROR;
			inRp.inum = ERROR_CODE_UNKNOWN_CMD;
			COM_Report(&inRp);
		}
		break;
	}
}

static void EnterNewMode(OperationMode mode)
{
	switch (mode)
	{
	case OPMODE_AUTO:
		break;
	case OPMODE_CONTROL:
		ring_buffer_init(&g_rbuf);
		break;
	default:
		break;
	}

	AdapterBoard *adapter = GetAdapterBoard();
	adapter->EnterNewMode(mode);
}

static void AutoModeProcess(bool gndDetectChanged)
{
	if (g_deadlineToRunQaCheck > 0 && HAL_GetTick() >= g_deadlineToRunQaCheck)
	{
		CB_QaCheck();
		g_deadlineToRunQaCheck = 0;
	}

	if (g_deadlineToPowerDevice > 0 && HAL_GetTick() >= g_deadlineToPowerDevice)
	{
		PowerStatus st = PM_SetDevicePower(true);
		if (st == POWER_OK)
		{
			g_deadlineToRunQaCheck = HAL_GetTick() + MS_TO_TICK(CONFIG_DELAY_POWER_ON_MS);
		}
		else if (st == POWER_SHORT_DETECTED)
		{
			LED_STT_R_SET;
			BZ_Play(g_shortedBuzzerScript);
			g_deadlineToRunQaCheck = 0;
		}
		g_deadlineToPowerDevice = 0;
	}

	if (gndDetectChanged)
	{
		if ((g_gndDetectFlags & GND_DETECT_ASSERTED))
		{
			// Schedule a deadline to turn device on
			LED_STT_R_RESET;
			LED_STT_G_RESET;
			g_deadlineToPowerDevice = HAL_GetTick() + MS_TO_TICK(CONFIG_DELAY_POWER_ON_MS);
		}
		else
		{
			g_deadlineToPowerDevice = 0;
			g_deadlineToRunQaCheck = 0;
		}
	}
}

static bool GndDetectPoll()
{
	if (GndDetect_Asserted())
	{
		g_gndDetectFlags |= GND_DETECT_ASSERTED;
	}
	else
	{
		g_gndDetectFlags &= ~GND_DETECT_ASSERTED;
	}

	if (g_lastGndDetectFlags != g_gndDetectFlags)
	{
		bool pwrOff = false;
		if (g_gndDetectFlags & GND_DETECT_ASSERTED)
		{
			BOARD_G_SET;
			BOARD_R_RESET;
		}
		else
		{
			pwrOff = true;
			BOARD_R_SET;
			BOARD_G_RESET;
		}

		if (g_gndDetectFlags & GND_DETECT_ASSERTED)
		{
			CMD_WaitForTarget_Cb();
		}
		else if (pwrOff)
		{
			PM_SetDevicePower(false);
		}
		g_lastGndDetectFlags = g_gndDetectFlags;
		return true;
	}
	return false;
}

static QaStatus RunQaCheck()
{
	bool passed, accepted;

	if (!CB_IsDeviceConnected())
	{
		return QA_DEVICE_NOT_FOUND;
	}

	if (!PM_IsDevicePowered())
	{
		return QA_DEVICE_NOT_POWERED;
	}

	PM_VerifyVoltages(&passed, &accepted);

	if (!passed)
	{
		if (accepted)
		{
			return QA_VOLTAGES_FAILED_BUT_ACCEPTED;
		}
		else
		{
			return QA_VOLTAGES_FAILED;
		}
	}

	return QA_PASSED;
}
