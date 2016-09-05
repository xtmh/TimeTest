#include <AppHardwareApi.h>
#include "utils.h"
#include "serial.h"
#include "sprintf.h"
#include "ToCoNet.h"
#include <PeripheralRegs.h>
#include <string.h>


#define UART_BAUD 115200 // シリアルのボーレート

#define DO3   4  // デジタル出力 3
#define DO4   9  // デジタル出力 4


static tsFILE sSerStream;          // シリアル用ストリーム
static tsSerialPortSetup sSerPort; // シリアルポートデスクリプタ

static tsTimerContext timer0;


// デバッグメッセージ出力用
#define DBG
#ifdef DBG
#define dbg(...) vfPrintf(&sSerStream, LB __VA_ARGS__)
#else
#define dbg(...)
#endif

uint16	u16adc;

// デバッグ出力用に UART を初期化
static void vSerialInit() {
    static uint8 au8SerialTxBuffer[96];
    static uint8 au8SerialRxBuffer[32];

    sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
    sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
    sSerPort.u32BaudRate = UART_BAUD;
    sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
    sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
    sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
    sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
    sSerPort.u8SerialPort = E_AHI_UART_0;
    sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
    SERIAL_vInit(&sSerPort);

    sSerStream.bPutChar = SERIAL_bTxChar;
    sSerStream.u8Device = E_AHI_UART_0;
}


static void vInitTimer()
{
    memset(&timer0, 0, sizeof(tsTimerContext));
    timer0.u8Device = E_AHI_DEVICE_TIMER0; // timer0使用

    timer0.u16Hz = 1000;        // 1000Hz
    timer0.u8PreScale = 1;      // プリスケーラ1/2
    timer0.bDisableInt = FALSE; // 割り込み禁止


    vTimerConfig(&timer0); // タイマ設定書き込み
    vTimerStart(&timer0);  // タイマスタート

}

/**
 * ADCの初期化
 */
static void vInitADC() {
	/*
	// ADC
	vADC_Init(&sAppData.sObjADC, &sAppData.sADC, TRUE);
	sAppData.u8AdcState = 0xFF; // 初期化中

#ifdef USE_TEMP_INSTDOF_ADC2
	sAppData.sObjADC.u8SourceMask =
			TEH_ADC_SRC_VOLT | TEH_ADC_SRC_ADC_1 | TEH_ADC_SRC_TEMP;
#else
	sAppData.sObjADC.u8SourceMask =
			TEH_ADC_SRC_VOLT | TEH_ADC_SRC_ADC_1 | TEH_ADC_SRC_ADC_2;
#endif
	*/

	//	①ｱﾅﾛｸﾞ部の電源投入
	if(!bAHI_APRegulatorEnabled()){
		vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,
						E_AHI_AP_INT_ENABLE,
						E_AHI_AP_SAMPLE_2,
						E_AHI_AP_CLOCKDIV_500KHZ,
						E_AHI_AP_INTREF);
		while(!bAHI_APRegulatorEnabled());
	}

	//	②ADC開始
	vAHI_AdcEnable(
			E_AHI_ADC_CONTINUOUS,
            //	E_AHI_ADC_SINGLE_SHOT //１回のみ
			//	E_AHI_ADC_CONTINUOUS,// 連続実行
		E_AHI_AP_INPUT_RANGE_2,
           // E_AHI_AP_INPUT_RANGE_1 (0-1.2V)
           // または E_AHI_AP_INPUT_RANGE_2 (0-2.4V)
		E_AHI_ADC_SRC_ADC_2
           // E_AHI_ADC_SRC_ADC_1 (ADC1)
           // E_AHI_ADC_SRC_ADC_2 (ADC2)
           // E_AHI_ADC_SRC_ADC_3 (ADC3)
           // E_AHI_ADC_SRC_ADC_4 (ADC4)
           // E_AHI_ADC_SRC_TEMP (温度)
           // E_AHI_ADC_SRC_VOLT (電圧)
    );

	vAHI_AdcStartSample(); // ADC開始
}

// ハードウェア初期化
static void vInitHardware()
{
    vSerialInit();                      //  シリアル通信の設定
    ToCoNet_vDebugInit(&sSerStream);
    ToCoNet_vDebugLevel(0);

	/////////////////////////////////
    // 使用ポートの設定
    vPortAsOutput(DO3);		//	DO3:4
    vPortSetHi(DO3);
    /////////////////////////////////

}

static int sum = 0;

// ユーザ定義のイベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
	static bool_t b= TRUE;


    // 1 秒周期のシステムタイマ通知
   if(eEvent == E_EVENT_TICK_SECOND)
   {
        //dbg("%d\t%d",u32TickCount_ms,sum);      //  Tickタイマが数えてる（らしい）msと，TIMER0によるmsを出力

	   uint16 u16AdcValue = u16AHI_AdcRead();
	   dbg("%d\t%d\t%d",u32TickCount_ms,sum,u16AdcValue);

    	bPortRead(DO3) ? vPortSetHi(DO3) : vPortSetLo(DO3);
   }

    return;
}

//	ADC 完了待ちおよび読み出し
//   (ポーリングのAPIに不具合が有るので割り込みを待ちます)
//   以下は AppQueueAPI を使用したコードの一部。
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind)
{
	uint32 u32DeviceId = psAHI_Ind->u32DeviceId;
	//uint32 u32ItemBitmap = psAHI_Ind->u32ItemBitmap;

	switch (u32DeviceId) {
	case E_AHI_DEVICE_ANALOGUE:
        // ADCが終了
        u16adc = u16AHI_AdcRead(); // 値の読み出し
 	    dbg("ad=%d",u16adc);
        break;
    }
}

void cbToCoNet_vMain(void)
{
    return;
}


void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
    return;
}


void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus)
{
    return;
}


void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg)
{
    return;
}

// ハードウェア割り込みの遅延実行部
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    if(u32DeviceId == E_AHI_DEVICE_TIMER0)  //  TIMER0で割り込まれたら
    {
        sum++;
    }
    return;
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{

    return FALSE;
}

// コールドスタート時
void cbAppColdStart(bool_t bAfterAhiInit)
{

    sToCoNet_AppContext.u8CPUClk = 3;       //  CPUクロックは最高の32MHz
    sToCoNet_AppContext.u16TickHz = 250;    //  TickTimerはデフォルトの250Hz
    ToCoNet_vRfConfig();                    //  設定適用．無くてもいい気がする（未検証）

    if (!bAfterAhiInit) {
    } else {
        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);
        ToCoNet_Event_Register_State_Machine(vProcessIncomingHwEvent);
        vInitHardware();
        vInitADC();
        vAHI_AdcStartSample();	//	ADC開始
        vInitTimer();
    }
}

// ウォームスタート時
void cbAppWarmStart(bool_t bAfterAhiInit)
{
    return;
}



