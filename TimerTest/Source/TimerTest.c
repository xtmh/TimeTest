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
static tsTimerContext timer1;


// デバッグメッセージ出力用
#define DBG
#ifdef DBG
#define dbg(...) vfPrintf(&sSerStream, LB __VA_ARGS__)
#else
#define dbg(...)
#endif

uint16	u16adc;
static int sum = 0;
static int t1=0;

uint16	adcBuffer;


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
	/*
	//	Timer用
	memset(&timer0, 0, sizeof(tsTimerContext));
    timer0.u8Device = E_AHI_DEVICE_TIMER0; // timer0使用
    timer0.u16Hz = 1000;        // 1000Hz
    timer0.u8PreScale = 1;      // プリスケーラ1/2
    timer0.bDisableInt = FALSE; // 割り込み禁止
    vTimerConfig(&timer0); // タイマ設定書き込み
    vTimerStart(&timer0);  // タイマスタート
    */

    // PWM の初期化
	memset(&timer1, 0, sizeof(tsTimerContext));
	vAHI_TimerFineGrainDIOControl(0x7);	// bit 0,1,2 をセット (TIMER0 の各ピンを解放する, PWM1..4 は使用する)
	//vAHI_TimerFineGrainDIOControl(0x0); 	// bit 0,1 をセット (TIMER0 の各ピンを解放する, PWM1..4 は使用する)
	vAHI_TimerSetLocation(E_AHI_TIMER_1, TRUE, TRUE); // IOの割り当てを設定
	// PWM用
	timer1.u8Device = E_AHI_DEVICE_TIMER1;	//	timer1使用
	timer1.u16Hz = 20000;		//	1000Hz
	timer1.u8PreScale = 1;		//	プリスケーラー
	//timer1.u16duty = 1024; 	//	1024=Hi, 0:Lo
	timer1.u16duty = 512; 		//	512=Hi, 512:Lo
	timer1.bPWMout = TRUE;		//	PWM出力
	//timer1.bDisableInt = TRUE;//	割り込みを許可する指定
	timer1.bDisableInt = FALSE;	//	割り込みを禁止する指定
	vTimerConfig(&timer1);
	vTimerStart(&timer1);
	t1 = 0;
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
		vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,	//	ｱﾅﾛｸﾞ部通電
						E_AHI_AP_INT_ENABLE,		//	ADC完了割り込み許可
						E_AHI_AP_SAMPLE_2,			//	ｻﾝﾌﾟﾙ周期：2clock periods
						E_AHI_AP_CLOCKDIV_500KHZ,	//	500kHz(recommended)
						E_AHI_AP_INTREF);			//	ﾘﾌｧﾚﾝｽ電圧(internal)
		while(!bAHI_APRegulatorEnabled());			//	電圧安定するまで待つ
	}

	//	②ADC設定
	vAHI_AdcEnable(	E_AHI_ADC_CONTINUOUS,
            		//	E_AHI_ADC_SINGLE_SHOT //１回のみ
					//	E_AHI_ADC_CONTINUOUS,// 連続実行
					E_AHI_AP_INPUT_RANGE_2,
					// E_AHI_AP_INPUT_RANGE_1 		(0-1.2V(Vref))
					// または E_AHI_AP_INPUT_RANGE_2 (0-2.4V(2Vref))
					E_AHI_ADC_SRC_ADC_2);
					// E_AHI_ADC_SRC_ADC_1 (ADC1)
					// E_AHI_ADC_SRC_ADC_2 (ADC2)
					// E_AHI_ADC_SRC_ADC_3 (ADC3)
					// E_AHI_ADC_SRC_ADC_4 (ADC4)
					// E_AHI_ADC_SRC_TEMP (温度 on-chip)
					// E_AHI_ADC_SRC_VOLT (電圧 internal voltage monitor)
	//	③ADC開始
	//vAHI_AdcStartSample(); // ADC開始
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

// ユーザ定義のイベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
    // 1 秒周期のシステムタイマ通知
   if(eEvent == E_EVENT_TICK_SECOND)
   {
       //dbg("%d\t%d",u32TickCount_ms,sum);      //  Tickタイマが数えてる（らしい）msと，TIMER0によるmsを出力
	   /*
	   uint16 u16AdcValue = u16AHI_AdcRead();
	   dbg("T1=%d\tT2=%d\tT3=%d\tADC=%d",
			   u32TickCount_ms,
			   sum,
			   t1,
			   u16AdcValue);
		*/
	   bPortRead(DO3) ? vPortSetHi(DO3) : vPortSetLo(DO3);
   }
   if(eEvent == E_EVENT_TICK_TIMER){
	   //	125Hz
	}

    return;
}

//	ADC 完了待ちおよび読み出し
//   (ポーリングのAPIに不具合が有るので割り込みを待ちます)
//   以下は AppQueueAPI を使用したコードの一部。
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind)
{
	/*
	uint32 u32DeviceId = psAHI_Ind->u32DeviceId;
	//uint32 u32ItemBitmap = psAHI_Ind->u32ItemBitmap;

	switch (u32DeviceId) {
	case E_AHI_DEVICE_ANALOGUE:
        // ADCが終了
        u16adc = u16AHI_AdcRead(); // 値の読み出し
 	    dbg("ad=%d",u16adc);

 	   vAHI_AdcStartSample();
        break;
    }
    */
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
    switch (u32DeviceId) {
    //	TickTimerによるｲﾍﾞﾝﾄ割り込み
    case E_AHI_DEVICE_TICK_TIMER:
        // LED BLINK
        //1msごとにLオンオフ
        //vPortSet_TrueAsLo(PORT_KIT_LED2, u32TickCount_ms & 0x001);
         break;
    case E_AHI_DEVICE_TIMER0:  //  TIMER0で割り込まれたら
    	//dbg("timer0");
        sum++;
        break;
    case E_AHI_DEVICE_TIMER1:  //  TIMER1で割り込まれたら
    	timer1.u16duty = t1; 		// 512=Hi, 512:Lo
    	vTimerStart(&timer1);
    	t1++;
    	if(t1>1023)	t1 = 0;
    	//dbg("timer1:%04d", t1);
        break;
    default:
        break;
    }
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{

    return FALSE;
}

//	timer0によるｺｰﾙﾊﾞｯｸ・・・ADで割り込み不要としており呼ばれない
PRIVATE void timer0Callback(uint32 device, uint32 bitmap)
{
	dbg("timer0Callback");
}

//	AD変換ｺｰﾙﾊﾞｯｸ
PRIVATE void adCallback(uint32 device, uint32 bitmap)
{
	static int n=0;
	dbg("%06d:%04d", n++, adcBuffer);
}

void adDMAInit()
{
	//	ｱﾅﾛｸﾞ部の電源投入
	if(!bAHI_APRegulatorEnabled()){
		vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,	//	ｱﾅﾛｸﾞ部通電
						E_AHI_AP_INT_ENABLE,		//	ADC完了割り込み許可
						E_AHI_AP_SAMPLE_2,			//	ｻﾝﾌﾟﾙ周期：2clock periods
						E_AHI_AP_CLOCKDIV_500KHZ,	//	500kHz(recommended)
						E_AHI_AP_INTREF);			//	ﾘﾌｧﾚﾝｽ電圧(internal)
		while(!bAHI_APRegulatorEnabled());			//	電圧安定するまで待つ
	}

	//	timer0
	vAHI_TimerFineGrainDIOControl(0xff);				//	timer0でIOﾎﾟｰﾄ使用しない
	vAHI_TimerConfigureOutputs(E_AHI_TIMER_0, FALSE, FALSE);
	vAHI_TimerEnable(E_AHI_TIMER_0, 14, FALSE, FALSE, FALSE);	//	AD変換でﾀｲﾏｰ割り込みは不要
	vAHI_TimerClockSelect(E_AHI_TIMER_0, FALSE, TRUE);	//
	vAHI_Timer0RegisterCallback(timer0Callback);
	vAHI_TimerDIOControl(E_AHI_TIMER_0, FALSE);
	vAHI_TimerStartRepeat(E_AHI_TIMER_0, 0, 500);		//	500msで繰り返し


	//	callback登録
	vAHI_APRegisterCallback(adCallback);

	//	DMAを使ったADC
	if(!bAHI_AdcEnableSampleBuffer(E_AHI_AP_INPUT_RANGE_2,
									E_AHI_TIMER_0,
									0x04,	//	AD3
									adcBuffer,
									1,
									TRUE,
									E_AHI_AP_INT_DMA_OVER_MASK|E_AHI_AP_INT_DMA_END_MASK)){
		//dbg("adc start");
	}


}


// コールドスタート時
void cbAppColdStart(bool_t bAfterAhiInit)
{

    sToCoNet_AppContext.u8CPUClk = 3;       //  CPUクロックは最高の32MHz(0:4MHz,1:8MHz,2:16MHz,3:32MHz)
    sToCoNet_AppContext.u16TickHz = 250;    //  TickTimerはデフォルトの250Hz
    //sToCoNet_AppContext.u16TickHz = 1000; //	システムTICK割り込みを1kHz
    ToCoNet_vRfConfig();                    //  設定適用．無くてもいい気がする（未検証）

    if (!bAfterAhiInit) {
    } else {
        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);
        ToCoNet_Event_Register_State_Machine(vProcessIncomingHwEvent);
        vInitHardware();
        //vInitADC();
        //vInitTimer();
        adDMAInit();
        vAHI_AdcStartSample();	//	ADC開始
    }
}

// ウォームスタート時
void cbAppWarmStart(bool_t bAfterAhiInit)
{
    return;
}
