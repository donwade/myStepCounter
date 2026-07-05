// Include this to enable the M5 global instance.
//#include <M5Unified.h>

#include <_m5Core2-only.h>

//#include <MahonyAHRS.h>
#include <_RTC.h>
#include <_OTAUpload.h>
#include <_onPowerDn.h>

#include "LowPassFilterIf.h"
#include "stepConfig.h"
#include "sd-logger.h"
#include "pretty.h"


#define LOG_FILENAME "/FLIGHT.LOG"
#define BACKUP1 	 "/DATA1.BKU"
#define BACKUP2 	 "/DATA2.BKU"
#define BACKUP3 	 "/DATA3.BKU"
#define BACKUP4 	 "/DATA4.BKU"
#define BACKUP5 	 "/DATA5.BKU"

#define FLIGHT_LEN 4000

typedef struct {
	bool bArmed;
	float deltaACC;
	float velocity;
} oneEntry;


static oneEntry flightRecorder[FLIGHT_LEN];
static int flightIndex = 0;
uint32_t bytesNotSavedYet = 0;
uint32_t bytesInFlightRecorder = 0;



// Strength of the calibration operation;
// 0: disables calibration.
// 1 is weakest and 255 is strongest.

static constexpr const uint8_t calDepth = 64;

// This sample code performs calibration by clicking on a button or screen.
// After 10 seconds of calibration, the results are stored in NVS.
// The saved calibration values are loaded at the next startup.
//
// === How to calibration ===
// ※ Calibration method for Accelerometer
//    Change the direction of the main unit by 90 degrees
//     and hold it still for 2 seconds. Repeat multiple times.
//     It is recommended that as many surfaces as possible be on the bottom.
//
// ※ Calibration method for Gyro
//    Simply place the unit on a quiet desk and hold it still.
//    It is recommended that this be done after the accelerometer calibration.
//
// ※ Calibration method for geomagnetic sensors
//    Rotate the main unit slowly in multiple directions.
//    It is recommended that as many surfaces as possible be oriented to the north.
//
// Values for extremely large attitude changes are ignored.
// During calibration, it is desirable to move the device as gently as possible.


typedef struct text_t  // size of printed text
{
	uint32_t cHeight1;
    int32_t topLeftX;
    int32_t topLeftY;
    int32_t width;
    int32_t heigth;
    uint32_t textForegndC;
    uint32_t textBackgndC;
    uint32_t handle;
    uint8_t tsize;
	char msg[50];
	uint32_t fgColor;
	uint32_t bgColor;
	const lgfx::v1::GFXfont *font;
 };

struct window_t
{
    int32_t win_topLeftX;
    int32_t win_topLeftY;
    int32_t win_width;
    int32_t win_heigth;
    uint32_t win_textForegndC;
    uint32_t win_textBackgndC;
    uint32_t win_boarderC;
    text_t tinfo[8];
};

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define 3D Coordinate structure
typedef struct {
    double x; // Easting
    double y; // Northing
    double z; // Altitude (unused for azimuth)
} Point3D;


uint8_t numSensorsInIMU = 2;  // we only have gyro and accel, no compass
uint8_t numItemsPerSensor = 3;   // x, y, z

#define BAR_THICK 18

static constexpr const uint32_t color_tbl[18] =
{
    0xFF0000u, 0xCCCC00u, 0xCC00FFu,
    0xFFCC00u, 0x00FF00u, 0x0088FFu,
    0xFF00CCu, 0x00FFCCu, 0x0000FFu,
    0xFF0000u, 0xCCCC00u, 0xCC00FFu,
    0xFFCC00u, 0x00FF00u, 0x0088FFu,
    0xFF00CCu, 0x00FFCCu, 0x0000FFu,
};

#define BMI_270  // m5 core2

#ifdef BMI_270
static constexpr const float coefficient_tbl[3] = {  
													(1.0f / 256.0f),	//scale ACCEL
													 0.5f,				//scale GYRO 
													(1.0f / 1024.0f)	//scale MAGNET
													};
#else
static constexpr const float coefficient_tbl[3] = {  0.5f,				//scale GYRO 
													(1.0f / 256.0f), 	//scale ACCEL
													(1.0f / 1024.0f)	//scale MAGNET
													};

#endif
//--------------------------------------------------------------------------------

static auto &display = (M5.Display);
static window_t graphicWindow;
static window_t textWindow;
static window_t topWindow;

static int prev_xpos[18];

void drawBar(int32_t midLeftX, int32_t midLeftY, int32_t newAcross, int32_t oldAcross, int32_t heigth, uint32_t color)
{
    uint32_t bgcolor = (color >> 3) & 0x1F1F1Fu;
    
	//Serial.printf("oldAcross = %d newAcross = %d\n", oldAcross, newAcross);

	// oldAcross is non-zero and while new and old are on opposite sides	
    if (oldAcross && ((newAcross < 0) != (oldAcross < 0)))
    {
    	// zap the old bar as the new bar isn't on this side of the middle line
        display.fillRect(midLeftX, midLeftY, oldAcross, heigth, bgcolor);

        // pretend there was no old-across ever used.
        oldAcross = 0;
    }

	// is there a difference in widths?
    if (oldAcross != newAcross)
    {
    	// if new across is to the right of the old across..... 
    	// draw darkness.
        if ((newAcross > oldAcross) != (newAcross < 0)) bgcolor = color;

        display.setColor(bgcolor);
        //               |<-right edge X-->|         |<- draw leftwards->|
        display.fillRect(newAcross + midLeftX, midLeftY, oldAcross - newAcross  , heigth);
    }
}

//--------------------------------------------------------------

void drawImuStats(const window_t& r, const m5::imu_data_t& imuDirect)
{
    int midPointX = r.win_topLeftX + r.win_width/2;  // move to horizontal center point.
    int topLeftY = r.win_topLeftY;
        
    int bar_count = numSensorsInIMU * numItemsPerSensor;  //0,1,2 accel xyz  4,5,6 gyro xyz
    int barThick = r.win_heigth/ bar_count; 

    display.startWrite();

	//Serial.printf("bar_count = %d\n", bar_count);
	// imuDirect.accel[3] + imuDirect.gyro[3] = 6 items

    int barNum;
    for (barNum = 0; barNum < bar_count; ++barNum)
    {
        float xval;

		auto coe = coefficient_tbl[barNum / 3] * r.win_width;
		xval = imuDirect.value[barNum] * coe;
 

        int newWidth = xval;
        int oldWidth = prev_xpos[barNum];

        int maxw = r.win_width/2 -1;
        
        if (newWidth < -maxw )
        	newWidth = -maxw;
        else if (newWidth > maxw)
        	newWidth = maxw;
        

        if (newWidth != oldWidth)
            prev_xpos[barNum] = newWidth;

        drawBar(midPointX, 
        		topLeftY + barThick * barNum, 
        		newWidth, 
        		oldWidth, 
        		barThick - 1, 
        		color_tbl[barNum]);
    }

    display.endWrite();
    //Serial.printf("x aph display ends at %d\n", topLeftY + heightY * barNum);
    //Serial.printf("ssss = %d\n", r.rectH);
}

//---------------------------------------------------------------------

void startCalibration(uint32_t waitS)
{
 	Serial.printf("start calibration ... depth = %d of 255\n", calDepth);
 	
    M5.Imu.setCalibration(calDepth, 
    					  calDepth, 
    					  calDepth);

	M5.Speaker.setVolume(30);

	for (int i= 0; i < waitS; i++)
	{
		
		M5.Speaker.tone(800, 100);
		Serial.printf("cal %d of %d\n", i, waitS);
		delay(900);
	}

    M5.Imu.setCalibration(0, //accel
    					  0, //gyro
    					  0  //calDepth
    					  );

	M5_LOGW("saving to NVS");

	M5.Imu.saveOffsetToNVS();

	M5.Speaker.setVolume(80);
	M5.Speaker.tone(1000, 100);
 	delay(500);
	M5.Speaker.tone(700, 100);
	M5.Speaker.setVolume(30);
 	
 	Serial.println("cal ended");
 	delay(1000);
 	
}

//-------------------------------------------------------------
void showRect(char *msg, window_t &reader)
{
	M5_LOGW("%s x=%d y=%d w=%d h=%d", msg, reader.win_topLeftX, reader.win_topLeftY, reader.win_width, reader.win_heigth);
}

//-------------------------------------------------------------
void makeWindow(window_t &win, int8_t boarder)
{
	display.fillRect(win.win_topLeftX, win.win_topLeftY, 
					 win.win_width, win.win_heigth, 
					 win.win_boarderC);

	showRect("in",  win);

	// boarder < 0. shrink ... > 0 grow
	
	win.win_heigth 		+= boarder * 2;
	win.win_width 		+= boarder * 2;
	win.win_topLeftX 	-= boarder;   // <0 = shrinking,  move in + dir
	win.win_topLeftY 	-= boarder;
	
	showRect("out",  win);
#if 1
	display.fillRect(win.win_topLeftX, win.win_topLeftY, 
					 win.win_width, win.win_heigth, 
					 win.win_textBackgndC);

#endif
	display.display();
}

//https://github.com/m5stack/M5Stack/blob/master/examples/Advanced/Display/Free_Font_Demo/Free_Font_Demo.ino

#include <TFT_eSPI.h>

#define BIG_FONT   &fonts::FreeSansBold24pt7b
#define TOPIC_FONT &fonts::FreeSansBold9pt7b
#define STATS_FONT &fonts::FreeMono12pt7b

const uint8_t VSPACE = 2;
uint32_t cHeight1;

uint32_t  myDrawString(window_t &window, 
						char *msg, 
						uint32_t topX, uint32_t topY, 
						const lgfx::v1::GFXfont *font,
						uint8_t size,
						uint32_t fgColour, uint32_t bgColour
						)
{
	static uint8_t nextIndex;
	assert ( nextIndex < 8);
	
	if (bgColour != TFT_BLACK )
	{
		M5_LOGW("only TFT_BLACK supported for background");
		bgColour = TFT_BLACK;
	}
	
	M5.Lcd.setTextColor(fgColour, bgColour);

	M5.Lcd.setTextSize(size);
	M5.Lcd.drawString(msg, topX, topY, font); 
	window.tinfo[nextIndex].heigth = M5.Lcd.fontHeight(font) + VSPACE;
	window.tinfo[nextIndex].width = M5.Lcd.textWidth(msg);
	window.tinfo[nextIndex].topLeftX = topX;
	window.tinfo[nextIndex].topLeftY = topY;
	window.tinfo[nextIndex].font = font;
	window.tinfo[nextIndex].tsize = size;
	window.tinfo[nextIndex].bgColor = bgColour;
	window.tinfo[nextIndex].fgColor = fgColour;
	
	strncpy(window.tinfo[nextIndex].msg, msg, sizeof(window.tinfo[nextIndex].msg));
	window.tinfo[nextIndex].handle = micros();
	display.display();

	nextIndex++;
	
	return window.tinfo[nextIndex-1].handle;
};

uint32_t  myRefreshString(window_t &window, uint32_t handle, char *msg )
{

	int i; 
	int cnt = sizeof(window.tinfo)/sizeof(window.tinfo[0]);
	for (i = 0; i < cnt ; i++)
	{
		if (handle == window.tinfo[i].handle) break;
	}
	
	if ( i == cnt)
	{
		M5_LOGE("bad handle passed in ... ignoring");
		return 0;
	}

	M5_LOGD("recovered index %d from handle %d", i, handle);
	// if the old and new string are different, erase old.

	M5.Lcd.setTextSize( window.tinfo[i].tsize);

	if (strcmp(msg, window.tinfo[i].msg))
	{
		// erase previous
		M5.Lcd.setTextColor(window.tinfo[i].bgColor, window.tinfo[i].bgColor);
	
		M5.Lcd.drawString(window.tinfo[i].msg, 
						  window.tinfo[i].topLeftX, window.tinfo[i].topLeftY,
						  window.tinfo[i].font); 
		M5_LOGD("erase old text \"%s\"", window.tinfo[i].msg);
	}
	
	//display.display();
	//delay(4000);
	//M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
	
	M5.Lcd.setTextColor(window.tinfo[i].fgColor, window.tinfo[i].bgColor);
	
	M5.Lcd.drawString(msg, window.tinfo[i].topLeftX, window.tinfo[i].topLeftY,
						 window.tinfo[i].font); 
						 
	strncpy(window.tinfo[i].msg, msg, sizeof(window.tinfo[i].msg));

	M5_LOGD("paint new \"%s\"", window.tinfo[i].msg);
	
	display.display();
	return window.tinfo[i].handle;
};
    

uint32_t hLargeTextArea; 
uint32_t hSmallTextArea;

#define RECORDING 1
#define PLAYBACK  0
#define LIVE      0

File hFile;

//-----------------------------------------------------
// do not close. this is not your job.
// return number of bytes in flight recorder .

uint32_t flushRecorder(void)
{
	uint32_t ret = 0;
	
	if (hFile)
	{
		if (flightIndex) // some still in the pipe.
		{
			bytesNotSavedYet += hFile.write((uint8_t *) flightRecorder, flightIndex * sizeof(oneEntry));
			bytesInFlightRecorder += flightIndex * sizeof(oneEntry);
		}
		
		Serial.printf("flush additional %d entries to %s\n", flightIndex, LOG_FILENAME );

		ret = bytesInFlightRecorder;
		
		bytesNotSavedYet  = 0;
		flightIndex = 0;

	}
	return ret;
}

//-----------------------------------------------------

void powerdownSave(void)
{
	flushRecorder();
	
	Serial.println(FG_CYAN "closing all files before shutdown" FG_DONE);
	if (bytesInFlightRecorder)
	{
		hFile.close();
		bytesInFlightRecorder = 0;
	}
	
	listDir(SD, "/", 2);
	Serial.println("bye");
	delay(3000);
}

void rotateLogs(void)
{
	uint32_t bytesInFile = 0;
	
	flushRecorder();

	if (bytesInFlightRecorder)
	{
		hFile.close();
		// no! do this laterbytesInFlightRecorder = 0;
	}
	
	listDir(SD, "/", 2);

	if (bytesInFlightRecorder)
	{
		// hold 5 versions on SD card if flight recorder was written.
		deleteFile(SD, BACKUP5);
		renameFile(SD, BACKUP4, BACKUP5);
		renameFile(SD, BACKUP3, BACKUP4);
		renameFile(SD, BACKUP2, BACKUP3);
		renameFile(SD, BACKUP1, BACKUP2);
		renameFile(SD, LOG_FILENAME, BACKUP1);

		listDir(SD, "/", 2);
		delay(3000);
	}
	else
	{
		Serial.println("skipping rotate, nothing recorded");
	}

	

	bytesInFile = 0;	
	hFile = SD.open(LOG_FILENAME, FILE_WRITE);
	bytesInFlightRecorder = 0;

	assert(hFile);
	
}



//-----------------------------------------------------

void setup(void)
{

	
	//esp_log_level_set("*", ESP_LOG_ERROR);	// set all components to ERROR level
	esp_log_level_set("*", ESP_LOG_INFO);		// set all components to ERROR level
	esp_log_level_set("wifi", ESP_LOG_WARN);	// enable WARN logs from WiFi stack
	esp_log_level_set("dhcpc", ESP_LOG_INFO);	// enable INFO logs from DHCP client

    //auto cfg = M5.config();
    m5::M5Unified::config_t cfg = M5.config();

    // If you want to use external IMU, write this
	//cfg.external_imu = true;

    M5.begin(cfg);
	Serial.begin(115200);

	_setup_M5();

	setup_SD();
	
    const char *name;
    auto imu_type = M5.Imu.getType();

    switch (imu_type)
    {
	    case m5::imu_none:
	    	name = "not found";
	    	break;

	    case m5::imu_sh200q:
	    	name = "sh200q";
	    	break;

	    case m5::imu_mpu6050:
	    	name = "mpu6050"; 
	    	break;

	    case m5::imu_mpu6886:
	    	name = "mpu6886";
	    	break;

	    case m5::imu_mpu9250:
	    	name = "mpu9250";
	    	break;

	    case m5::imu_bmi270:
	    	name = "bmi270";
	    	numSensorsInIMU = 2; //gyro and accel, no mag
	    	break;

	    default:
	    	name = "unknown";
	    	break;
    }

    if (imu_type == m5::imu_none)
    {
        for (;;)
        {
        	Serial.print('.');
            delay(1000);
            assert(0);
        }
    }

    int32_t displayWidth = display.width();
    int32_t displayHeight = display.height();

	
    if (displayWidth < displayHeight)
    {
        display.setRotation(display.getRotation() ^ 1);
        displayWidth = display.width();
        displayHeight = display.height();
    }

	M5_LOGW("physical display is %d w x %d h\n", displayWidth, displayHeight);

    _setup_RTC();

    int32_t graph_area_h = numSensorsInIMU * numItemsPerSensor * BAR_THICK;
    int32_t text_area_h = displayHeight - graph_area_h;
    
    float fontsize = 3;

    //Serial.printf("graph height=%d text height = %d\n", graph_area_h, text_area_h);
    
    display.setTextSize(fontsize);

	
    topWindow =     { 0,                0, displayWidth, displayHeight, TFT_WHITE, TFT_BLACK, TFT_WHITE};
    
    graphicWindow = { 0,                0, displayWidth, graph_area_h, TFT_RED, TFT_BLACK, TFT_YELLOW};
    textWindow =    { 0, graph_area_h + 1, displayWidth, text_area_h,  TFT_GREEN, TFT_BLACK, TFT_CYAN};

    // show perimeter of above debug windows.
    display.clear();

	makeWindow(graphicWindow, -5);
	makeWindow(textWindow, -5);

  					 
	display.display();

    delay(2000);

    // Read calibration values from NVS.

    M5_LOGW("IMU displayHeight/displayWidth type :%s", name);
    
	/*	
	M5_LOGW("checking NVS");
    if (M5.Imu.loadOffsetFromNVS())
    {
    	M5_LOGW("Loading data found NVS ... skipping cal");
    }
    else
    {
    	M5_LOGW("Nothing found in NVS");
        startCalibration();
    }
*/


	
	M5.Lcd.setTextSize(2);
	
    //https://doc-tft-espi.readthedocs.io/tft_espi/colors/
	M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
	
	// https://doc-tft-espi.readthedocs.io/tft_espi/datums/
 	M5.Lcd.setTextDatum(TC_DATUM);  // center on X

	hLargeTextArea = myDrawString(textWindow, "GO!", 
								textWindow.win_width/2, textWindow.win_topLeftY, 
								BIG_FONT, 2,
								TFT_YELLOW, TFT_BLACK); 

	M5.Lcd.setTextSize(1);
	hSmallTextArea = myDrawString(textWindow, "ok", 
								textWindow.win_width/2, textWindow.win_topLeftY + textWindow.win_heigth *8/10, 
								STATS_FONT, 1,
								TFT_YELLOW, TFT_BLACK); 

	setup_onPwrDn();
	
	set_factoryDefaults();


#if PLAYBACK
	M5.Speaker.setVolume(20);

	hFile = SD.open(BACKUP3, FILE_READ);
	Serial.printf("MODE : Playback  FILE=%s\n", BACKUP3);
	delay(2000);
	
#endif

#if RECORDING
	deleteFile(SD, "/bench.dat");

	setLongPressCB(powerdownSave);
	setShortPressCB(rotateLogs);

	// do not rotate on power up!
	
	startCalibration(15); // only on record, no point on playback
	
 #endif

	M5.Speaker.setVolume(20);

}

static float max_ACC = 0.0;
static float min_ACC = 0.0;
static float last_ACC = 0.0;

#define REPORT_AFTER_nSAMPLES 10


#define HIST_LEN 300
static float historyMag[HIST_LEN];
static int historyIndex = 0;


bool fread(char *dest, uint32_t len)
{

	if ( hFile.available()) 
	{
		String data;
		data = hFile.readStringUntil('\n');
		//Serial.println(data);
		strncpy (dest, data.c_str(), len);
		return true;
	}
	return false;
}

//-----------------------------------------------------

void loop(void)
{
    static uint32_t imuNumReads = 0;
    static uint32_t prev_sec = 0;
	static uint32_t lastNumSteps;
	static uint16_t lastAction = -1;
	static uint32_t keptSteps;
	static uint32_t oldTime;

	static float peakAnyPlus  = 20;   // typical  plus hard hits go up to 1000
	static float peakAnyMinus = -20;  // typical  minus
	
	static float peakMagPlus  =  0;
	static float peakMagMinus =  99999;
	
	//readPowerButton();
	loop_onPwrDn();
	
	uint32_t stepsNow = getStepsTaken();
	
	char msg[40];
	float now_ACC;
    uint32_t lapTime;
    static float velocity = 0.0;

    // if accel below X keep recording next N samples
    #define KEEP_mpsS 15   // meters per second per second
    #define KEEP_CTR 300   // keep next N samples after accel goes quiet.		
    static uint8_t keepRecordingCtr = 0;
    
	_loop_ota();

    // To update the IMU value, use M5.Imu.update.
    // If a new value is obtained, the return value is non-zero.


#if LIVE || RECORDING
IMU_loop:

    auto bNewImuData = M5.Imu.update();
    if (bNewImuData)

    {
    	static uint32_t stopWatch;
    	if (!stopWatch) 
    	{
    		stopWatch = micros();
    		return;
    	}

    	lapTime = micros() - stopWatch;
    	stopWatch += lapTime;


    	// tbd, doesn't see 100% monotonic ? why?
		//Serial.println(lapTime);
		//return;
		
    		
        // Obtain data on the current value of the IMU.
        m5::IMU_Class::imu_data_t data = M5.Imu.getImuData();
        
		// bug with BMI270. accel is where gyro is and vicea versa
		// swap now so all consumers don't have to swap.
		
		#ifdef BMI_270
			auto temp = data.accel;
			data.accel = data.gyro;
			data.gyro = temp;
        #endif
        
        drawImuStats(graphicWindow, data);

		now_ACC = sqrt(data.accel.x * data.accel.x +
					   data.accel.y * data.accel.y + 
					   data.accel.z * data.accel.z);

		if (now_ACC > KEEP_mpsS ) keepRecordingCtr = KEEP_CTR;
		
		// need two samples to make a difference.
		static bool bFirstAcc = true;
		if (bFirstAcc)
		{
			bFirstAcc = false;
			last_ACC = now_ACC;
			return;	  // see you next time thru.
		}
		
		// now_ACC is always postive, are we increasing or decreasing
		
		float deltaACC = now_ACC - last_ACC;
		
		last_ACC = now_ACC;
		
		if (max_ACC < deltaACC) max_ACC = deltaACC;
		if (min_ACC > deltaACC) min_ACC = deltaACC;

		// velocity will always be positive because we cannot determine direction
		// from an absolute accel value.

		// sometimes it goes a tenth of a point below zero. 
		velocity += deltaACC; // dont use time as per integration, doesnt work;

		static bool bArmed;

		if (!bArmed  && velocity > 25)
		{
			bArmed = true;
		}

		if (bArmed && velocity < 10)
		{
			bArmed = false;
		}

		if (keepRecordingCtr)
		{
			Serial.printf("%d %8.3f %8.3f\n", bArmed, deltaACC, velocity);
		}
#endif


#define LINE Serial.printf("%s:%d \n", __FUNCTION__, __LINE__);

#if PLAYBACK
	char temp[70];
	uint32_t ftime;
	float	 fvalue;
	
	if ( fread(temp, sizeof(temp))) 
	{
		sscanf(temp, "%d %f", &ftime, &fvalue);

		if (!oldTime)
		{	
			// very first time thru, some setup needed
			oldTime = ftime;
			if ( fread(temp, sizeof(temp)))
			{
				sscanf(temp, "%d %f", &ftime, &fvalue);
			}
			else
			{
				Serial.println("SOURE FILE TOO SMALL ... zzzz");
				delay(-1);
			}
		}

		now_ACC = fvalue;
		lapTime = ftime - oldTime;
		oldTime = ftime;

    	if (lapTime < 4000 || lapTime > 6000) return; // bad data.
    	
#endif

        float lpValACC = 0.0;

        // time in seconds please.
        if (oldTime) lpValACC = run_LP(now_ACC, (float)lapTime/1000000. , 1); // 1 hz lowpass


#if PLAYBACK
		// diff time = 5244 uS rate = 190.7 S/s
        // Serial.printf("ftime = %d diff time = %d uS rate = %.1f S/s\n", ftime, lapTime, 1000000./ (float) lapTime);
        delay(1); 
#endif        
		// ------------------------------------

		static uint16_t cnt;
		cnt++;

#if LIVE
        oldTime = micros();
        
#endif

		// remember now_ACC is always positive. 
		if ( now_ACC < peakMagMinus) peakMagMinus = now_ACC;
		if ( now_ACC > peakMagPlus) peakMagPlus = now_ACC;

		// rolling history
		memcpy (&historyMag[0], &historyMag[1], (HIST_LEN) * sizeof(historyMag[0]));
		
		  historyMag[HIST_LEN-1] = now_ACC;
		//historyMag[HIST_LEN-1] = lpValACC;

#if RECORDING

		if (keepRecordingCtr)
		{
			keepRecordingCtr--;
			
			flightRecorder[flightIndex].bArmed = bArmed;
			flightRecorder[flightIndex].deltaACC = deltaACC;
			flightRecorder[flightIndex++].velocity = velocity;
			
			
			if (flightIndex == FLIGHT_LEN)
			{
				char msg[70];
				uint32_t k;
				M5.Speaker.setVolume(50);

				Serial.println("dump flight recorder to SD"); 

				M5.Speaker.tone(2000, 50); 
				
				bytesNotSavedYet += hFile.write((uint8_t *) flightRecorder, sizeof(flightRecorder));
				bytesInFlightRecorder += sizeof(flightRecorder);

				// do I have to worry about packing?
				assert( sizeof(flightRecorder) == (FLIGHT_LEN * sizeof(oneEntry)));
				
				Serial.printf("flight recorder size = %d \n\n", bytesInFlightRecorder);
				
				flightIndex = 0;

				M5.Speaker.setVolume(20);
				
			}
		}
#endif
		
		//----------------------------------------------------------
#if 0	
		if (cnt == REPORT_AFTER_nSAMPLES)
		{	
			cnt = 0;
			const int SHORT = 20;
			uint32_t j = 0;

			// ---- calc avg
			float longTermAvg = 0;
			float shortTermAvg = 0;
			
			j = 0;			
			for (float  hist : historyMag) 
			{
				longTermAvg += historyMag[j];
				
				if (j > (HIST_LEN - SHORT)) shortTermAvg += historyMag[j];
				j++;
				
				//printf("[%2d] %f\n", j, hist);
			}
			longTermAvg /= (float) j;
			shortTermAvg /= SHORT;
			
			const int8_t hysterisis = 10;
			
			// bias up the line for display purposes

			static float decide;
			static bool bLastState;
			
			    if (lpValACC > longTermAvg + hysterisis)
			    {
					decide = -5;
					if (!bLastState) M5.Speaker.tone(2000, 100);
					bLastState = 1;
			    }
			    else if (lpValACC < longTermAvg - hysterisis)
			    {
			    	decide = 0;
					if (bLastState) M5.Speaker.tone(1000, 100);
					bLastState = 0;
			    }
			    else
			    {
					//decide = -20;  // use last decide
			    }

			// Print sensor data in CSV format for Serial Studio visualization
			Serial.printf("%d\t%f\t%f\t%f\n", (int) decide, longTermAvg , shortTermAvg, lpValACC);


			M5_LOGD("|A| = %f", now_ACC);
			M5_LOGD("steps %d ", getStepsTaken());
			M5_LOGD(" ");

			max_ACC = 0.0;
			min_ACC = 0.0;
		}
#endif

		//----------------------------------------------------------

		// force display update ever 250mS
		static uint32_t dispTime;
		if (dispTime < millis())
		{
			dispTime = millis() + 250;
			sprintf(msg, "%d=%d%% %s %d", M5.Power.getBatteryVoltage(), M5.Power.getBatteryVoltage()*100/4170, activity2string(lastAction), stepsNow);
			myRefreshString(textWindow, hSmallTextArea, msg);
        }
		
		
		if (lastNumSteps != stepsNow)
		{
			lastNumSteps = stepsNow;
			//sprintf(msg, "bat=%d%%% %s %d", M5.Power.getBatteryVoltage()*100/4170, activity2string(lastAction), stepsNow);
			sprintf(msg, "%d=%d%% %s %d", M5.Power.getBatteryVoltage(), M5.Power.getBatteryVoltage()*100/4170, activity2string(lastAction), stepsNow);
			myRefreshString(textWindow, hSmallTextArea, msg);
		}

		uint16_t actionNow = getActivity();
		if (lastAction != actionNow)
		{
		
			if (actionNow == 0) 
			{
				keptSteps = lastNumSteps;
				resetStepCtr();
			}
			M5_LOGD("actionNow = %s", activity2string(actionNow));

			lastAction = actionNow;
			//sprintf(msg, "bat=%d%% %s %d", M5.Power.getBatteryVoltage()*100/4170, activity2string(actionNow), lastNumSteps);
			sprintf(msg, "%d=%d%% %s %d", M5.Power.getBatteryVoltage(), M5.Power.getBatteryVoltage()*100/4170, activity2string(lastAction), stepsNow);
			myRefreshString(textWindow, hSmallTextArea, msg);

			sprintf(msg, "%d", keptSteps);
			myRefreshString(textWindow,hLargeTextArea, msg); 
		}
	
        ++imuNumReads;
    }

#if PLAYBACK
    else
    {
    	Serial.printf("end of data\n");
    	delay(-1);
    }
#endif 

}


#if 0
			/*			
						// Z-axis accel (m/s^2)
						Serial.printf("%f\t%f\t%f\n", data.accel.x * 100.,
													  data.accel.y * 100.,
													  data.accel.z * 100.);
						Serial.print(" ");
						Serial.print(data.gyro.x);	// X-axis gyroscope (deg/s)
						Serial.print(" ");
						Serial.print(data.gyro.y);	// Y-axis gyroscope (deg/s)
						Serial.print(" ");
						Serial.print(data.gyro.z);	// Z-axis gyroscope (deg/s)
						Serial.print(" ");
			*/			
			
			#if 1
			Serial.printf("%d, %d, %d, %d, %d, %d\n", (int) (10. * data.accel.x), 
										  (int) (10. * data.accel.y),
										  (int) (10. * data.accel.z),
										  (int) (10. * MAG_ACC),
										  (int) (10. * peakAnyPlus),
										  (int) (10. * peakAnyMinus)  );
			#else

			Serial.printf("ACC-x:%d\n", (int) (10. * data.accel.x));
			Serial.printf("ACC-y:%d\n", (int) (10. * data.accel.y));
			Serial.printf("ACC-z:%d\n", (int) (10. * data.accel.z));
			Serial.printf("ACC-hi:%d\n", (int) (10. * peakAnyPlus));
			Serial.printf("ACC-lo:%d\n", (int) (10. * peakAnyMinus));
			#endif
			
			/*
			timbit = micros();
			//float uSperSample = (float)REPORT_AFTER_nSAMPLES / (float) (timbit - stopWatch);
			float uSperSample = (float) (REPORT_AFTER_nSAMPLES * 1000000)/(float) (timbit - stopWatch);
            
			Serial.printf("uS\/sample = %.1f\n", uSperSample);
			stopWatch = timbit;

			answer: 180uS per sample
			*/
			
			// accel +-100
			M5_LOGD("ax:%+9.7f  ay:%+9.7f  az:%+9.7f", data.accel.x, data.accel.y, data.accel.z);

			// gyro +- 1.0000
			M5_LOGD("gx:%+9.7f  gy:%+9.7f  gz:%+9.7f", data.gyro.x , data.gyro.y , data.gyro.z );

			//M5_LOGD("mx:%+9.7f  my:%+9.7f  mz:%+9.7f", data.mag.x  , data.mag.y  , data.mag.z  );

			M5_LOGD("|G| = %f", MAG_GYRO);
			M5_LOGD("%.1f < |A| < %.1f",  min_ACC, max_ACC);
			M5_LOGD("azim = %.1f  elev = %.1f ", azim, elev);


//-------------------------------------------------------------
//-------------------------------------------------------------
double azimuth(Point3D target)
{
    // Define origin and target positions
    Point3D origin = {0.0, 0.0, 0.0};
    //Point3D target = {10.0, 10.0, 5.0}; // Northeast quadrant

    // Compute differences
    double dx = target.x - origin.x;
    double dy = target.y - origin.y;

    // Calculate azimuth (Clockwise from North)
    double azimuth_rad = atan2(dx, dy);
    double azimuth_deg = azimuth_rad * (180.0 / M_PI);

    // Keep angle positive between 0 and 360 degrees
    //if (azimuth_deg < 0) {
    //    azimuth_deg += 360.0;
    //}

    //printf("Target Vector: dx=%.2f, dy=%.2f\n", dx, dy);
    //printf("Calculated Azimuth: %.2f degrees\n", azimuth_deg);

    return azimuth_deg;
}

//-------------------------------------------------------------

double elevation(Point3D target) {
    // Define 3D Cartesian coordinates (X, Y, Z)
    double x = target.x;
    double y = target.y;
    double z = target.z; // This is your absolute height/elevation

    // 1. Absolute vertical elevation
    double elevation_value = z;

    // 2. Horizontal distance from the origin in the X-Y plane
    double horizontal_dist = sqrt((x * x) + (y * y));

    // 3. Compute elevation angle (in radians) using atan2 to avoid division-by-zero errors
    double elevation_angle_rad = atan2(elevation_value, horizontal_dist);

    // 4. Convert the angle from radians to degrees
    double elevation_angle_deg = elevation_angle_rad * (180.0 / M_PI);

	return elevation_angle_deg;
}

			
#endif			


