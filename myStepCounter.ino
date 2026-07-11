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


#define LINE Serial.printf("%s:%d \n", __FUNCTION__, __LINE__);

#define CONTINUOUS  1
#define RECORDING 	1
#define PLAYBACK  	0
#define LIVE        0

#define DEFAULT_VOLUME 80
#define QUIET_VOLUME   30

#define FLIGHT_LOG "/FLIGHT.LOG"
#define BACKUP1 	 "/DATA1.BKU"
#define BACKUP2 	 "/DATA2.BKU"
#define BACKUP3 	 "/DATA3.BKU"
#define BACKUP4 	 "/DATA4.BKU"
#define BACKUP5 	 "/DATA5.BKU"

#define FLIGHT_LEN 2800

typedef struct {
	bool bArmed;
	float deltaACC;
	float iVelocity;
	float lpVelocity;
	float aVelocity;
} oneEntry;


static oneEntry flightRecorder[FLIGHT_LEN];
static int flightIndex = 0;
uint32_t bytesNotSavedYet = 0;
uint32_t bytesInFlightRecorder = 0;

// playback vars
uint32_t totalNumRecordsLoaded = 0;
uint32_t currentReadRecordNum = 0;
uint32_t totalNumberEntries; // file size/oneEntry


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

void startCalibration(void)
{
	uint8_t waitS = 15;
 	Serial.printf("start calibration ... depth = %d of 255\n", calDepth);
 	
    M5.Imu.setCalibration(calDepth, 
    					  calDepth, 
    					  calDepth);

	M5.Speaker.setVolume(QUIET_VOLUME);

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

	M5.Speaker.tone(1000, 100);
 	delay(500);
	M5.Speaker.tone(700, 100);

	M5.Speaker.setVolume(DEFAULT_VOLUME);
 	
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

File hFile;

//-----------------------------------------------------

// ****do not close. this is not your job.
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
		
		Serial.printf("flush additional %d entries to %s\n", flightIndex, FLIGHT_LOG );

		ret = bytesInFlightRecorder;
		
		bytesNotSavedYet  = 0;
		flightIndex = 0;

	}
	else
	{
		Serial.println("why did I fail");		
	}
	return ret;
}

//-----------------------------------------------------

void powerdownSave(void)
{
#if PLAYBACK
	Serial.printf("%s nothing to do in PLAYBACK\n", __FUNCTION__);
	return;
#endif
	flushRecorder();
	
	Serial.println(FG_CYAN "closing all files before shutdown" FG_DONE);

	// flight recorder closed. 
	hFile.close();
	
	rotateLogs();
	
	listDir(SD, "/", 2);
	Serial.println("bye");
	delay(2000);
}

//-----------------------------------------------------

// used for testing rotation and resume.
void quickSave(void)
{
#if PLAYBACK
	Serial.printf("%s nothing to do in PLAYBACK\n", __FUNCTION__);
	return;
#endif
	flushRecorder();
	
	Serial.println(FG_CYAN "closing all files before shutdown" FG_DONE);

	// flight recorder closed. 
	hFile.close();
	rotateLogs();
	
	listDir(SD, "/", 2);

	// this is not a power down.
	// reopen the flight recorder.
	
	hFile = SD.open(FLIGHT_LOG, FILE_WRITE);
	Serial.println ("opening for write " FLIGHT_LOG);
	bytesInFlightRecorder = 0;

	// return and resume logging into a new flight log
}


void rotateLogs(void)
{
		listDir(SD, "/", 2);

#if PLAYBACK
		Serial.printf("%s nothing to do in PLAYBACK\n", __FUNCTION__);
		return;
#endif

	int bExist = SD.exists(FLIGHT_LOG);
	if (bExist)
	{
		// hold 5 versions on SD card if flight recorder was written.
		deleteFile(SD, BACKUP5);
		renameFile(SD, BACKUP4, BACKUP5);
		renameFile(SD, BACKUP3, BACKUP4);
		renameFile(SD, BACKUP2, BACKUP3);
		renameFile(SD, BACKUP1, BACKUP2);
		renameFile(SD, FLIGHT_LOG, BACKUP1);
	
		listDir(SD, "/", 2);
	}
	else
	{
		Serial.println (FLIGHT_LOG " does not exist. No rotate");
	}
}



//-----------------------------------------------------

void setup(void)
{

	
	//esp_log_level_set("*", ESP_LOG_ERROR);	// set all components to ERROR level
	esp_log_level_set("*", ESP_LOG_INFO);		// set all components to ERROR level
	esp_log_level_set("wifi", ESP_LOG_WARN);	// enable WARN logs from WiFi stack
	esp_log_level_set("dhcpc", ESP_LOG_INFO);	// enable INFO logs from DHCP client

#if 0
	{
	
		m5::M5Unified::config_t cfg = M5.config();
	
		cfg.internal_spk = true;
	
		M5.begin(cfg);
		M5.Speaker.begin();
	
		if (!Serial)
			Serial.begin(115200);
	
		M5.Power.setExtOutput(true);	  // enable external bus
	
		// confusing. this sets font for background display
		M5.Lcd.setTextFont(DEFAULT_FONT);
	
		// confusing. this sets font for buttons
		M5.Lcd.setFont(WIDGET_FONT);

		M5.Speaker.setVolume(100);		// Set max volume
		// M5.Speaker.tone(2000, 100);	// no a reboot loop is so annoying
	
		Serial.printf("**** _setup_M5 does not do SD.begin() anymore\n");
		Serial.printf("call _setup_SD AFTER all spi devices claim their access\n");
	
		//_setup_ota();
		_setup_RTC();			  //setup_RTC calls setup OTA
		//_setup_button();
	
	}
#endif

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


	for (int loud = 0; loud < 255; loud += 255/4)
	{
		M5.Speaker.setVolume(loud);
 		M5.Speaker.tone(1500, 50);
		Serial.printf("vol = %d\n", loud);
		delay(500);
	}


#if PLAYBACK

	hFile = SD.open(BACKUP1, FILE_READ);
	Serial.printf(FG_YELLOW "MODE : Playback  FILE=%s\n" FG_DONE, BACKUP1);
	freadOneBlock();
	
#endif

#if RECORDING
	deleteFile(SD, "/bench.dat");

	rotateLogs();

	
	hFile = SD.open(FLIGHT_LOG, FILE_WRITE);
	Serial.println ("opening for write " FLIGHT_LOG);
	bytesInFlightRecorder = 0;
	
	setLongPressCB(powerdownSave);

	setShortPressCB(startCalibration);

 #endif


}

static float last_ACC = 0.0;

#define REPORT_AFTER_nSAMPLES 10


#define HIST_LEN 300
static float hVelocity[HIST_LEN];
static int historyIndex = 0;

//-----------------------------------------------------
uint32_t freadOneBlock()
{
	totalNumberEntries = hFile.size();	// number of bytes.
	totalNumberEntries /= sizeof(oneEntry);

	
	int32_t bytesRead = hFile.read((uint8_t *) flightRecorder, FLIGHT_LEN * sizeof(oneEntry));

	if (bytesRead < 0)
	{
		Serial.printf("cannot continue. file %s returns %d on read\n", FLIGHT_LOG,  bytesRead);
		delay(-1);
	}

	totalNumRecordsLoaded = bytesRead/sizeof(oneEntry);
	
	Serial.printf("%s: bytes read %d, records found %d\n", __FUNCTION__, bytesRead, totalNumRecordsLoaded);
	delay(10000);
	
	currentReadRecordNum = 0;
	return totalNumRecordsLoaded;
}
//-----------------------------------------------------

bool fgetLine(oneEntry &input)
{
	if (currentReadRecordNum == totalNumRecordsLoaded)
	{

		Serial.println("STOP TO TEST"); delay(-1);
		// exhausted current block, get another if possible.
		if (! freadOneBlock()) return false;
		currentReadRecordNum = 0;
	}
	input = flightRecorder[currentReadRecordNum++];
	return true;
}
//-----------------------------------------------------
bool freadLine(char *dest, uint32_t len)
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
    static uint32_t prev_sec = 0;
	static uint32_t lastNumSteps;
	static uint16_t lastAction = -1;
	static uint32_t keptSteps;
	static uint32_t oldTime;

	static uint32_t lineCtr = 0;

	static float peakAnyPlus  = 20;   // typical  plus hard hits go up to 1000
	static float peakAnyMinus = -20;  // typical  minus
	
	static float peakMagPlus  =  0;
	static float peakMagMinus =  99999;

	bool bDecide = false;
	float lpVelocity;
	
	
	//readPowerButton();
	loop_onPwrDn();
	
	static uint32_t stepsNow;
	
	char msg[40];
	float now_ACC;
	
    static float iVelocity = 0.0;

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

		if (CONTINUOUS || now_ACC > KEEP_mpsS ) keepRecordingCtr = KEEP_CTR;
		
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


		
		// iVelocity will always be positive because we cannot determine direction
		// from an absolute accel value.

		// sometimes it goes a tenth of a point below zero. 
		iVelocity += deltaACC; // dont use time as per integration, doesnt work;


        // time in seconds please. is it 5000uS per sample
        lpVelocity = run_LP(iVelocity, 5000./1000000. , 1); // 1 hz lowpass


		// rolling history of iVelocity
		memcpy (&hVelocity[0], &hVelocity[1], (HIST_LEN) * sizeof(hVelocity[0]));
		hVelocity[HIST_LEN-1] = iVelocity;

		float aVelocity =0.0;
		for (int x = 0; x < HIST_LEN; x++)
		{
			aVelocity += hVelocity[x];
		}
		aVelocity /= (float) HIST_LEN;


		#define HYSTERISIS 7.0
		#define DEBOUNCEms 300
		
		static uint8_t bArmed;
		static uint32_t delayDisarm;
		static uint32_t delayArm;
		
		// is low pass above or below slowAvg.

		// keep tone length same for up and down
		if ( (millis() > delayArm) && !bArmed  && lpVelocity > aVelocity + HYSTERISIS )
		{
			stepsNow++;
			bArmed = 1;
			M5.Speaker.tone(1000, 100);

			// now armed, but wont accept a de-arm for at least DEBOUNCEms
			delayDisarm = millis() + DEBOUNCEms;
		}

		// watch for negative num.
		if ((millis() > delayDisarm) && bArmed && lpVelocity < max(aVelocity - HYSTERISIS, 2.0) )
		{
			bArmed = 0;
			delayArm = millis() + DEBOUNCEms;

			// don't play this beep unless debugging debouce
			// M5.Speaker.tone(800, 100); 
		}

#endif

#if PLAYBACK
	oneEntry lineInput;
	float aVelocity;
	
	if ( fgetLine(lineInput)) 
	{
		now_ACC = lineInput.deltaACC;
		iVelocity = lineInput.iVelocity;
		bDecide = lineInput.bArmed;
		lpVelocity = lineInput.lpVelocity;
		aVelocity = lineInput.aVelocity;
		
		Serial.printf("%d %8.3f %8.3f %8.3f %8.3f\n", 
					bDecide * 20, now_ACC, iVelocity, lpVelocity, aVelocity);
    	
#endif

#if LIVE
        oldTime = micros();
        
#endif


#if RECORDING
		if (keepRecordingCtr)
		{
			Serial.printf("%d %8.3f %8.3f %8.3f %8.3f\n", bArmed, deltaACC, iVelocity, lpVelocity, aVelocity);
			
			keepRecordingCtr--;
			
			flightRecorder[flightIndex].bArmed = bArmed;
			flightRecorder[flightIndex].deltaACC = deltaACC;
			flightRecorder[flightIndex].lpVelocity = lpVelocity;
			flightRecorder[flightIndex].aVelocity = aVelocity;
			flightRecorder[flightIndex].iVelocity = iVelocity;
			flightIndex++;
			
			if (flightIndex == FLIGHT_LEN)
			{
				char msg[70];
				uint32_t k;

				Serial.println("dump flight recorder to SD"); 


				// NO NO NO
				// DO NOT PLAY A TONE NEAR A SD WRITE.
				// Doing so, causes the speaker to permanently cut volume 
				// and setVolume is inoperative.
				
				/////M5.Speaker.setVolume(QUIET_VOLUME);
				/////M5.Speaker.tone(2000, 50); 
				
				bytesNotSavedYet += hFile.write((uint8_t *) flightRecorder, sizeof(flightRecorder));
				bytesInFlightRecorder += sizeof(flightRecorder);

				// do I have to worry about packing?
				assert( sizeof(flightRecorder) == (FLIGHT_LEN * sizeof(oneEntry)));
				
				Serial.printf("flight recorder size = %d \n\n", bytesInFlightRecorder);
				
				flightIndex = 0;

				/////M5.Speaker.setVolume(DEFAULT_VOLUME);
				
			}
		}
#endif
		


		//----------------------------------------------------------

		// force display small area update every 250mS
		static uint32_t dispTime;
		if (dispTime < millis())
		{
			dispTime = millis() + 250;
			sprintf(msg, "%d=%d%% %s %d", M5.Power.getBatteryVoltage(), M5.Power.getBatteryVoltage()*100/4170, activity2string(lastAction), stepsNow);
			myRefreshString(textWindow, hSmallTextArea, msg);
        }
		
		// update steps in large window.
		if (lastNumSteps != stepsNow)
		{
			lastNumSteps = stepsNow;
			sprintf(msg, "%d", stepsNow);
			myRefreshString(textWindow,hLargeTextArea, msg); 
		}

/*
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
*/

    }

#if PLAYBACK
    else
    {
    	Serial.printf("end of data\n");
    	delay(-1);
    }
#endif 

}




