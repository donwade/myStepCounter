// If you use Unit OLED, write this.
// #include <M5UnitOLED.h>

// If you use Unit LCD, write this.
// #include <M5UnitLCD.h>


// Include this to enable the M5 global instance.
#include <M5Unified.h>
//#include <MahonyAHRS.h>
#include <_RTC.h>

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
    int32_t topLeftX;
    int32_t topLeftY;
    int32_t width;
    int32_t heigth;
    uint32_t textForegndC;
    uint32_t textBackgndC;
    uint32_t boarderC;
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

static auto &display = (M5.Display);
static window_t graphicWindow;
static window_t textWindow;
static window_t topWindow;

static uint8_t calib_countdown = 0;

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
    int midPointX = r.topLeftX + r.width/2;  // move to horizontal center point.
    int topLeftY = r.topLeftY;
        
    int bar_count = numSensorsInIMU * numItemsPerSensor;  //0,1,2 accel xyz  4,5,6 gyro xyz
    int barThick = r.heigth/ bar_count; 

    display.startWrite();

	//Serial.printf("bar_count = %d\n", bar_count);
	// imuDirect.accel[3] + imuDirect.gyro[3] = 6 items

    int barNum;
    for (barNum = 0; barNum < bar_count; ++barNum)
    {
        float xval;

		auto coe = coefficient_tbl[barNum / 3] * r.width;
		xval = imuDirect.value[barNum] * coe;
 

        int newWidth = xval;
        int oldWidth = prev_xpos[barNum];

        int maxw = r.width/2 -1;
        
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

void updateCalibration(uint32_t uCalCount, bool bForceStart = false)
{
    calib_countdown = uCalCount;

	static uint32_t stopwatch;


	// what do if count hits zero.
    if (uCalCount == 0) bForceStart = true;

    if (bForceStart)
    {
        memset(prev_xpos, 0, sizeof(prev_xpos));
        display.fillScreen(TFT_BLACK);

        if (uCalCount)
        { 
        	M5.Speaker.tone(2000, 300);
        	delay(1000);
        	
        	// Start calibration.
			Serial.printf("start calibration ... depth = %d of 255\n", calDepth);
			
            M5.Imu.setCalibration(calDepth, calDepth, calDepth);
          	stopwatch = millis();
          	
            // ※ The actual calibration operation is performed each time during M5.Imu.update.
            //
            // There are three arguments, which can be specified in the order of Accelerometer, gyro, and geomagnetic.
            // If you want to calibrate only the Accelerometer, do the following.
            // M5.Imu.setCalibration(100, 0, 0);
            //
            // If you want to calibrate only the gyro, do the following.
            // M5.Imu.setCalibration(0, 100, 0);
            //
            // If you want to calibrate only the geomagnetism, do the following.
            // M5.Imu.setCalibration(0, 0, 100);
        }
        else
        { 
        	// Stop calibration. (Continue calibration only for the geomagnetic sensor)
        	M5.Speaker.tone(2000, 200);
        	delay(200);
        	M5.Speaker.tone(1000, 200);

 			Serial.printf("stop  calibration ... depth = %d of 255 time = %d mS\n", calDepth, millis()-stopwatch);
                                     
            M5.Imu.setCalibration(0, //accel
            					  0, //gyro
            					  0  //calDepth
            					  );

            // If you want to stop all calibration, write this.
            // M5.Imu.setCalibration(0, // accel
            //						 0, // gyro
            //						 0  // mag
            //						 );

            // save calibration values.

            M5_LOGW("saving to NVS");
            
            M5.Imu.saveOffsetToNVS();
        }
    }

    auto backcolor = (uCalCount == 0) ? TFT_BLACK : TFT_BLUE;

	// clear text window.
    display.fillRect(textWindow.topLeftX,
    				 textWindow.topLeftY, 
    				 textWindow.width, 
    				 textWindow.heigth, 
    				 backcolor);

    if (uCalCount)
    {
        display.setCursor(textWindow.topLeftX + 2, textWindow.topLeftY + 1);
        display.setTextColor(TFT_WHITE, TFT_BLUE);
        display.printf("Countdown:%d ", uCalCount);

		M5.Speaker.tone(900, 100);
    }
}

//-------------------------------------------------------------

void startCalibration(void)
{
    updateCalibration(10, true);
}
//-------------------------------------------------------------

void showRect(char *msg, window_t &reader)
{
	M5_LOGW("%s x=%d y=%d w=%d h=%d\n", msg, reader.topLeftX, reader.topLeftY, reader.width, reader.heigth);
}

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

//-------------------------------------------------------------
void makeWindow(window_t &win, int8_t boarder)
{
	display.fillRect(win.topLeftX, win.topLeftY, 
					 win.width, win.heigth, 
					 win.boarderC);

	showRect("in",  win);

	// boarder < 0. shrink ... > 0 grow
	
	win.heigth 		+= boarder * 2;
	win.width 		+= boarder * 2;
	win.topLeftX 	-= boarder;   // <0 = shrinking,  move in + dir
	win.topLeftY 	-= boarder;
	
	showRect("out",  win);
	display.fillRect(win.topLeftX, win.topLeftY, 
					 win.width, win.heigth, 
					 win.textBackgndC);

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
	if (bgColour != TFT_BLACK )
	{
		M5_LOGW("only TFT_BLACK supported for background");
		bgColour = TFT_BLACK;
	}
	
	M5.Lcd.setTextColor(fgColour, bgColour);

	M5.Lcd.setTextSize(size);
	M5.Lcd.drawString(msg, topX, topY, font); 
	window.tinfo[0].heigth = M5.Lcd.fontHeight(font) + VSPACE;
	window.tinfo[0].width = M5.Lcd.textWidth(msg);
	window.tinfo[0].topLeftX = topX;
	window.tinfo[0].topLeftY = topY;
	window.tinfo[0].font = font;
	window.tinfo[0].tsize = size;
	window.tinfo[0].bgColor = bgColour;
	window.tinfo[0].fgColor = fgColour;
	
	strncpy(window.tinfo[0].msg, msg, sizeof(window.tinfo[0].msg));
	window.tinfo[0].handle = micros();
	display.display();
	
	return window.tinfo[0].handle;
};

uint32_t  myRefreshString(window_t &window, uint32_t handle, char *msg )
{

	if (window.tinfo[0].handle)
	{
		// erase previous
		M5.Lcd.setTextColor(window.tinfo[0].bgColor, window.tinfo[0].bgColor);
	
		M5.Lcd.setTextSize( window.tinfo[0].tsize);
		
		M5.Lcd.drawString(window.tinfo[0].msg, 
						  window.tinfo[0].topLeftX, window.tinfo[0].topLeftY,
						  window.tinfo[0].font); 
	}
	
	//display.display();
	//delay(4000);
	
	M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
	M5.Lcd.setTextColor(window.tinfo[0].fgColor, window.tinfo[0].bgColor);
	
	M5.Lcd.drawString(msg, window.tinfo[0].topLeftX, window.tinfo[0].topLeftY,
						 window.tinfo[0].font); 
	window.tinfo[0].width = M5.Lcd.textWidth(msg);

	strncpy(window.tinfo[0].msg, msg, sizeof(window.tinfo[0].msg));
	
	display.display();
	return window.tinfo[0].handle;
};
    

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
	M5.Speaker.setVolume(32);

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

    Serial.printf("graph height=%d text height = %d\n", graph_area_h, text_area_h);
    
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



	
	M5.Lcd.setTextSize(2);
	
    //https://doc-tft-espi.readthedocs.io/tft_espi/colors/
	M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
	
	// https://doc-tft-espi.readthedocs.io/tft_espi/datums/
 	M5.Lcd.setTextDatum(TC_DATUM);  // center on X

	uint32_t foo = myDrawString(textWindow, "NOW", 
								textWindow.width/2, textWindow.topLeftY, 
								BIG_FONT, 2,
								TFT_GREEN, TFT_BLACK); 

}

static float MAX_ACC = 0.0;
static float MIN_ACC = 0.0;
static float VELOCITY = 0.0;

static float LAST_ACC = 0.0;

#define REPORT_TIMEmS  100
#define HYSTERESYS   200

static uint32_t hysteresis;

#define PROFILING 0


void loop(void)
{
    static uint32_t imuNumReads = 0;
    static uint32_t prev_sec = 0;


    // To update the IMU value, use M5.Imu.update.
    // If a new value is obtained, the return value is non-zero.

	delay(10);    
    auto bNewImuData = M5.Imu.update();

    if (bNewImuData)
    {
    
		//	see below, end result = 11.0116 mS/sample

#if PROFILING
    	{
    		// run profiling
			static int32_t profileCtr = 3;
			static uint32_t profileTime;
			#define NUM_SAMPLES 1000
	
			if (profileCtr)
			{
				profileCtr--;
			}
			else
			{
				uint32_t diffTime = micros() - profileTime;
				M5_LOGI("%d %.1f uS/sample" , diffTime, (float)diffTime /NUM_SAMPLES);
				profileTime = micros();
				profileCtr = NUM_SAMPLES;
			}
		}
#endif

        // Obtain data on the current value of the IMU.
        m5::IMU_Class::imu_data_t data = M5.Imu.getImuData();
        
		// auto data = blah blah;
    	//char *hareball;
        //auto data = M5.Imu.getImuData();
        //hareball = data;

		// bug with BMI270. accel is where gyro is and vicea versa
		// swap now so all consumers don't have to swap.
		
		#ifdef BMI_270
			auto temp = data.accel;
			data.accel = data.gyro;
			data.gyro = temp;
        #endif
        
        drawImuStats(graphicWindow, data);

#if !PROFILING
		// The data obtained by getImuData can be used as follows.
		data.accel.x;       // accel x-axis value.
		data.accel.y;       // accel y-axis value.
		data.accel.z;       // accel z-axis value.
		//data.accel.value; // accel 3values array [0]=x / [1]=y / [2]=z.

		data.gyro.x;       // gyro x-axis value.
		data.gyro.y;       // gyro y-axis value.
		data.gyro.z;       // gyro z-axis value.
		//data.gyro.value; // gyro 3values array [0]=x / [1]=y / [2]=z.

		data.mag.x;       // mag x-axis value.
		data.mag.y;       // mag y-axis value.
		data.mag.z;       // mag z-axis value.
		//data.mag.value; // mag 3values array [0]=x / [1]=y / [2]=z.

		// interesting.... a 3x3 array of everthing.
		//data.value;      // all sensor 9values array [0~2]=accel / [3~5]=gyro / [6~8]=mag

		float MAG_GYRO;

		// normalize gyro magnitude (always should be
		MAG_GYRO = sqrt(data.gyro.x * data.gyro.x +
						data.gyro.y * data.gyro.y + 
						data.gyro.z * data.gyro.z) / sqrt(3.0);

		float MAG_ACC;
		MAG_ACC = sqrt(data.accel.x * data.accel.x +
					   data.accel.y * data.accel.y + 
					   data.accel.z * data.accel.z);

		float holdACC;
		holdACC = (LAST_ACC < MAG_ACC) ? -MAG_ACC : MAG_ACC;

		VELOCITY += holdACC;
		
		if (MAX_ACC < holdACC) MAX_ACC = holdACC;
		if (MIN_ACC > holdACC) MIN_ACC = holdACC;

		LAST_ACC = MAG_ACC;
		
		
		static uint32_t lastReportMs;
		static bool state;

		Point3D stick;
		stick.x = data.gyro.x;
		stick.y = data.gyro.y;
		stick.z = data.gyro.z;

		double elev = elevation(stick);
		double azim = azimuth(stick);

		if ( millis() >  REPORT_TIMEmS + lastReportMs)
		{	
			lastReportMs = millis();

			if ( state )
			{
				if ( VELOCITY < -HYSTERESYS)
				{
					M5.Speaker.tone(3000, 100);
					state = false;	// look for - next time.
					M5_LOGI("ax:%+9.7f	ay:%+9.7f  az:%+9.7f", data.accel.x, data.accel.y, data.accel.z);
					M5_LOGI("%.1f < |A| < %.1f",  MIN_ACC, MAX_ACC);
					M5_LOGI("V = %.1f ", VELOCITY);
					M5_LOGI(" ");
				}
			}
			else
			{
				if (VELOCITY > +HYSTERESYS)
				{
					M5.Speaker.tone(2000, 100);
					state = true;		// look for + next time
					M5_LOGI("ax:%+9.7f	ay:%+9.7f  az:%+9.7f", data.accel.x, data.accel.y, data.accel.z);
					M5_LOGI("%.1f < |A| < %.1f",  MIN_ACC, MAX_ACC);
					M5_LOGI("V = %.1f ", VELOCITY);
					M5_LOGI(" ");
				}
			}
			
		  //M5_LOGI("ax:%+9.7f  ay:%+9.7f  az:%+9.7f", data.accel.x, data.accel.y, data.accel.z);
		  //M5_LOGI("gx:%+9.7f  gy:%+9.7f  gz:%+9.7f", data.gyro.x , data.gyro.y , data.gyro.z );
		  //M5_LOGI("mx:%+9.7f  my:%+9.7f  mz:%+9.7f", data.mag.x  , data.mag.y  , data.mag.z  );
		  //M5_LOGI("|G| = %f  |A| = %f", MAG_GYRO, MAG_ACC);
		  
		  //M5_LOGI("%.1f < |A| < %.1f",  MIN_ACC, MAX_ACC);
		  //M5_LOGI("%.1f ", VELOCITY);
			
		  //M5_LOGI("azim = %.1f  elev = %.1f ", azim, elev);


			// new game.
			MAX_ACC = 0.0;
			MIN_ACC = 0.0;
			VELOCITY = 0.0;
			

			static uint32_t loopy;
			char msg[100];
			sprintf(msg, "%d", loopy++);
			
			myRefreshString(textWindow, 0, msg);
		}
#endif

	
        ++imuNumReads;
    }
    else
    {
        M5.update();

        // Calibration is initiated when a button or screen is clicked.
        if (M5.BtnA.wasClicked() || M5.BtnPWR.wasClicked() || M5.Touch.getDetail().wasClicked())
            startCalibration();
    }

    int32_t secondsPassed = millis() / 1000;

    if (prev_sec != secondsPassed)
    {
        prev_sec = secondsPassed;
        
        //M5_LOGI("secondsPassed:%d  frame:%d", secondsPassed, imuNumReads);
        //imuNumReads = 0;

        if (calib_countdown)
            updateCalibration(calib_countdown - 1);

        if ((secondsPassed & 7) == 0) // prevent WDT.
            vTaskDelay(1);
    }
}
