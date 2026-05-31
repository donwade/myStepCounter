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

struct rect_t
{
    int32_t topLeftX;
    int32_t topLeftY;
    int32_t rectW;
    int32_t rectH;
};

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
static rect_t graphicWindow;
static rect_t textWindow;

static uint8_t calib_countdown = 0;

static int prev_xpos[18];
void drawBar(int32_t topLeftX, int32_t topLeftY, int32_t offsetX, int32_t width, int32_t heigth, uint32_t color)
{
    uint32_t bgcolor = (color >> 3) & 0x1F1F1Fu;

    if (width && ((offsetX < 0) != (width < 0)))
    {
    	// a bar from left edge to the middle of display
        display.fillRect(topLeftX, topLeftY, width, heigth, bgcolor);
        width = 0;
    }

    if (width != offsetX)
    {
    	// a bar from middle of display towards right edge of specifed width
        if ((offsetX > width) != (offsetX < 0))
            bgcolor = color;

        display.setColor(bgcolor);
        //               |<-right edge X-->|         |<- draw leftwards->|
        display.fillRect(offsetX + topLeftX, topLeftY, width - offsetX  , heigth);
    }
}



void drawImuStatsII(const rect_t& r, const m5::imu_data_t& data)
{
    int topLeftX = (r.topLeftX + r.rectW) /2;  // move to horizontal center point.
    int topLeftY = r.topLeftY;
    
    int heightY = BAR_THICK;
    
    int bar_count = numSensorsInIMU * numItemsPerSensor;

    display.startWrite();

	//Serial.printf("bar_count = %d\n", bar_count);
	// data.accel[3] + data.gyro[3] = 6 items

    int barNum;
    for (barNum = 0; barNum < bar_count; ++barNum)
    {
        float xval;

		auto coe = coefficient_tbl[barNum / 3] * r.rectW;
		xval = data.value[barNum] * coe;
  
        // for Linear scale graph.
        float tmp = xval;

        // The smaller the value, the larger the amount of change in the graph.
		//  float tmp = sqrtf(fabsf(xval * 128)) * (signbit(xval) ? -1 : 1);

        int offsetX = tmp;
        int widthX = prev_xpos[barNum];

        if (offsetX != widthX)
            prev_xpos[barNum] = offsetX;

        drawBar(topLeftX, 
        		topLeftY + heightY * barNum, 
        		offsetX, 
        		widthX, 
        		heightY - 1, 
        		color_tbl[barNum]);
    }

    display.endWrite();
    //Serial.printf("x aph display ends at %d\n", topLeftY + heightY * barNum);
    //Serial.printf("ssss = %d\n", r.rectH);
}

void drawImuStats(const rect_t& r, const m5::imu_data_t& data)
{
    int topLeftX = (r.topLeftX + r.rectW) /2;  // move to horizontal center point.
    int topLeftY = r.topLeftY;
    
    int heightY = BAR_THICK;
    
    int bar_count = numSensorsInIMU * numItemsPerSensor;

    display.startWrite();

	//Serial.printf("bar_count = %d\n", bar_count);
	// data.accel[3] + data.gyro[3] = 6 items

    int barNum;
    for (barNum = 0; barNum < bar_count; ++barNum)
    {
        float xval;

		auto coe = coefficient_tbl[barNum / 3] * r.rectW;
		xval = data.value[barNum] * coe;
 

        int offsetX = xval;
        int widthX = prev_xpos[barNum];

        if (offsetX != widthX)
            prev_xpos[barNum] = offsetX;

        drawBar(topLeftX, 
        		topLeftY + heightY * barNum, 
        		offsetX, 
        		widthX, 
        		heightY - 1, 
        		color_tbl[barNum]);
    }

    display.endWrite();
    //Serial.printf("x aph display ends at %d\n", topLeftY + heightY * barNum);
    //Serial.printf("ssss = %d\n", r.rectH);
}

//---------------------------------------------------------------------

#if 0 // USELESS

#include <MahonyAHRS.h>
Mahony filter;

inline float  DEGREES(float x) { return (x * 180. / 3.14159);}

void showMahony(float fGx,float fGy, float fGz)
{
  int ax, ay, az;
  int gx, gy, gz;

  float roll, pitch, yaw;

  // Update the Mahony filter, with scaled gyroscope
  float gyroScale =  1;  // TODO: the filter updates too fast
  filter.updateIMU(DEGREES(fGx * gyroScale),
  				   DEGREES(fGy * gyroScale),
  				   DEGREES(fGz * gyroScale),
  				   ax, ay, az);

  static uint32_t ticker;
  if (millis() > ticker)
  {
    ticker = millis() + 1000;
    // print the yaw, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    Serial.printf("yaw = %+5.1f pitch = %+5.1f roll = %+5.1f\n", yaw, pitch, roll);
  }

} 
#endif

//---------------------------------------------------------------------
//---------------------------------------------------------------------
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
    				 textWindow.rectW, 
    				 textWindow.rectH, 
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

void showRect(char *msg, rect_t *reader)
{
	M5_LOGW("%s x=%d y=%d w=%d h=%d\n", msg, reader->topLeftX, reader->topLeftY, reader->rectW, reader->rectH);
}
//-------------------------------------------------------------

void setup(void)
{
    //auto cfg = M5.config();
    m5::M5Unified::config_t cfg = M5.config();

    // If you want to use external IMU, write this
	//cfg.external_imu = true;

    M5.begin(cfg);
	Serial.begin(115200);

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


    //int32_t graph_area_h = ((displayHeight - 8) / BAR_THICK) * BAR_THICK;
    int32_t graph_area_h = numSensorsInIMU * numItemsPerSensor * BAR_THICK;
    int32_t text_area_h = displayHeight - graph_area_h;
    
    float fontsize = 3;

    Serial.printf("graph height=%d text height = %d\n", graph_area_h, text_area_h);
    
    display.setTextSize(fontsize);

    graphicWindow = { 0, 0, displayWidth, graph_area_h };
    textWindow = { 0, graph_area_h, displayWidth, text_area_h };

    // show perimeter of above debug windows.
    display.clear();

	showRect("graphicWindow", &graphicWindow);
	display.drawRect(graphicWindow.topLeftX, graphicWindow.topLeftY, 
					 graphicWindow.rectW, graphicWindow.rectH, 
					 TFT_BLUE);
	display.display();
    
	showRect("textWindow", &textWindow);
	display.drawRect(textWindow.topLeftX, textWindow.topLeftY, 
					 textWindow.rectW, textWindow.rectH, 
					 TFT_YELLOW);
					 
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

    _setup_RTC();
}

void loop(void)
{
    static uint32_t imuNumReads = 0;
    static uint32_t prev_sec = 0;

    // To update the IMU value, use M5.Imu.update.
    // If a new value is obtained, the return value is non-zero.
    
    auto bNewImuData = M5.Imu.update();

    if (bNewImuData)
    {
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

#if 1
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

		static uint16_t cnt;
		cnt++;

		
		if (cnt > 300)
		{	
			cnt = 0;
			M5_LOGI("ax:%+9.7f  ay:%+9.7f  az:%+9.7f", data.accel.x, data.accel.y, data.accel.z);
			M5_LOGI("gx:%+9.7f  gy:%+9.7f  gz:%+9.7f", data.gyro.x , data.gyro.y , data.gyro.z );
		  //M5_LOGI("mx:%+9.7f  my:%+9.7f  mz:%+9.7f", data.mag.x  , data.mag.y  , data.mag.z  );
			M5_LOGI("|G| = %f  |A| = %f", MAG_GYRO, MAG_ACC);
			M5_LOGI(" ");
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
