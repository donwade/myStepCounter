#include <M5Unified.h>
#include <Preferences.h>

#include "stepConfig.h"

#include "src/bmi270-defs.h"
/*
1. SC_26.watermark_level â€“ watermark level; 
the step counter will trigger output every time specific number of
steps are counted

2. SC_26.reset_counter â€“ flag to reset the counted steps.
Step count value can be reset only when any one of
features mentioned in this register is enabled.

3. SC_26.en_counter â€“ indicates if the Step Counter feature is enabled or not.

4. SC_26.en_detector â€“ indicates if the Step Detector feature is enabled or not.

5. SC_26.en_activity â€“ indicates if the activity feature is enabled or not

6. SC_1.param_1 to SC_25.param_25 â€“ there are 25 parameters, 
   which can customize the sensitivity of the Step
*/

Preferences myNv;

// from the BMI250 doc for a pedometer.
char bin[65+7];  // max 64 bit plus zero terminator plus some dots

template <typename T>
char * showAsBinary ( T value)
{
	int i = sizeof(value) * 8 -1;
	
	bin[0] = 0;
	for (int j = i; j > -1; j--)
	{
		value & (1 << j) ? strcat(bin,"1") : strcat(bin,"0");
		if (j && !((j) % 4)) strcat(bin,"."); 
	}
	return bin;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------



typedef struct STEP_ENTRY
{
	uint8_t stepCtrRegNum;   // a step counter reg is not same as phys reg num
	int16_t value;
};

static STEP_ENTRY cmdSetup[]=
{
	{ 1,	301},	// 0000.0001.0010.1101
	{ 2,  31700},	// 0111.1011.1101.0100
	{ 3,	315},	// 0000.0001.0011.1011
	{ 4,  31451},	// 0111.1010.1101.1011
	{ 5,	  4},	// 0000.0000.0000.0100
	{ 6,  31551},	// 0111.1011.0011.1111
	{ 7,  27853},	// 0110.1100.1100.1101
	{ 8,   1219},	// 0000.0100.1100.0011
	{ 9,   2437},	// 0000.1001.1000.0101
	{ 10,   1219},	// 0000.0100.1100.0011
	{ 11,  -6420},	// 1110.0110.1110.1100
	{ 12,  17932},	// 0100.0110.0000.1100
	{ 13,	  1},	// 0000.0000.0000.0001
	{ 14,	 39},	// 0000.0000.0010.0111
	{ 15,	 25},	// 0000.0000.0001.1001
	{ 16,	150},	// 0000.0000.1001.0110
	{ 17,	160},	// 0000.0000.1010.0000
	{ 18,	  1},	// 0000.0000.0000.0001
	{ 19,	 12},	// 0000.0000.0000.1100
	{ 20,  15600},	// 0011.1100.1111.0000
	{ 21,	256},	// 0000.0001.0000.0000
	{ 22,	  1},	// 0000.0000.0000.0001
	{ 23,	  3},	// 0000.0000.0000.0011
	{ 24,	  1},	// 0000.0000.0000.0001
	{ 25,	 14},	// 0000.0000.0000.1110

};

// see 5.2 of documentation for top view of register layout.

#define ENTRIES(x) (sizeof (x) / sizeof(x[0]))
//----------------------------------------------------------------------------

static void changeSCpage(uint8_t page, bool bQuiet)
{
	static uint16_t lastPage = 0xFFFF;
	if (page != lastPage)
	{
		if (!bQuiet) log_i("changing page from 0x%X to 0x%X", lastPage, page);
		M5.Imu.write8(0x2F, page, bQuiet);
		lastPage = page;
	}
}


// load up step counter configuration. all 25 register settings.
// ONLY used for loading up the mfg table.

void SCnum2physical(uint8_t scRegNumber, uint8_t &page, uint8_t &index)
{
	
	assert(scRegNumber > 0 && scRegNumber < 28);
	
	// SC NUMBERS ARE 1 BASED, convert to zero based for programmers
	uint8_t zeroBased = scRegNumber - 1;	// make zero based.
	
	page  = zeroBased / 8 + 3;  		 // sc are kept in bank 3,4,5,6
	
	index = (zeroBased % 8) * 2 + 0x30; // sc are kept in regs 0x30..0x3E;
	changeSCpage(page, 1);

}
//-----------------------------------------------------------------------------

// normal way to write to the feature reg set. 

// must originate from enums SC_FEATURE, WAKEUP_FEATURE, GESTURE_FEATURE

uint16_t readFeature( uint16_t enumReg , bool bQuiet)
{	
	uint16_t retval;
	uint8_t page = enumReg >> 8;
	uint8_t index = enumReg & 0xFF;

	changeSCpage(page, bQuiet);
	
	retval = M5.Imu.read16(index, retval, bQuiet);
	return retval;
}


void writeFeature( uint16_t enumReg, uint16_t value, bool bQuiet)
{
 	uint8_t page = enumReg >> 8;
	uint8_t index = enumReg & 0xFF;

	changeSCpage(page, bQuiet);

	uint16_t red = readFeature(enumReg, bQuiet);
	log_i("1] {0x%04X} >> 0x%4X %s", enumReg, red, showAsBinary(red));
	
	M5.Imu.write16(index, value, bQuiet);

	log_i("2] {0x%04X} << 0x%4X %s", enumReg, value, showAsBinary(value));

	red = readFeature(enumReg, bQuiet);
	log_i("3] {0x%04X} >> 0x%4X %s", enumReg, red, showAsBinary(red));

}


// read modify write a field of bits for a feature.

uint16_t RMWFeature( uint16_t enumReg, uint8_t LHS, uint8_t RHS, 
					 bool bQuiet, uint16_t value, char *msg)
{

	assert ( RHS <= LHS);

	uint8_t width = LHS - RHS + 1;
	uint16_t mask  = (1 << (width))-1;

	assert (width < 16);
	
	assert (! (value & ~mask));  // value has bits outside of mask 

	mask = mask << RHS;

 	uint8_t page = enumReg >> 8;
	uint8_t index = enumReg & 0xFF;

	changeSCpage(page, bQuiet);

	uint16_t red = readFeature(enumReg, bQuiet);

	if(!bQuiet) log_d("in  {0x%04X} >> 0x%4X %s", enumReg, red, showAsBinary(red));
	red &= ~mask;
	red |= (value << RHS);
	if (!bQuiet) log_d("out {0x%04X} >> 0x%4X %s", enumReg, red, showAsBinary(red));

	// show final result
	log_d("=== %s ===", msg );
	log_d("{0x%04X} LHS=%d RHS=%d width=%d value=0x%X result=%s", enumReg, LHS, RHS, width, value, showAsBinary(red));

	
	M5.Imu.write16(index, red, bQuiet);

	return red;
	
}

//----------------------------------------------------------------------------
#include <nvs_flash.h>
void eraseNV(void)
{
	nvs_flash_erase();
	nvs_flash_init();
	log_i("NV flash erased ... rebooting");
	delay(3000);
	ESP.restart();
}
//----------------------------------------------------------------------------

/*
1. SC_26.watermark_level – watermark level; the step counter will trigger output every time specific number of
steps are counted
2. SC_26.reset_counter – flag to reset the counted steps. Step count value can be reset only when any one of
features mentioned in this register is enabled.
3. SC_26.en_counter – indicates if the Step Counter feature is enabled or not.
4. SC_26.en_detector – indicates if the Step Detector feature is enabled or not.
5. SC_26.en_activity – indicates if the activity feature is enabled or not
6. SC_1.param_1 to SC_25.param_25 – there are 25 parameters, which can customize the sensitivity of the Step
Counter and Detector.
*/


// these are the values for programming a step counter.
#include <Preferences.h>
#include "esp_partition.h"
//-------------------------------------------------------------

void writeFactoryNv(bool bQuiet)
{

	// Create an instance of the Preferences library
	Preferences myNV;

	// Open NVS namespace named "BMI270". 
	// False means read/write mode. True means read-only.
	// Note: Namespace names must be 15 characters or less!
	myNV.begin("BMI270", false);

	// test from nothing    myNV.clear();
	
	for (int stepRegister = 1; stepRegister < 26; stepRegister ++)
	{
		char cKey[40];
		uint16_t scWriteVal, uKeyRead;
		uint8_t page, index;
		
		SCnum2physical(stepRegister, page,index);
		sprintf(cKey, "SC-%d", stepRegister);

		//log_w("reading ... key %s does %s exist", cKey, !myNV.isKey(cKey) ? "NOT":"");
		//uKeyRead = myNv.getUShort(cKey, 0);
		//log_w("read of %s finds %d", cKey, uKeyRead);
		
		scWriteVal = M5.Imu.read16(index, scWriteVal, 1);

		if (myNV.isKey(cKey))
		{
			uKeyRead = myNV.getUShort(cKey, scWriteVal); // if key doesn't exist, force value.
			if (uKeyRead != scWriteVal)
			{
				log_d("difference in %s values %d vs %d ... updating", cKey, uKeyRead, scWriteVal);
				assert( myNV.putUShort(cKey, scWriteVal) == 2); // 2 bytes should be written.
			}
			else
			{
				log_d("skipping %s values identical %d", cKey, uKeyRead);
			}
		}
		else
		{
			log_d("making new key %s = %d", cKey, scWriteVal);
			uKeyRead = myNV.getUShort(cKey, scWriteVal); // if key doesn't exist, force value.
			assert( myNV.putUShort(cKey, scWriteVal) == 2); // 2 bytes should be written.
		}

 	}


	// Always close the myNV to release resources
	myNV.end();

	
	//Serial.println("Data updated. Restarting ESP32 in 10 seconds...\n");
	//delay(10000);
	//ESP.restart(); // Restart to see the counter increase

}

//-------------------------------------------------------------

bool restoreFromFactoryNv(bool bQuiet)
{

	// Create an instance of the Preferences library
	Preferences myNV;

	// Open NVS namespace named "BMI270". 
	// False means read/write mode. True means read-only.
	// Note: Namespace names must be 15 characters or less!
	myNV.begin("BMI270", false);

	// test from nothing    myNV.clear();

	// test to see if all keys are in place, else fail out return = true;

	char cKey[40];
	uint16_t scWriteVal, uKeyRead;
	uint8_t page, index;
	
	
	for (int stepRegister = 1; stepRegister < 26; stepRegister ++)
	{
		SCnum2physical(stepRegister, page,index);
		sprintf(cKey, "SC-%d", stepRegister);
		if (myNV.isKey(cKey))
		{
			log_d("testing key %s pass", cKey);
			continue;
		}
		log_e("cannot restore from NV, not all keys present");
 		
		// Always close the myNV to release resources
		myNV.end();
		return 1; // fail
		
 	}
 	
	for (int stepRegister = 1; stepRegister < 26; stepRegister ++)
	{
		char cKey[40];
		uint16_t scWriteVal, uKeyRead;
		uint8_t page, index;
		
		SCnum2physical(stepRegister, page,index);
		sprintf(cKey, "SC-%d", stepRegister);
		
		if (myNV.isKey(cKey))
		{
			uKeyRead = myNV.getUShort(cKey, scWriteVal); // if key doesn't exist, force value.
			log_w("[SC=%d]	page %02d index 0x%X <= %6d (NV)", 
				  stepRegister, page, index, uKeyRead);
			M5.Imu.write16(index , uKeyRead, 1);
		}
		else
		{
			// earlier test should have caught this. game over.
			assert(stepRegister != stepRegister);

			delay(10000);
			ESP.restart(); // Restart to see the counter increase
		}
	}

	// Always close the myNV to release resources
	myNV.end();
	return 0;    //success
	
}

void load_factoryDefaultsFromROM(void)
{
	uint8_t page;
	uint8_t index;

	log_w("%s: setting factory SC defaults -----------------------", __FUNCTION__);

	for (int j = 0; j < ENTRIES(cmdSetup); j++)
	{
		SCnum2physical(cmdSetup[j].stepCtrRegNum, page,index);
		log_w("[SC=%d]  page %02d index 0x%X <= %6d (ROM)", 
			  cmdSetup[j].stepCtrRegNum, page, index, cmdSetup[j].value);
		M5.Imu.write16(index , cmdSetup[j].value, 1);
 	}
	
 	log_w("factory step counter defaults done -------------------");

 	
}

//-------------------------------------------------------------
void set_factoryDefaults(void)
{

	if (restoreFromFactoryNv(0))
	{
		// error was found loading from NV
		load_factoryDefaultsFromROM();
		writeFactoryNv(0);
	}
	
	//RMWFeature(SC_26, 11, 11, 1, 1, "enable detector");
	RMWFeature(SC_26, 12, 12, 1, 1, " enable counter");
	RMWFeature(SC_26, 13, 13, 1, 1, " enable walking, running etc ");
	RMWFeature(SC_26,  9,  0, 1, 1, " report on every 1 step");

	resetStepCtr();
}
//-------------------------------------------------------------
void resetStepCtr(void)
{
	RMWFeature(SC_26, 10, 10, 1, 1, " reset on");
	delay(10);
	RMWFeature(SC_26, 10, 10, 1, 1, " out of reset");
	delay(10);
}

//-------------------------------------------------------------

uint32_t getStepsTaken()
{
	uint16_t lo = readFeature(SC_OUT_0_1, 1);
	uint16_t hi = readFeature(SC_OUT_2_3, 1);
	return  (hi << 16) | lo;
}
//-------------------------------------------------------------
uint8_t getActivity()
{
	return readFeature(ACT_OUT, 1);
}
//-------------------------------------------------------------
const char* activity2string(uint8_t act)
{
	if (act > 3) return "---";
	
	const char *msgs[] = {"still", "walking", "running", "unknown"};
	return msgs[act];
}

