#include <_m5Core2-only.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "pretty.h"

#include "sd-logger.h"

#define STFU

#ifndef STFU
#define xprintln
#define xprint
#define xprintf
#else
#define xprintln 	Serial.println
#define xprint 		Serial.print
#define xprintf 	Serial.printf
#endif


void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  xprintf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    xprintln("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    xprintln("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      xprint("  DIR : ");
      xprintln(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      xprint("  FILE: ");
      xprint(file.name());
      xprint("  SIZE: ");
      xprintln(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  xprintf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    xprintln("Dir created");
  } else {
    xprintln("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  xprintf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    xprintln("Dir removed");
  } else {
    xprintln("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  xprintf("Reading file: %s\n", path);

  File file = fs.open(path, FILE_READ);
  if (!file) {
    xprintln("Failed to open file for reading");
    return;
  }

  xprint("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  xprintf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    xprintln("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    xprintln("File written");
  } else {
    xprintln("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  //xprintf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    assert(0);
    return;
  }
  if (file.print(message)) {
    //xprintf("appending %s", message);
  } else {
    Serial.println("Append failed");
    assert(0);
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  xprintf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
  } else {
    xprintln("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  xprintf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
  } else {
    xprintln("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    xprintf("%lu bytes read for %" PRIu32 " ms\n", (unsigned long)flen, end);
    file.close();
  } else {
    xprintln("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    xprintln("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  xprintf("%u bytes written for %" PRIu32 " ms\n", 2048 * 512, end);
  file.close();
}


void setup_SD()
{
 	static const gpio_num_t SDCARD_CSPIN = GPIO_NUM_4;

	static bool bInited = false;
	bool ok;
	if (!bInited)
	{
		bInited = true;
		ok = SD.begin(SDCARD_CSPIN, SPI, 2000000);
		Serial.printf(FG_CYAN "SD card is %s READY\n" FG_DONE, ok ? "" : "NOT");
	}


	uint8_t cardType = SD.cardType();

	if (cardType == CARD_NONE) {
	Serial.println("No SD card attached");
	return;
	}

	Serial.print("SD Card Type: ");
	if (cardType == CARD_MMC) {
	Serial.println("MMC");
	} else if (cardType == CARD_SD) {
	Serial.println("SDSC");
	} else if (cardType == CARD_SDHC) {
	Serial.println("SDHC");
	} else {
	Serial.println("UNKNOWN");
	}

	uint64_t cardSize = SD.cardSize() / (1024 * 1024);
	Serial.print("SD Card Size: ");
	Serial.print(cardSize);
	Serial.println("MB");

	listDir(SD, "/", 0);

#if 0
  createDir(SD, "/mydir");
  listDir(SD, "/", 0);
  
  removeDir(SD, "/mydir");
  listDir(SD, "/", 2);
  
  writeFile(SD, "/hello.txt", "Hello ");
  
  appendFile(SD, "/hello.txt", "World!\n");

  readFile(SD, "/hello.txt");

  deleteFile(SD, "/foo.txt");

  renameFile(SD, "/hello.txt", "/foo.txt");

  readFile(SD, "/foo.txt");

  testFileIO(SD, "/test.txt");
#endif

  Serial.print("Total space: ");
  Serial.print(SD.totalBytes() / (1024 * 1024));
  Serial.println("MB");
  Serial.print("Used space: ");
  Serial.print(SD.usedBytes() / (1024 * 1024));
  Serial.println("MB");
}

void loop_SD() {}

