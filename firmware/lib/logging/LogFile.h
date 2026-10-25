#pragma once

#include <SD.h>

// Opens the first unused "/<prefix>NNN.csv" (NNN = 000..999) on an SD card
// that is already mounted, writes `header` and flushes it. On success the
// chosen name is copied into `name` and the open file is returned in `file`.
bool openNextLogFile(File& file, const char* prefix, const char* header, char* name,
                     size_t nameSize);
