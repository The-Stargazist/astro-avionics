#include "LogFile.h"

#include <stdio.h>

bool openNextLogFile(File& file, const char* prefix, const char* header, char* name,
                     size_t nameSize) {
    for (int index = 0; index < 1000; ++index) {
        snprintf(name, nameSize, "/%s%03d.csv", prefix, index);
        if (SD.exists(name)) {
            continue;
        }
        file = SD.open(name, FILE_WRITE);
        if (!file) {
            return false;
        }
        file.print(header);
        file.flush();
        return true;
    }
    return false;  // card full of logs
}
