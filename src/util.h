#include <Arduino.h>
#include <SdFat.h>

#ifndef UTIL_H
#define UTIL_H

/**
 * Generates and selects an unused filename from the SD card. Currently implemented in an inefficient way that
 * doesn't matter because it runs in setup and nowhere else. It works by incrementally mutating the passed buffer
 * with filenames ranging from "D0.BIN" to "D999.BIN", then checking if they already exist on the SD card.
 * If not, it returns. The program is expected to use the mutated char buffer to get the filename. It should
 * only be called AFTER SD has successfully been initialized.
 * */
//CREDIT FOR THIS CODE GOES TO RAHUL
void select_next_filename(char *buffer, SdFs *sd) { //Passed buff should be of size FILENAME_SIZE
    for (int fileNum = 0; fileNum < 1000; fileNum++) {
        char fileNumber[5]; //4-character number + null
        sprintf(fileNumber, "%d", fileNum);
        strcpy(buffer, "DAT");
        strcat(buffer, fileNumber);
        strcat(buffer, ".csv");
        //debugl(buffer);
        if (!sd->exists(buffer)) {
            return;
        }
    }
}

#endif
