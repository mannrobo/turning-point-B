/**
 * lcd.c - LCD Selection Library
 **/

#pragma systemFile


void lcdClear() {
    clearLCDLine(0);
    clearLCDLine(1);
}


int lcdDebugSlot = 0;

task lcdDebug() {
    while(true) {
        if (nLCDButtons == kButtonLeft && lcdDebugSlot > 0) lcdDebugSlot--;
        if (nLCDButtons == kButtonRight) lcdDebugSlot++;

        lcdClear();
        string lineOne;
        string lineTwo;

        switch(lcdDebugSlot) {
            // Specialized displays for specific situations, not useful in general debug mode


            // Normal access displays
            case 1:
                sprintf(lineOne, "M: %1.2fV", nImmediateBatteryLevel/1000.0);
                sprintf(lineTwo, "E: %1.2fV B: %1.2fV", SensorValue[powerExpander]/270.0, BackupBatteryLevel/1000.0);
                break;
            case 0:
                sprintf(lineOne, "%d,%d", motor[FlywheelA], motor[FlywheelB]);
                sprintf(lineTwo, "%1.2f-%1.2f", robot.flywheel.setpoint, robot.flywheel.process);
                break;
            case 2:
                sprintf(lineOne, "%d,%d", robot.leftDrive, robot.rightDrive);
                sprintf(lineTwo, "%d,%d,%d,%d", motor[DriveFL], motor[DriveFR], motor[DriveBL], motor[DriveBR]);
                break;
            case 3:
                sprintf(lineOne, "%d,%d", robot.leftDrive, robot.rightDrive);
                sprintf(lineTwo, "%d,%d,%f", SensorValue[leftDrive], SensorValue[rightDrive], SensorValue[gyro]);
                break;
            default:
                sprintf(lineOne, "LCD DEBUG SYSTEM");
                sprintf(lineTwo, "Slot %d", lcdDebugSlot);

        }

        displayLCDString(0, 0, lineOne);
        displayLCDString(1, 0, lineTwo);
        wait1Msec(140);
}
}

/**
 * UI Component: Pick between two options, using left and right buttons to choose, and center to confirm
 * @param char * leftOption The option to display on the left
 * @param char * rightOption The option to display on the right
 * @return int The index of the choosen option, 0 for the left option, 1 for the right
 **/
int lcdPick(int line, char * leftOption, char * rightOption) {
    int choice = 0;

    string indicator = "*";
    string spacer = "  ";

    clearLCDLine(line);

    while(nLCDButtons != 2) {
        string buffer = "";
        if (nLCDButtons == 1) choice = 0;
        if (nLCDButtons == 4) choice = 1;

        if(!choice) strncat(buffer, indicator, 1);
        strncat(buffer, leftOption, sizeof(leftOption));
        strncat(buffer, spacer, 2);
        if(choice) strncat(buffer, indicator, 1);
        strncat(buffer, rightOption, sizeof(rightOption));

        displayLCDCenteredString(line, buffer); // All UI components, by default, display on line 1
    }
    lcdClear();
    return choice;

}

/**
 * UI Component: Pick between multiple options
 * @param int line The line to display on
 * @param string options The options to choose from, split by commas
 * @return int The index of the choosen option
 **/
int lcdMenu(int line, string * options, int size) {
	int choice = 0;
	int prev_choice = -1;

    clearLCDLine(line);

	while(true) {
		if (nLCDButtons == kButtonLeft && choice > 0) {
			choice--;
		}
		if (nLCDButtons == kButtonRight && choice < size - 1) {
			choice++;
		}
		if (nLCDButtons == kButtonCenter) {
			break;
		}
		string buffer = (string) options[choice];

		if(prev_choice != choice) {
			displayLCDCenteredString(line, buffer);
            displayLCDChar(line, 1, '<');
            displayLCDChar(line, 14, '>');
		}

		prev_choice = choice;
        wait1Msec(140);
	}

    return choice;
}


void lcdStartup() {
	bLCDBacklight = true;
    lcdClear();

    displayLCDCenteredString(0, "Secure the Flag");
    displayLCDCenteredString(1, "3796B");
    wait1Msec(1000);

    lcdClear();
}
