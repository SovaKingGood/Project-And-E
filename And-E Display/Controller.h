//Display
#include <Bluepad32.h>

void setupController();
void pollController();
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);

extern ControllerPtr myControllers[1];