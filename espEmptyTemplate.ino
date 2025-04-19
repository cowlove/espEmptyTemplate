#include "jimlib.h"
#include "serialLog.h"
#ifndef CSIM
#include "rom/uart.h"
#endif

JStuff j;
CLI_VARIABLE_FLOAT(x, 1);

void setup() {
    j.begin();
    j.cli.on("RESET", [](){ ESP.restart(); });
}

void loop() {
    j.run();
    OUT("loop %f", (float)x);
    delay(1000);
}

#ifdef CSIM
class SketchCsim : public Csim_Module {
    public:
    void setup() { HTTPClient::csim_onPOST("http://.*/log", 
        [](const char *url, const char *hdr, const char *data, string &result) {
 	return 200; }); }
    string dummy;
    void parseArg(char **&a, char **la) override { if (strcmp(*a, "--dummy") == 0) dummy = *(++a); }
    void loop() override {}
} sketchCsim;
#endif
 
