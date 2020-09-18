#include <glog/logging.h>

#include "EdgeDevice.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "starting LDS app, argc = " << argc;

    EdgeDevice e("config.json");
    e.runForever();

    return 0;
}
