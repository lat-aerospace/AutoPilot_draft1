// ======================================================================
// \title  Main.cpp
// \brief  SITL deployment entry point
// ======================================================================

#include <AP/Top/Sitl/SitlTopology.hpp>
#include <signal.h>
#include <getopt.h>
#include <cstdlib>
#include <Os/Os.hpp>

void print_usage(const char* app) {
    (void)printf("Usage: ./%s [options]\n-a\thostname/IP address\n-p\tport_number\n", app);
}

static void signalHandler(int signum) {
    Sitl::stopRateGroups();
}

int main(int argc, char* argv[]) {
    Os::init();
    U16 port_number = 0;
    I32 option = 0;
    char* hostname = nullptr;

    while ((option = getopt(argc, argv, "hp:a:")) != -1) {
        switch (option) {
            case 'a':
                hostname = optarg;
                break;
            case 'p':
                port_number = static_cast<U16>(atoi(optarg));
                break;
            case 'h':
            case '?':
            default:
                print_usage(argv[0]);
                return (option == 'h') ? 0 : 1;
        }
    }

    Sitl::TopologyState inputs;
    inputs.hostname = hostname;
    inputs.port = port_number;

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    (void)printf("Hit Ctrl-C to quit\n");

    Sitl::setupTopology(inputs);
    // 400 Hz base tick = 2.5 ms = 0 seconds + 2500 microseconds
    Sitl::startRateGroups(Fw::TimeInterval(0, 2500));
    Sitl::teardownTopology(inputs);
    (void)printf("Exiting...\n");
    return 0;
}
