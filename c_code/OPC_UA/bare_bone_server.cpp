#include <open62541pp/server.hpp>




int main(int argc, char* argv[]) {
    opcua::Server server;

    server.run();
}