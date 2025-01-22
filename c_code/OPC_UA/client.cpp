#include <iostream>

#include <open62541pp/client.hpp>
#include <open62541pp/node.hpp>

// Soruces
// https://github.com/open62541pp/open62541pp/tree/master

int main()
{
    opcua::Client client;
    client.connect("opc.tcp://localhost:4840");

    opcua::Node node(client, opcua::VariableId::Server_ServerStatus_CurrentTime);
    const auto dt = node.readValueScalar<opcua::DateTime>();

    std::cout << "Server date (UTC): " << dt.format("%Y-%m-%d %H:%M:%S") << std::endl;
}
