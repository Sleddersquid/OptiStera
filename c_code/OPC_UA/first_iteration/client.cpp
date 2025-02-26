#include <iostream>

#include <open62541pp/client.hpp>
#include <open62541pp/node.hpp>

// Soruces
// https://open62541pp.github.io/open62541pp/client_minimal_8cpp-example.html

int main()
{
    opcua::Client client;
    client.connect("opc.tcp://sleddersquid:4840");

    opcua::Node node(client, opcua::VariableId::Server_ServerStatus_CurrentTime);
    const auto dt = node.readValueScalar<opcua::DateTime>();

    std::cout << "Server date (UTC): " << dt.format("%Y-%m-%d %H:%M:%S") << std::endl;
}
