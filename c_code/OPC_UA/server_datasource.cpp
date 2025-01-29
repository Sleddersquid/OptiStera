#include <iostream>
 
#include <open62541pp/server.hpp>
#include <open62541pp/services/nodemanagement.hpp>

#include <gpiod.hpp>

// Sources:
// https://open62541pp.github.io/open62541pp/server_datasource_8cpp-example.html
 
/// Templated data source that stores the data of type `T` internally.

class GPIOD_line {
private:
    gpiod::line _line;

public:
    GPIOD_line(int line_number) {
        gpiod::chip chip("gpiochip0");
        this->_line = chip.get_line(line_number);
        this->_line.request({"input", gpiod::line_request::DIRECTION_INPUT, 0});
    }

    int get_value() {
        return this->_line.get_value();
    }
};

GPIOD_line button_line(14);

template <typename T>
struct DataSource : public opcua::DataSourceBase {
    opcua::StatusCode read(
        [[maybe_unused]] opcua::Session& session,
        [[maybe_unused]] const opcua::NodeId& id,
        [[maybe_unused]] const opcua::NumericRange* range,
        opcua::DataValue& dv,
        bool timestamp
    ) override {
        dv.setValue(opcua::Variant(button_line.get_value()));
        if (timestamp) {
            dv.setSourceTimestamp(opcua::DateTime::now());
            dv.status();
            // dv.setValue((opcua::Variant)button_line.get_value());
        }
        std::cout << "Read value from data source: " << dv.value().to<T>() << "\n";
        return UA_STATUSCODE_GOOD;
    }
 
    opcua::StatusCode write(
        [[maybe_unused]] opcua::Session& session,
        [[maybe_unused]] const opcua::NodeId& id,
        [[maybe_unused]] const opcua::NumericRange* range,
        const opcua::DataValue& dv
    ) override {
        data = dv.value().to<T>();
        std::cout << "Write value to data source: " << data << "\n";
        return UA_STATUSCODE_GOOD;
    }

    T data{};
};
 
int main() {
    opcua::Server server;
 
    // Add variable node
    const auto node =
        opcua::services::addVariable(
            server,
            opcua::ObjectId::ObjectsFolder,
            {1, 1000},
            "DataSource",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE)
                .setDataType<int>(),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent
        ).value();
 
    // Define data source
    DataSource<int> dataSource;

    server.setVariableNodeDataSource(node, dataSource);
 
    server.run();
}