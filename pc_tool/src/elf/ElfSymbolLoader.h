#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace drivescope {

struct VariableSymbol {
    std::string name;
    uint32_t address = 0;
    std::string type = "uint32";
    uint8_t size = 4;
    uint8_t nodeId = 2;
    std::string file;
};

class ElfSymbolLoader {
public:
    bool loadSymbolsJson(const std::string& path, std::vector<VariableSymbol>& symbols, std::string& error);
};

} // namespace drivescope
