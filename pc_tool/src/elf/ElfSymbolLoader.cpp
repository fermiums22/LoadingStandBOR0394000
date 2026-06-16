#include "elf/ElfSymbolLoader.h"

#include <fstream>
#include <regex>
#include <sstream>

namespace drivescope {

namespace {
std::string readFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string jsonString(const std::string& obj, const char* key, const std::string& fallback = {})
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch m;
    if (!std::regex_search(obj, m, re)) return fallback;
    return m[1].str();
}

uint32_t jsonUint(const std::string& obj, const char* key, uint32_t fallback = 0)
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*(0x[0-9A-Fa-f]+|[0-9]+)");
    std::smatch m;
    if (!std::regex_search(obj, m, re)) return fallback;
    return static_cast<uint32_t>(std::stoul(m[1].str(), nullptr, 0));
}
} // namespace

bool ElfSymbolLoader::loadSymbolsJson(const std::string& path,
                                      std::vector<VariableSymbol>& symbols,
                                      std::string& error)
{
    const std::string text = readFile(path);
    if (text.empty()) {
        error = "empty or missing symbols file";
        return false;
    }

    std::vector<VariableSymbol> out;
    const std::regex objRe("\\{[^\\}]*\\}");
    for (auto it = std::sregex_iterator(text.begin(), text.end(), objRe);
         it != std::sregex_iterator(); ++it) {
        const std::string obj = it->str();
        VariableSymbol sym;
        sym.name = jsonString(obj, "name");
        sym.address = jsonUint(obj, "address");
        sym.type = jsonString(obj, "type", "uint32");
        sym.size = static_cast<uint8_t>(jsonUint(obj, "size", 4));
        sym.nodeId = static_cast<uint8_t>(jsonUint(obj, "node_id", 2));
        sym.file = jsonString(obj, "file");
        if (!sym.name.empty() && sym.address != 0 && sym.size > 0) out.push_back(sym);
    }

    if (out.empty()) {
        error = "no symbols found";
        return false;
    }

    symbols = std::move(out);
    error.clear();
    return true;
}

} // namespace drivescope
