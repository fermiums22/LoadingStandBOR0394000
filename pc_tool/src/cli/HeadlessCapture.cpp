#include "cli/HeadlessCapture.h"

#include "can/SlcanParser.h"
#include "elf/ElfSymbolLoader.h"
#include "protocol/DebugProtocol.h"
#include "transport/ICanTransport.h"
#include "transport/SlcanTransport.h"
#include "transport/TcpCanTransport.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace drivescope {

namespace {
struct RuntimeWatch : VariableSymbol {
    bool enabled = true;
    float hz = 10.0f;
    double nextPoll = 0.0;
    double pendingSince = 0.0;
    bool pending = false;
    uint8_t seq = 0;
};

uint32_t pendingKey(uint8_t nodeId, uint8_t seq)
{
    return (static_cast<uint32_t>(nodeId) << 8) | seq;
}

uint8_t sizeForType(const std::string& type)
{
    if (type == "uint8" || type == "int8" || type == "u8" || type == "i8") return 1;
    if (type == "uint16" || type == "int16" || type == "u16" || type == "i16") return 2;
    if (type == "double") return 8;
    return 4;
}

std::vector<VariableSymbol> loadAllSymbols(const std::vector<std::string>& files)
{
    std::vector<VariableSymbol> out;
    ElfSymbolLoader loader;
    for (const std::string& file : files) {
        std::vector<VariableSymbol> loaded;
        std::string error;
        if (!loader.loadSymbolsJson(file, loaded, error)) {
            std::fprintf(stderr, "[capture] symbols skipped: %s (%s)\n", file.c_str(), error.c_str());
            continue;
        }
        out.insert(out.end(), loaded.begin(), loaded.end());
    }
    return out;
}

bool resolveWatch(const WatchConfig& cfg,
                  const std::vector<VariableSymbol>& symbols,
                  RuntimeWatch& out)
{
    VariableSymbol sym;
    if (!cfg.hasAddress) {
        const std::string& wanted = cfg.symbolName.empty() ? cfg.name : cfg.symbolName;
        const auto it = std::find_if(symbols.begin(), symbols.end(), [&](const VariableSymbol& s) {
            return s.name == wanted;
        });
        if (it == symbols.end()) {
            std::fprintf(stderr, "[capture] watch symbol not found: %s\n", wanted.c_str());
            return false;
        }
        sym = *it;
        sym.nodeId = cfg.nodeId;
        if (!cfg.name.empty()) sym.name = cfg.name;
    } else {
        sym.name = cfg.name.empty() ? cfg.symbolName : cfg.name;
        sym.address = cfg.address;
        sym.type = cfg.type;
        sym.size = cfg.size;
        sym.nodeId = cfg.nodeId;
    }

    if (sym.address == 0 || sym.name.empty()) return false;
    static_cast<VariableSymbol&>(out) = sym;
    out.enabled = cfg.enabled;
    out.hz = cfg.hz;
    return true;
}

std::unique_ptr<ICanTransport> makeTransport(const ToolConfig& config)
{
    if (config.transport == "slcan") {
        auto transport = std::make_unique<SlcanTransport>();
        transport->portName = config.slcan.port;
        transport->baud = config.slcan.baud;
        transport->nominalCommand = config.slcan.nominal;
        transport->dataCommand = config.slcan.data;
        transport->silentMode = config.slcan.silent;
        return transport;
    }

    auto transport = std::make_unique<TcpCanTransport>();
    transport->host = config.tcp.host;
    transport->port = config.tcp.port;
    return transport;
}

void writeCsvHeader(std::ofstream& csv)
{
    csv << "timestamp,node_id,name,address,type,value\n";
}
} // namespace

int runHeadlessCapture(const ToolConfig& config, double durationSec, const std::string& csvPath)
{
    std::vector<VariableSymbol> symbols = loadAllSymbols(config.symbolFiles);
    std::vector<RuntimeWatch> watches;
    for (const WatchConfig& watch : config.watches) {
        RuntimeWatch runtime;
        if (resolveWatch(watch, symbols, runtime)) watches.push_back(std::move(runtime));
    }

    if (watches.empty()) {
        std::fprintf(stderr, "[capture] no watches resolved\n");
        return 2;
    }

    std::unique_ptr<ICanTransport> transport = makeTransport(config);
    if (!transport->open()) {
        std::fprintf(stderr, "[capture] transport open failed: %s\n", transport->status().c_str());
        return 3;
    }

    std::ofstream csv(csvPath, std::ios::binary);
    if (!csv) {
        std::fprintf(stderr, "[capture] cannot open csv: %s\n", csvPath.c_str());
        return 4;
    }
    writeCsvHeader(csv);

    DebugProtocol debug;
    SlcanParser clock;
    std::unordered_map<uint32_t, size_t> pending;
    uint8_t nextSeq = 1;
    const double start = clock.nowSeconds();
    const double end = start + durationSec;
    size_t samples = 0;

    std::printf("[capture] connected: %s\n", transport->status().c_str());
    std::printf("[capture] watches=%zu duration=%.3f csv=%s\n", watches.size(), durationSec, csvPath.c_str());

    while (clock.nowSeconds() < end) {
        CanFrame frame;
        for (int i = 0; i < 1000 && transport->poll(frame); ++i) {
            const auto rsp = debug.parseReadMemResponse(frame);
            if (!rsp) continue;
            const auto it = pending.find(pendingKey(rsp->nodeId, rsp->seq));
            if (it == pending.end() || it->second >= watches.size()) continue;

            RuntimeWatch& watch = watches[it->second];
            pending.erase(it);
            watch.pending = false;

            bool ok = false;
            double value = 0.0;
            if (rsp->status == 0) value = decodeValue(rsp->value, watch.type, ok);
            const std::string valueText = formatValue(value, watch.type, ok);
            csv << frame.timestamp << ','
                << static_cast<int>(watch.nodeId) << ','
                << watch.name << ",0x" << std::hex << std::uppercase << watch.address << std::dec << ','
                << watch.type << ','
                << valueText << '\n';
            ++samples;

            if (ok) {
                std::printf("%.6f node=%u %-28s %s\n",
                            frame.timestamp,
                            static_cast<unsigned>(watch.nodeId),
                            watch.name.c_str(),
                            valueText.c_str());
            } else {
                std::printf("%.6f node=%u %-28s status=%u\n",
                            frame.timestamp,
                            static_cast<unsigned>(watch.nodeId),
                            watch.name.c_str(),
                            static_cast<unsigned>(rsp->status));
            }
        }

        const double now = clock.nowSeconds();
        for (size_t i = 0; i < watches.size(); ++i) {
            RuntimeWatch& watch = watches[i];
            if (!watch.enabled || now < watch.nextPoll) continue;
            if (watch.pending && now - watch.pendingSince < 0.5) continue;

            const uint8_t size = sizeForType(watch.type);
            if (size > 4) continue;

            DebugReadRequest req;
            req.nodeId = watch.nodeId;
            req.seq = nextSeq++;
            req.address = watch.address;
            req.size = size;
            CanFrame tx = debug.makeReadMem(req, now);
            if (transport->send(tx)) {
                watch.pending = true;
                watch.pendingSince = now;
                watch.seq = req.seq;
                pending[pendingKey(req.nodeId, req.seq)] = i;
            }
            watch.nextPoll = now + 1.0 / std::max(0.1f, watch.hz);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    transport->close();
    csv.flush();
    std::printf("[capture] samples=%zu\n", samples);
    return samples > 0 ? 0 : 5;
}

} // namespace drivescope
