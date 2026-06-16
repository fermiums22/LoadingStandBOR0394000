// Host self-test for the DebugProtocol codec split (Slice 2b):
// ReadMemCodec / CaptureCodec / StreamCodec.
//
// Proves pack/decode are byte-identical to the known golden frames (mirrors
// umbrella protocol/golden_frames.json) and that the DebugProtocol facade
// delegates to the codecs unchanged.
//
//   g++ -std=c++20 -I../src debug_protocol_codecs_selftest.cpp \
//       ../src/protocol/ReadMemCodec.cpp ../src/protocol/CaptureCodec.cpp \
//       ../src/protocol/StreamCodec.cpp ../src/protocol/PropCanFrame.cpp \
//       ../src/protocol/Crc32.cpp ../src/protocol/DebugProtocol.cpp -o dp_selftest
#include "protocol/DebugProtocol.h"
#include "protocol/ReadMemCodec.h"
#include "protocol/CaptureCodec.h"
#include "protocol/StreamCodec.h"

#include <cstdint>
#include <cstdio>
#include <vector>

using namespace drivescope;

static int g_failures = 0;

static void check(const char* name, bool ok)
{
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

static bool sameFrame(const CanFrame& f, uint32_t id, const std::vector<uint8_t>& bytes)
{
    if (f.id != id) return false;
    if (f.data.size() < bytes.size()) return false;
    for (size_t i = 0; i < bytes.size(); ++i) {
        if (f.data[i] != bytes[i]) return false;
    }
    return true;
}

static CanFrame rxFrame(uint32_t id, std::vector<uint8_t> data)
{
    CanFrame f;
    f.id = id;
    f.extended = true;
    f.dlc = 8;
    f.data = std::move(data);
    return f;
}

int main()
{
    // -- READ_MEM request (golden read_mem_req: id 0x10D00210) --------------
    DebugReadRequest rq;
    rq.nodeId = kCanAddrMotor; rq.seq = 1; rq.address = 0x20000240u; rq.size = 4;
    const CanFrame rmReq = readmem::makeRequest(rq, 0.0);
    check("READ_MEM request bytes",
          sameFrame(rmReq, 0x10D00210u, {0x03, 0x01, 0x40, 0x02, 0x00, 0x20, 0x04, 0x00}));

    // facade delegates identically
    check("facade makeReadMem == codec",
          DebugProtocol{}.makeReadMem(rq, 0.0).id == rmReq.id);

    // -- READ_MEM response decode (golden read_mem_rsp: id 0x10D11002) ------
    auto rmRsp = readmem::parseResponse(
        rxFrame(0x10D11002u, {0x83, 0x01, 0x00, 0x04, 0xEF, 0xBE, 0xAD, 0xDE}));
    check("READ_MEM response decode",
          rmRsp && rmRsp->nodeId == kCanAddrMotor && rmRsp->seq == 1 && rmRsp->status == 0 &&
          rmRsp->size == 4 && rmRsp->value.size() == 4 &&
          rmRsp->value[0] == 0xEF && rmRsp->value[3] == 0xDE);

    // -- CAPTURE set_config (golden cap_set_config: id 0x10E00210) ----------
    CaptureConfig cfg;
    cfg.nodeId = kCanAddrMotor; cfg.slotCount = 2; cfg.periodUs = 500;
    cfg.prePct = 20; cfg.postPct = 80; cfg.bufferKb = 32;
    check("CAPTURE set_config bytes",
          sameFrame(capture::makeSetConfig(cfg.nodeId, cfg, 0.0), 0x10E00210u,
                    {0x12, 0x02, 0xF4, 0x01, 0x14, 0x50, 0x20, 0x00}));

    // -- CAPTURE read_chunk (encoder zero-fills reserved bytes 5..7) --------
    CaptureChunkRequest ch; ch.nodeId = kCanAddrMotor; ch.byteOffset = 0; ch.byteCount = 0;
    check("CAPTURE read_chunk bytes",
          sameFrame(capture::makeReadChunk(ch, 0.0), 0x10E00210u,
                    {0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));

    // -- CAPTURE no-arg requests: reserved bytes zero-filled (0x00, not 0xFF) -
    check("CAPTURE reset bytes",
          sameFrame(capture::makeReset(kCanAddrMotor, 0.0), 0x10E00210u,
                    {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    check("CAPTURE arm bytes",
          sameFrame(capture::makeArm(kCanAddrMotor, 0.0), 0x10E00210u,
                    {0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    check("CAPTURE status_req bytes",
          sameFrame(capture::makeStatusReq(kCanAddrMotor, 0.0), 0x10E00210u,
                    {0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
    check("CAPTURE crc_req bytes",
          sameFrame(capture::makeCrcReq(kCanAddrMotor, 0.0), 0x10E00210u,
                    {0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));

    // -- CAPTURE DATA_END parse: byte 7 carries validBytes -----------------
    auto dataEnd = capture::parseData(
        rxFrame(0x10E11002u, {0x98, 0x05, 0x00, 0x36, 0x37, 0x38, 0x39, 0x04}));
    check("CAPTURE DATA_END decode",
          dataEnd && dataEnd->last && dataEnd->byteOffset == 5 && dataEnd->validBytes == 4 &&
          dataEnd->payload[0] == 0x36 && dataEnd->payload[3] == 0x39);

    // -- CAPTURE crc response decode (golden cap_crc_rsp: id 0x10E11002) ----
    auto crc = capture::parseCrc(
        rxFrame(0x10E11002u, {0x96, 0xE7, 0xE6, 0x76, 0x03, 0x09, 0x00, 0xFF}));
    check("CAPTURE crc response decode",
          crc && crc->crc == 0x0376E6E7u && crc->totalBytes == 9 && crc->nodeId == kCanAddrMotor);

    // -- STREAM_ADD request (golden stream_add_req: id 0x10E20210) ----------
    StreamSubscribeRequest sa;
    sa.nodeId = kCanAddrMotor; sa.seq = 1; sa.address = 0x20000240u; sa.sizeType = 0x02; sa.periodHint = 0;
    check("STREAM_ADD request bytes",
          sameFrame(stream::makeAdd(sa, 0.0), 0x10E20210u,
                    {0x01, 0x01, 0x40, 0x02, 0x00, 0x20, 0x02, 0x00}));

    // -- STREAM_VALUE broadcast decode (golden stream_value: id 0x10E8FF02) --
    auto sv = stream::parseValue(
        rxFrame(0x10E8FF02u, {0x00, 0x02, 0xEF, 0xBE, 0xAD, 0xDE, 0x2A, 0x00}));
    check("STREAM_VALUE broadcast decode",
          sv && sv->nodeId == kCanAddrMotor && sv->slotId == 0 && sv->sizeType == 0x02 &&
          sv->valueBytes[0] == 0xEF && sv->valueBytes[3] == 0xDE && sv->tickLsb == 0x2A);

    std::printf("%s\n", g_failures ? "RESULT: FAIL" : "RESULT: PASS");
    return g_failures ? 1 : 0;
}
