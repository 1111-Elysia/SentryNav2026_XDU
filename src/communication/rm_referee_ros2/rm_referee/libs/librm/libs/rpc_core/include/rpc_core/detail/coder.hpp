#pragma once

#ifdef RPC_CORE_FEATURE_CODER_VARINT
#include "coder_varint.hpp"
#else
#include "msg_wrapper.hpp"

namespace rpc_core {
namespace detail {

class coder {
 public:
  static std::string serialize(const msg_wrapper& msg) {
    std::string payload;
    payload.reserve(PayloadMinLen + msg.cmd.size() + msg.data.size());
    payload.append((char*)&msg.seq, 4);
    auto cmd_len = (uint16_t)msg.cmd.length();
    payload.append((char*)&cmd_len, 2);
    payload.append((char*)msg.cmd.data(), cmd_len);
    payload.append((char*)&msg.type, 1);
    if (msg.request_payload) {
      payload.append(*msg.request_payload);
    } else {
      payload.append(msg.data);
    }
    return payload;
  }

  static msg_wrapper deserialize(const std::string& payload, bool& ok) {
    msg_wrapper msg;
    if (payload.size() < PayloadMinLen) {
      ok = false;
      return msg;
    }
    char* p = (char*)payload.data();
    const char* pend = payload.data() + payload.size();
    msg.seq = *(seq_type*)p;
    p += 4;
    uint16_t cmd_len = *(uint16_t*)p;
    p += 2;
    if (p + cmd_len + 1 > pend) {
      ok = false;
      return msg;
    }
    msg.cmd.assign(p, cmd_len);
    p += cmd_len;
    msg.type = *(msg_wrapper::msg_type*)(p);
    p += 1;
    msg.data.assign(p, pend - p);
    ok = true;
    return msg;
  }

 private:
  static const uint8_t PayloadMinLen = 4 /*seq*/ + 2 /*cmd_len*/ + 1 /*type*/;
};

}  // namespace detail
}  // namespace rpc_core
#endif
