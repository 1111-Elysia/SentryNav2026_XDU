#include <pybind11/pybind11.h>
#include <pybind11/functional.h>   // 支持回调函数
#include <pybind11/stl.h>          // 支持 std::vector/std::map
#include <librm.hpp>
#include <cstdint>
#include <vector>
#include <functional>

namespace py = pybind11;

// 类型别名，方便写回调
using CmdCallback = std::function<void(uint16_t cmd_id, uint8_t seq)>;

class PyRefereeWrapper {
public:
    PyRefereeWrapper()
        : ref_(rm::device::Referee<rm::device::RefereeRevision::kV170>())
    {}

    // 传入单个字节
    void feed_byte(uint8_t byte) {
        ref_ << byte;
    }

    // 传入一组字节
    void feed_bytes(const std::vector<uint8_t> &data) {
        for (auto b : data)
            ref_ << b;
    }

    // 获取解析后的数据对象
    auto get_data() {
        return ref_.data();
    }

    // 注册回调函数
    void attach_callback(CmdCallback cb) {
        ref_.AttachCallback([cb](uint16_t cmd_id, uint8_t seq){
            cb(cmd_id, seq);
        });
    }

private:
    rm::device::Referee<rm::device::RefereeRevision::kV170> ref_;
};

PYBIND11_MODULE(pyreferee, m) {
    py::class_<PyRefereeWrapper>(m, "Referee")
        .def(py::init<>())
        .def("feed_byte", &PyRefereeWrapper::feed_byte)
        .def("feed_bytes", &PyRefereeWrapper::feed_bytes)
        .def("get_data", &PyRefereeWrapper::get_data)
        .def("attach_callback", &PyRefereeWrapper::attach_callback);
}
