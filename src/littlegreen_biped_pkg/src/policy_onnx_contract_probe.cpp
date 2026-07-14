// Inspect a policy ONNX tensor contract using the same ONNX Runtime installation
// used by littlegreen_biped_node. Output is a compact JSON object for
// policy_bundle_audit.py.

#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace
{
std::string shape_json(const std::vector<int64_t>& shape)
{
    std::ostringstream stream;
    stream << '[';
    for (std::size_t i = 0; i < shape.size(); ++i) {
        if (i != 0U) {
            stream << ',';
        }
        stream << shape[i];
    }
    stream << ']';
    return stream.str();
}

std::string json_escape(const std::string& value)
{
    std::ostringstream stream;
    for (const char ch : value) {
        switch (ch) {
        case '\\': stream << "\\\\"; break;
        case '"': stream << "\\\""; break;
        case '\n': stream << "\\n"; break;
        case '\r': stream << "\\r"; break;
        case '\t': stream << "\\t"; break;
        default: stream << ch; break;
        }
    }
    return stream.str();
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: policy_onnx_contract_probe POLICY.onnx\n";
        return 64;
    }

    try {
        Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "littlegreen_policy_probe");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        Ort::Session session(environment, argv[1], options);

        if (session.GetInputCount() != 1U || session.GetOutputCount() != 1U) {
            std::cerr << "policy must expose exactly one input and one output tensor\n";
            return 2;
        }

        Ort::AllocatorWithDefaultOptions allocator;
        const std::string input_name(
            session.GetInputNameAllocated(0, allocator).get());
        const std::string output_name(
            session.GetOutputNameAllocated(0, allocator).get());
        const auto input_info = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
        const auto output_info = session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
        const auto input_shape = input_info.GetShape();
        const auto output_shape = output_info.GetShape();

        std::cout
            << "{\"input_name\":\"" << json_escape(input_name)
            << "\",\"output_name\":\"" << json_escape(output_name)
            << "\",\"input_shape\":" << shape_json(input_shape)
            << ",\"output_shape\":" << shape_json(output_shape)
            << ",\"input_element_type\":"
            << static_cast<int>(input_info.GetElementType())
            << ",\"output_element_type\":"
            << static_cast<int>(output_info.GetElementType())
            << "}" << std::endl;
        return 0;
    } catch (const Ort::Exception& error) {
        std::cerr << "ONNX Runtime error: " << error.what() << '\n';
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "probe error: " << error.what() << '\n';
        return 70;
    }
}
