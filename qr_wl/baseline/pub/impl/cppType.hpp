
#pragma once

#include <bitset>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

template <typename T>
using Vec7 = typename Eigen::Matrix<T, 7, 1>;

template <typename T>
using Vec8 = typename Eigen::Matrix<T, 8, 1>;
template <typename T>
using Vec9 = typename Eigen::Matrix<T, 9, 1>;

template <typename T>
using Vec9 = typename Eigen::Matrix<T, 9, 1>;

// 6x1 Vector#include <eigen3/Eigen/Dense>
// 12x1 Vector
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;
// 18x1 Vector
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

// 28x1 vector
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

template <typename T>
using Mat9 = typename Eigen::Matrix<T, 9, 9>;

template <typename T>
using Mat24 = typename Eigen::Matrix<T, 2, 4>;

// 4x1 Vector
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template <typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 Matrix
template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 18x18 Matrix
template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 Matrix
template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 Matrix
template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

template <typename T>
using Mat44 = Eigen::Matrix<T, 4, 4>;

// 3x4 Matrix
template <typename T>
using Mat43 = Eigen::Matrix<T, 4, 3>;

// 3x4 Matrix
template <typename T>
using Mat23 = Eigen::Matrix<T, 2, 3>;

// 4x4 Matrix
template <typename T>
using Mat2 = typename Eigen::Matrix<T, 2, 2>;

// 4x4 Matrix
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 8x6 Matrix
template <typename T>
using Mat86 = typename Eigen::Matrix<T, 8, 6>;

// 8x7 Matrix
template <typename T>
using Mat87 = typename Eigen::Matrix<T, 8, 7>;

template <typename T>
using Mat624 = typename Eigen::Matrix<T, 6, 24>;

// 10x1 Vector
template <typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with spatial vector columns
template <typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

// std::vector (a list) of Eigen things
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

// C++14新写法，参见effective modern c++ 条款10
template <typename T>
constexpr auto Enum2Num(T enumerator) noexcept
{
    return static_cast<std::underlying_type_t<T>>(enumerator);
}

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

// 返回状态，非成功的返回要告知失败原因
enum class RetState
{
    ok,  // 无错误
    netErr,
    outRange,
    timeout,
    noSupport,
    parseErr,
    noUpdate,
    busy,
    interrupt,
    noExist,
    error,  // 其他错误
};

template <typename T>
using Result = std::pair<RetState, T>;

#define UNUSED(x) (void)(x)

/**
 * @description: C结构体转vector<uint8_t>
 * @return {}
 */
template <typename T>
std::vector<uint8_t> Struct2Vector(const T &in)
{
    const uint8_t *rawData = reinterpret_cast<const uint8_t *>(&in);
    std::vector<uint8_t> data(rawData, rawData + sizeof(T));
    return data;
}

/**
 * @description: vector<uint8_t>转C结构体
 * @return {}
 */
template <typename T>
T Vector2Struct(const std::vector<uint8_t> &in)
{
    T structData;
    std::memcpy(&structData, in.data(), in.size());
    return structData;
}
