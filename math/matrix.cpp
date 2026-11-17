/**
 * @file matrix.cpp
 * @author LiQun (liqunyan@gmail.com)
 * @brief 列主序矩阵底层运算实现。
 * @version 0.1
 * @date 2026-11-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "matrix.hpp"

namespace matrix
{
    namespace detail
    {
        /**
         * @brief 将右侧缓冲逐元素加到左侧。
         *
         * @tparam T 浮点元素类型。
         * @param lhs 左侧输入输出缓冲。
         * @param rhs 右侧输入矩阵或缓冲。
         * @param count 元素个数。
         */
        template <class T>
        void operations<T>::add(T* lhs, const T* rhs, std::size_t count)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                lhs[i] += rhs[i];
            }
        }

        /**
         * @brief 从左侧缓冲逐元素减去右侧。
         *
         * @tparam T 浮点元素类型。
         * @param lhs 左侧输入输出缓冲。
         * @param rhs 右侧输入矩阵或缓冲。
         * @param count 元素个数。
         */
        template <class T>
        void operations<T>::subtract(T* lhs, const T* rhs, std::size_t count)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                lhs[i] -= rhs[i];
            }
        }

        /**
         * @brief 原位执行标量乘法。
         *
         * @tparam T 浮点元素类型。
         * @param values 元素缓冲。
         * @param scalar 运算标量，作为除数时必须非零。
         * @param count 元素个数。
         */
        template <class T>
        void operations<T>::scale(T* values, T scalar, std::size_t count)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                values[i] *= scalar;
            }
        }

        /**
         * @brief 原位执行非零标量除法。
         *
         * @tparam T 浮点元素类型。
         * @param values 元素缓冲。
         * @param scalar 运算标量，作为除数时必须非零。
         * @param count 元素个数。
         */
        template <class T>
        void operations<T>::divide(T* values, T scalar, std::size_t count)
        {
            assert(scalar != T(0));
            for (std::size_t i = 0; i < count; ++i)
            {
                values[i] /= scalar;
            }
        }

        /**
         * @brief 计算矩阵乘积并累加到结果缓冲，调用者须将结果初始化为零。
         *
         * @tparam T 浮点元素类型。
         * @param lhs 左侧输入输出缓冲。
         * @param rhs 右侧输入矩阵或缓冲。
         * @param result 输出缓冲，不得与输入重叠。
         * @param rows 输入矩阵行数。
         * @param inner 矩阵乘法的公共维度。
         * @param cols 输入矩阵列数。
         */
        template <class T>
        void operations<T>::multiply(const T* lhs, const T* rhs, T* result, std::size_t rows, std::size_t inner,
                                     std::size_t cols)
        {
            for (std::size_t col = 0; col < cols; ++col)
            {
                for (std::size_t k = 0; k < inner; ++k)
                {
                    for (std::size_t row = 0; row < rows; ++row)
                    {
                        result[row + col * rows] += lhs[row + k * rows] * rhs[k + col * inner];
                    }
                }
            }
        }

        /**
         * @brief 将列主序矩阵转置到独立结果缓冲。
         *
         * @tparam T 浮点元素类型。
         * @param values 元素缓冲。
         * @param result 输出缓冲，不得与输入重叠。
         * @param rows 输入矩阵行数。
         * @param cols 输入矩阵列数。
         */
        template <class T>
        void operations<T>::transpose(const T* values, T* result, std::size_t rows, std::size_t cols)
        {
            for (std::size_t col = 0; col < cols; ++col)
            {
                for (std::size_t row = 0; row < rows; ++row)
                {
                    result[col + row * cols] = values[row + col * rows];
                }
            }
        }

        /**
         * @brief 将主对角线置为一，调用者须将其他元素初始化为零。
         *
         * @tparam T 浮点元素类型。
         * @param values 元素缓冲。
         * @param size 方阵阶数。
         */
        template <class T>
        void operations<T>::identity(T* values, std::size_t size)
        {
            for (std::size_t i = 0; i < size; ++i)
            {
                values[i + i * size] = T(1);
            }
        }

        template struct operations<float>;

        template struct operations<double>;

        template struct operations<long double>;
    }
}
