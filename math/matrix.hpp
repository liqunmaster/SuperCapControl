/**
 * @file matrix.hpp
 * @author LiQun (liqunyan@gmail.com)
 * @brief 固定尺寸列主序矩阵接口。
 * @version 0.1
 * @date 2026-11-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cassert>
#include <cstddef>
#include <type_traits>

namespace matrix
{
    namespace detail
    {
        /**
         * @brief 列主序连续缓冲的基础矩阵运算。
         * @tparam T 浮点元素类型。

         */
        template <class T>
        struct operations
        {
            public:
                static void add(T* lhs, const T* rhs, std::size_t count);

                static void subtract(T* lhs, const T* rhs, std::size_t count);

                static void scale(T* values, T scalar, std::size_t count);

                static void divide(T* values, T scalar, std::size_t count);

                static void multiply(const T* lhs, const T* rhs, T* result, std::size_t rows, std::size_t inner,
                                     std::size_t cols);

                static void transpose(const T* values, T* result, std::size_t rows, std::size_t cols);

                static void identity(T* values, std::size_t size);
        };
    }

    /**
     * @brief 固定尺寸列主序矩阵，元素默认初始化为零。
     * @tparam Rows 行数，须大于零。
     * @tparam Cols
     * 列数，须大于零。
     * @tparam T 浮点元素类型。
     */
    template <std::size_t Rows, std::size_t Cols, class T = float>
    class fixed_matrix
    {
        private:
            static_assert(Rows > 0 && Cols > 0, "");

            static_assert(std::is_floating_point<T>::value, "");

        public:
            fixed_matrix() = default;

            /**
             * @brief 从二维数组复制构造矩阵。
             * @param values
             * 按行列索引给定的初始元素。
             *
             */
            fixed_matrix(const T (&values)[Rows][Cols])
            {
                for (std::size_t row = 0; row < Rows; ++row)
                {
                    for (std::size_t col = 0; col < Cols; ++col)
                    {
                        (*this)(row, col) = values[row][col];
                    }
                }
            }

            /**
             * @brief 返回矩阵行数。
             *
             * @return constexpr std::size_t，返回对应运算结果。
             */
            constexpr std::size_t rows() const
            {
                return Rows;
            }

            /**
             * @brief 返回矩阵列数。
             *
             * @return constexpr std::size_t，返回对应运算结果。
             */
            constexpr std::size_t cols() const
            {
                return Cols;
            }

            /**
             * @brief 访问可写矩阵元素，调试构建检查边界。
             *
             * @param row 从零开始的行索引。
             * @param col 从零开始的列索引。
             * @return T&，返回对应运算结果。
             */
            T& operator()(std::size_t row, std::size_t col)
            {
                assert(row < Rows && col < Cols);
                return values_[row + col * Rows];
            }

            /**
             * @brief 访问只读矩阵元素，调试构建检查边界。
             *
             * @param row 从零开始的行索引。
             * @param col 从零开始的列索引。
             * @return const T&，返回对应运算结果。
             */
            const T& operator()(std::size_t row, std::size_t col) const
            {
                assert(row < Rows && col < Cols);
                return values_[row + col * Rows];
            }

            /**
             * @brief 逐元素原位相加。
             *
             * @param rhs 右侧输入矩阵或缓冲。
             * @return fixed_matrix&，返回对应运算结果。
             */
            fixed_matrix& operator+=(const fixed_matrix& rhs)
            {
                detail::operations<T>::add(values_, rhs.values_, Rows * Cols);
                return *this;
            }

            /**
             * @brief 逐元素原位相减。
             *
             * @param rhs 右侧输入矩阵或缓冲。
             * @return fixed_matrix&，返回对应运算结果。
             */
            fixed_matrix& operator-=(const fixed_matrix& rhs)
            {
                detail::operations<T>::subtract(values_, rhs.values_, Rows * Cols);
                return *this;
            }

            /**
             * @brief 逐元素原位乘以标量。
             *
             * @param scalar 运算标量，作为除数时必须非零。
             * @return fixed_matrix&，返回对应运算结果。
             */
            fixed_matrix& operator*=(T scalar)
            {
                detail::operations<T>::scale(values_, scalar, Rows * Cols);
                return *this;
            }

            /**
             * @brief 逐元素原位除以非零标量。
             *
             * @param scalar 运算标量，作为除数时必须非零。
             * @return fixed_matrix&，返回对应运算结果。
             */
            fixed_matrix& operator/=(T scalar)
            {
                detail::operations<T>::divide(values_, scalar, Rows * Cols);
                return *this;
            }

            /**
             * @brief 返回逐元素相加结果。
             *
             * @param rhs 右侧输入矩阵或缓冲。
             * @return fixed_matrix，返回对应运算结果。
             */
            fixed_matrix operator+(const fixed_matrix& rhs) const
            {
                fixed_matrix result = *this;
                result += rhs;
                return result;
            }

            /**
             * @brief 返回逐元素相减结果。
             *
             * @param rhs 右侧输入矩阵或缓冲。
             * @return fixed_matrix，返回对应运算结果。
             */
            fixed_matrix operator-(const fixed_matrix& rhs) const
            {
                fixed_matrix result = *this;
                result -= rhs;
                return result;
            }

            /**
             * @brief 返回矩阵与标量的乘积。
             *
             * @param scalar 运算标量，作为除数时必须非零。
             * @return fixed_matrix，返回对应运算结果。
             */
            fixed_matrix operator*(T scalar) const
            {
                fixed_matrix result = *this;
                result *= scalar;
                return result;
            }

            /**
             * @brief 返回矩阵除以非零标量的结果。
             *
             * @param scalar 运算标量，作为除数时必须非零。
             * @return fixed_matrix，返回对应运算结果。
             */
            fixed_matrix operator/(T scalar) const
            {
                fixed_matrix result = *this;
                result /= scalar;
                return result;
            }

            /**
             * @brief 返回符合维度约束的矩阵乘积。
             *
             * @tparam OtherCols 右侧矩阵列数。
             * @param rhs 右侧输入矩阵或缓冲。
             * @return fixed_matrix<Rows, OtherCols, T>，返回对应运算结果。
             */
            template <std::size_t OtherCols>
            fixed_matrix<Rows, OtherCols, T> operator*(const fixed_matrix<Cols, OtherCols, T>& rhs) const
            {
                fixed_matrix<Rows, OtherCols, T> result;
                detail::operations<T>::multiply(values_, &rhs(0, 0), &result(0, 0), Rows, Cols, OtherCols);
                return result;
            }

            /**
             * @brief 返回标量左乘矩阵的结果。
             *
             * @param scalar 运算标量，作为除数时必须非零。
             * @param matrix 输入矩阵。
             * @return fixed_matrix，返回对应运算结果。
             */
            friend fixed_matrix operator*(T scalar, fixed_matrix matrix)
            {
                matrix *= scalar;
                return matrix;
            }

        private:
            T values_[Rows * Cols]{};
    };
    /**
     * @brief 返回行列互换后的矩阵。
     *
     * @param matrix 输入矩阵。
     * @return fixed_matrix<Cols, Rows, T>，返回对应运算结果。
     */

    template <std::size_t Rows, std::size_t Cols, class T>
    fixed_matrix<Cols, Rows, T> transpose(const fixed_matrix<Rows, Cols, T>& matrix)
    {
        fixed_matrix<Cols, Rows, T> result;
        detail::operations<T>::transpose(&matrix(0, 0), &result(0, 0), Rows, Cols);
        return result;
    }

    /**
     * @brief 返回指定尺寸的单位矩阵。
     *
     * @tparam Size 单位矩阵阶数。
     * @tparam T 浮点元素类型。
     * @return fixed_matrix<Size, Size, T>，返回对应运算结果。
     */
    template <std::size_t Size, class T = float>
    fixed_matrix<Size, Size, T> eye()
    {
        fixed_matrix<Size, Size, T> result;
        detail::operations<T>::identity(&result(0, 0), Size);
        return result;
    }
}
