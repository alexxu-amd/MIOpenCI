/*******************************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2023 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *******************************************************************************/
#include <miopen/miopen.h>
#ifdef MIOPEN_BETA_API
#include <gtest/gtest.h>
#include <miopen/cat.hpp>

#include "tensor_holder.hpp"
#include "cpu_cat.hpp"
#include "get_handle.hpp"
#include "../driver/tensor_driver.hpp"
#include "verify.hpp"
#include <random>

struct CatTestCase
{
    size_t dim;
    std::vector<std::vector<int>> inputs;
    friend std::ostream& operator<<(std::ostream& os, const CatTestCase& tc)
    {
        for(auto input : tc.inputs)
        {
            os << " input:" << input[0];
            for(int i = 1; i < input.size(); i++)
            {
                os << "," << input[i];
            }
        }
        return os << " dim:" << tc.dim;
    }

    const std::vector<std::vector<int>>& GetInputs() { return inputs; }
};

std::vector<CatTestCase> CatTestConfigs()
{ // dim, dims
    // clang-format off
    return {{1, {{2, 32, 128, 128, 128},{2, 32, 128, 128, 128}}},
            {1, {{16, 256, 32, 32}, {16, 256, 32, 32}, {16, 256, 32, 32}, {16, 256, 32, 32}}},
            {1, {{12277440, 1}, {12277440, 1}, {12277440, 1}, {12277440, 1}}},
            {2, {{3990480,1,1},{3990480,1,1},{3990480,1,1},{3990480,1,1}}},
            {1, {{64,1056,7,7},{64,48,7,7},{64,48,7,7},{64,48,7,7},{64,48,7,7},{64,48,7,7},{64,48,7,7},{64,48,7,7}}},
            {1, {{4,136800,91},{4,34200,91},{4,8550,91},{4,2223,91},{4,630,91}}},
            {1, {{6,182400,4},{6,45600,4},{6,11400,4},{6,2850,4},{6,741,4}}},
            {2, {{256,81,5776},{256,81,2166},{256,81,600},{256,81,150},{256,81,36},{256,81,4}}},
            {1, {{2,192,80,80},{2,192,80,80}}},
            {0, {{2,1024,768},{2,1024,768},{2,1024,768}}},
            {0, {{9600,384},{128,384}}},
            {0, {{192},{64}}},
          {0, {{8,1024},{8,1024},{8,1024},{8,1024},{8,1024},{8,1024},{8,1024},{8,1024}}}};
    // clang-format on
}

inline int32_t SetTensorLayout(miopen::TensorDescriptor& desc)
{
    std::vector<std::size_t> lens = desc.GetLengths();
    std::vector<int32_t> int32_t_lens(lens.begin(), lens.end());

    // set the strides for the tensor
    return SetTensorNd(&desc, int32_t_lens, desc.GetType());
}

template <typename T = float>
struct CatSolverTest : public ::testing::TestWithParam<CatTestCase>
{
protected:
    void SetUp() override
    {
        auto&& handle = get_handle();
        test_skipped  = false;
        cat_config    = GetParam();
        std::mt19937 gen(0);
        std::uniform_real_distribution<> d{-3, 3};
        auto gen_value = [&](auto...) { return d(gen); };

        dim = cat_config.dim;

        auto in_dims = cat_config.GetInputs();
        std::vector<size_t> out_dim{in_dims[0].begin(), in_dims[0].end()};
        out_dim[dim] = 0;

        for(auto in_dim : in_dims)
        {
            inputs.push_back(tensor<T>{in_dim}.generate(gen_value));
            SetTensorLayout(inputs.back().desc);
            out_dim[dim] += in_dim[dim];
        }

        output = tensor<T>{out_dim};
        SetTensorLayout(output.desc);
        std::fill(output.begin(), output.end(), std::numeric_limits<T>::quiet_NaN());

        ref_output = tensor<T>{out_dim};
        std::fill(ref_output.begin(), ref_output.end(), std::numeric_limits<T>::quiet_NaN());

        std::transform(inputs.begin(), inputs.end(), inputs_dev.begin(), [&](auto input) {
            return handle.Write(input.data);
        });
        output_dev = handle.Write(output.data);
    }
    void TearDown() override
    {
        if(test_skipped)
            return;

        auto&& handle = get_handle();

        cpu_cat_forward<T>(inputs, ref_output, dim);
        std::vector<miopen::TensorDescriptor> inputDescs;
        std::vector<ConstData_t> inputData;
        std::transform(inputs_dev.begin(), inputs_dev.end(), inputData.begin(), [](auto& input) {
            return input.get();
        });
        std::transform(inputs.begin(), inputs.end(), inputDescs.begin(), [](auto input) {
            return input.desc;
        });

        miopenStatus_t status =
            miopen::CatForward(handle, inputDescs, inputData, output.desc, output_dev.get(), dim);

        EXPECT_EQ(status, miopenStatusSuccess);

        output.data = handle.Read<T>(output_dev, output.data.size());

        double threshold = std::numeric_limits<T>::epsilon();
        auto error       = miopen::rms_range(ref_output, output);

        EXPECT_TRUE(miopen::range_distance(ref_output) == miopen::range_distance(output));
        EXPECT_TRUE(error < threshold * 1000) << "Error output beyond tolerance Error:" << error
                                              << ",  Thresholdx1000: " << threshold * 1000;

        EXPECT_TRUE(error < threshold * 2000) << "Error rstd beyond tolerance Error:" << error
                                              << ",  Thresholdx2000: " << threshold * 2000;
    }
    CatTestCase cat_config;

    std::vector<tensor<T>> inputs;
    tensor<T> output;
    tensor<T> ref_output;

    std::vector<miopen::Allocator::ManageDataPtr> inputs_dev;
    miopen::Allocator::ManageDataPtr output_dev;

    size_t dim;

    bool test_skipped = false;
};
#endif
