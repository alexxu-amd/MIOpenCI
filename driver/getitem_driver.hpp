/*******************************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
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
#ifndef GUARD_MIOPEN_GETITEM_DRIVER_HPP
#define GUARD_MIOPEN_GETITEM_DRIVER_HPP

#include "InputFlags.hpp"
#include "driver.hpp"
#include "tensor_driver.hpp"
#include "timer.hpp"
#include "random.hpp"
#include <algorithm>
#include <cfloat>
#include <cstdlib>
#include <memory>
#include <miopen/miopen.h>
#include <miopen/tensor.hpp>
#include <numeric>
#include <vector>
#include <../test/tensor_holder.hpp>
#include <../test/verify.hpp>

typedef struct
{
    size_t size[5];
    size_t stride[5];
} tensor_view_5d_t;

tensor_view_5d_t get_inner_expanded_tv(const miopenTensorDescriptor_t Desc)
{
    auto dims    = miopen::deref(Desc).GetLengths();
    auto strides = miopen::deref(Desc).GetStrides();

    tensor_view_5d_t tv_5d;
    for(size_t i = 0; i < strides.size(); ++i)
    {
        tv_5d.stride[i] = strides[i];
        tv_5d.size[i]   = dims[i];
    }
    auto rest = strides.size();
    for(size_t j = rest; j < 5; ++j)
    {
        tv_5d.stride[j] = (rest == 0 ? 1 : strides[rest - 1]);
        tv_5d.size[j]   = 1;
    }
    return tv_5d;
}

template <typename Tgpu, typename Tcheck>
int32_t mloGetitemBackwardRunHost(miopenTensorDescriptor_t dyDesc,
                                  miopenTensorDescriptor_t xDesc,
                                  std::vector<miopenTensorDescriptor_t> indexDescs,
                                  miopenTensorDescriptor_t yDesc,
                                  miopenTensorDescriptor_t dxDesc,
                                  miopenTensorDescriptor_t errorDesc,
                                  Tgpu* dy,
                                  Tgpu* x,
                                  Tgpu* y,
                                  std::vector<int32_t*> indexs,
                                  Tcheck* dxhost,
                                  Tcheck* errorhost,
                                  std::vector<int32_t> dims,
                                  std::vector<std::vector<int32_t>> slices,
                                  int32_t offset)
{
    auto dy_dims    = miopen::deref(dyDesc).GetLengths();
    auto dy_strides = miopen::deref(dyDesc).GetStrides();
    auto dy_numel = std::accumulate(dy_dims.begin(), dy_dims.end(), 1L, std::multiplies<int64_t>());
    auto dx_dims  = miopen::deref(dxDesc).GetLengths();
    auto dx_strides = miopen::deref(dxDesc).GetStrides();
    auto index_dims = miopen::deref(indexDescs[0]).GetLengths();
    auto index_numel =
        std::accumulate(index_dims.begin(), index_dims.end(), 1L, std::multiplies<int64_t>());
    auto indexs_len    = indexDescs.size();
    auto element_index = std::vector<int32_t>(indexs_len * index_numel);

    std::vector<int32_t> output_dims;
    for(auto dim : dims)
    {
        output_dims.push_back(dx_dims[dim]);
    }

    int32_t dim_info_offset = indexs_len * index_dims[0];
    auto start_dim          = dims[0];

    // Get element index form indexs
    for(int j = 0; j < indexs_len; j++)
    {
        auto dim_size = output_dims[j];

        for(size_t o = 0; o < index_numel; o++)
        {
            int32_t getitem_index = indexs[j][o];

            if(getitem_index >= 0 && getitem_index < dim_size)
            {
                element_index[(o * indexs_len) + j] = getitem_index;
            }
            else if(getitem_index >= -dim_size && getitem_index < 0)
            {
                element_index[(o * indexs_len) + j] = getitem_index + dim_size;
            }
            else
            {
                errorhost[j] = -1;
            }

            if(o == 0)
            {
                element_index[dim_info_offset + j] = dim_size;
            }
        }
    }

    // Apply slice to dx
    for(auto slice : slices)
    {
        int32_t dim   = slice[0];
        int32_t start = slice[1];
        int32_t end   = slice[2];
        int32_t step  = slice[3];

        if(end > static_cast<int32_t>(dx_dims[dim]))
            end = dx_dims[dim];

        auto len = end - start;

        dx_dims[dim] = (len + step - 1) / step;
        dx_strides[dim] *= step;
    }

    // GetItem
    for(size_t o = 0; o < dy_numel; o++)
    {
        tensor_view_5d_t tv_5d = get_inner_expanded_tv(dyDesc);
        size_t NCDHW[5], NCDHW2[5];
        size_t ncdh = (o) / tv_5d.size[4];
        NCDHW[4]    = (o) % tv_5d.size[4];
        size_t ncd  = ncdh / tv_5d.size[3];
        NCDHW[3]    = ncdh % tv_5d.size[3];
        size_t nc   = ncd / tv_5d.size[2];
        NCDHW[2]    = ncd % tv_5d.size[2];
        NCDHW[0]    = nc / tv_5d.size[1];
        NCDHW[1]    = nc % tv_5d.size[1];

        for(int i = 0; i < 5; i++)
        {
            NCDHW2[i] = NCDHW[i];
        }

        if(indexs_len > 0)
        {
            size_t dim_cursor = NCDHW[start_dim];
            size_t i          = start_dim;
            size_t j          = 0;

            for(; i < start_dim + indexs_len; ++i, ++j)
            {
                size_t dim_idx  = element_index[dim_info_offset + j];
                NCDHW2[dim_idx] = element_index[(dim_cursor * indexs_len) + j];
            }

            i          = element_index[dim_info_offset + indexs_len - 1] + 1;
            dim_cursor = start_dim + 1;
            for(; i < 5; ++i, ++dim_cursor)
            {
                NCDHW2[i] = NCDHW[dim_cursor];
            }
        }

        auto dy_idx = dy_strides[4] * (NCDHW2[4]) + dy_strides[3] * (NCDHW2[3]) +
                      dy_strides[2] * (NCDHW2[2]) + dy_strides[1] * (NCDHW2[1]) +
                      dy_strides[0] * (NCDHW2[0]);
        auto dx_idx = dx_strides[4] * (NCDHW[4]) + dx_strides[3] * (NCDHW[3]) +
                      dx_strides[2] * (NCDHW[2]) + dx_strides[1] * (NCDHW[1]) +
                      dx_strides[0] * (NCDHW[0]);

        dxhost[dx_idx] += dy[dy_idx];
    }
}

template <typename Tgpu, typename Tref>
class GetitemDriver : public Driver
{
public:
    GetitemDriver() : Driver()
    {
        miopenCreateTensorDescriptor(&dyDesc);
        miopenCreateTensorDescriptor(&xDesc);
        miopenCreateTensorDescriptor(&yDesc);
        miopenCreateTensorDescriptor(&dxDesc);
        miopenCreateTensorDescriptor(&errorDesc);

        data_type = miopen_type<Tgpu>{};
    }

    int AddCmdLineArgs() override;
    int ParseCmdLineArgs(int argc, char* argv[]) override;
    InputFlags& GetInputFlags() override { return inflags; }

    int GetandSetData() override;
    std::vector<int> GetInputTensorLengthsFromCmdLine();

    int AllocateBuffersAndCopy() override;

    int RunForwardGPU() override;

    int RunBackwardGPU() override;
    int RunBackwardCPU();

    Tref GetTolerance();

    int VerifyBackward() override;
    int VerifyForward() override;
    ~GetitemDriver() override
    {
        miopenDestroyTensorDescriptor(dyDesc);
        miopenDestroyTensorDescriptor(xDesc);
        miopenDestroyTensorDescriptor(yDesc);
        for(auto indexDesc : indexDescs)
        {
            miopenDestroyTensorDescriptor(indexDesc);
        }
        miopenDestroyTensorDescriptor(dxDesc);
        miopenDestroyTensorDescriptor(errorDesc);
    }

private:
    InputFlags inflags;

    int forw;

    miopenTensorDescriptor_t dyDesc;
    miopenTensorDescriptor_t xDesc;
    miopenTensorDescriptor_t yDesc;
    std::vector<miopenTensorDescriptor_t> indexDescs;
    miopenTensorDescriptor_t dxDesc;
    miopenTensorDescriptor_t errorDesc;

    std::unique_ptr<GPUMem> dy_dev;
    std::unique_ptr<GPUMem> x_dev;
    std::unique_ptr<GPUMem> y_dev;
    std::vector<std::unique_ptr<GPUMem>> index_devs;
    std::unique_ptr<GPUMem> dx_dev;
    std::unique_ptr<GPUMem> error_dev;
    std::unique_ptr<GPUMem> workspace_dev;

    std::vector<Tgpu> dy;
    std::vector<Tgpu> x;
    std::vector<Tgpu> y;
    std::vector<std::vector<int32_t>> indexs;
    std::vector<Tgpu> dx;
    std::vector<Tgpu> error;
    std::vector<Tref> dxhost;
    std::vector<Tref> errorhost;

    size_t ws_sizeInBytes;

    std::vector<int32_t> dims;
    std::vector<std::vector<int32_t>> slices;
    std::vector<int32_t> slices_flat;
    int32_t offset;

    std::vector<int32_t> output_dims;
    std::vector<void*> index_devs_ptr;
    std::vector<int32_t*> indexs_ptr;
};

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::ParseCmdLineArgs(int argc, char* argv[])
{
    inflags.Parse(argc, argv);

    if(inflags.GetValueInt("time") == 1)
    {
        miopenEnableProfiling(GetHandle(), true);
    }
    return miopenStatusSuccess;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::GetandSetData()
{
    auto dyTensorParam   = inflags.GetValueTensor("doutput");
    auto xTensorParam    = inflags.GetValueTensor("input");
    auto yTensorParam    = inflags.GetValueTensor("output");
    auto dxTensorParam   = inflags.GetValueTensor("dinput");
    auto indexCountParam = inflags.GetValueInt("indexcount");
    auto dimCountParam   = inflags.GetValueInt("dimcount");
    auto sliceCountParam = inflags.GetValueInt("slicecount");

    auto indexTensorLengths = inflags.GetValue2dVectorInt("indexs");
    if(indexTensorLengths.size() != indexCountParam)
        MIOPEN_THROW("Error parsing indexs tensor: " + inflags.GetValueStr("indexs") + ".");

    dims = inflags.GetValueVectorInt("dims");
    if(dims.size() != dimCountParam)
        MIOPEN_THROW("Error parsing dims tensor: " + inflags.GetValueStr("dims") + ".");

    for(auto dim : dims)
    {
        output_dims.push_back(dxTensorParam.lengths[dim]);
    }

    slices = inflags.GetValue2dVectorInt("slices");
    if(slices.size() != sliceCountParam)
        MIOPEN_THROW("Error parsing slices: " + inflags.GetValueStr("slices") + ".");

    for(auto slice : slices)
    {
        for(int32_t i = 0; i < 4; i++)
        {
            slices_flat.push_back(slice[i]);
        }
    }

    if(SetTensorNd(dyDesc, dyTensorParam.lengths, data_type) != miopenStatusSuccess)
        MIOPEN_THROW("Error parsing doutput tensor: " + inflags.GetValueStr("doutput") + ".");

    if(SetTensorNd(xDesc, xTensorParam.lengths, data_type) != miopenStatusSuccess)
        MIOPEN_THROW("Error parsing input tensor: " + inflags.GetValueStr("input") + ".");

    if(SetTensorNd(yDesc, yTensorParam.lengths, data_type) != miopenStatusSuccess)
        MIOPEN_THROW("Error parsing output tensor: " + inflags.GetValueStr("output") + ".");

    for(auto indexTensorLength : indexTensorLengths)
    {
        miopenTensorDescriptor_t indexDesc;
        miopenCreateTensorDescriptor(&indexDesc);
        if(SetTensorNd(indexDesc, indexTensorLength, miopenInt32) != miopenStatusSuccess)
            MIOPEN_THROW("Error parsing indexs tensor: " + inflags.GetValueStr("indexs") + ".");
        indexDescs.push_back(indexDesc);
    }

    if(SetTensorNd(dxDesc, dxTensorParam.lengths, data_type) != miopenStatusSuccess)
        MIOPEN_THROW("Error parsing dinput tensor: " + inflags.GetValueStr("dinput") + ".");

    std::vector<int32_t> error_length;
    error_length.push_back(indexCountParam);
    if(SetTensorNd(errorDesc, error_length, data_type) != miopenStatusSuccess)
        MIOPEN_THROW("Error making error tensor: " + inflags.GetValueStr("indexcount") + ".");

    return 0;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::AddCmdLineArgs()
{
    inflags.AddInputFlag("forw", 'F', "1", "Run only Forward Getitem (Default=1)", "int");
    inflags.AddTensorFlag("doutput", 'O', "100x3x32x32", "doutput tensor descriptor");
    inflags.AddTensorFlag("input", 'X', "100x3x32x32", "input tensor descriptor");
    inflags.AddTensorFlag("output", 'Y', "100x3x32x32", "output tensor descriptor");
    inflags.AddTensorFlag("indexs", 'D', "100x3x32x32", "indexs tensor descriptor");
    inflags.AddTensorFlag("dinput", 'N', "100x3x32x32", "dinput tensor descriptor");

    inflags.AddInputFlag("dimcount", '1', "1", "The dimensions(Default=1)", "int");
    inflags.AddInputFlag("dims", '2', "0", "The dimensions(Default=0)", "vector<int>");
    inflags.AddInputFlag("slicecount", '3', "0", "The number of slices(Default=0)", "int");
    inflags.AddInputFlag("slices",
                         '4',
                         "",
                         "The slices(Default=\'\'"
                         ")",
                         "vector<vector<int>>");
    inflags.AddInputFlag("offset", '5', "0", "The offset of output(Default=0)", "int");

    inflags.AddInputFlag("iter", 'i', "10", "Number of Iterations (Default=10)", "int");
    inflags.AddInputFlag("verify", 'V', "1", "Verify Each Layer (Default=1)", "int");
    inflags.AddInputFlag("time", 't', "0", "Time Each Layer (Default=0)", "int");
    inflags.AddInputFlag(
        "wall", 'w', "0", "Wall-clock Time Each Layer, Requires time == 1 (Default=0)", "int");

    return miopenStatusSuccess;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::AllocateBuffersAndCopy()
{
    size_t dy_sz    = GetTensorSize(dyDesc);
    size_t x_sz     = GetTensorSize(xDesc);
    size_t y_sz     = GetTensorSize(yDesc);
    size_t dx_sz    = GetTensorSize(dxDesc);
    size_t error_sz = GetTensorSize(errorDesc);

    miopenGetGetItemWorkspaceSize(
        GetHandle(), indexDescs.size(), indexDescs.data(), &ws_sizeInBytes);
    if(ws_sizeInBytes == static_cast<size_t>(-1))
        return miopenStatusAllocFailed;

    uint32_t ctx = 0;

    dy_dev        = std::unique_ptr<GPUMem>(new GPUMem(ctx, dy_sz, sizeof(Tgpu)));
    x_dev         = std::unique_ptr<GPUMem>(new GPUMem(ctx, x_sz, sizeof(Tgpu)));
    y_dev         = std::unique_ptr<GPUMem>(new GPUMem(ctx, y_sz, sizeof(Tgpu)));
    dx_dev        = std::unique_ptr<GPUMem>(new GPUMem(ctx, dx_sz, sizeof(Tgpu)));
    error_dev     = std::unique_ptr<GPUMem>(new GPUMem(ctx, error_sz, sizeof(Tgpu)));
    workspace_dev = std::unique_ptr<GPUMem>(new GPUMem(ctx, ws_sizeInBytes, sizeof(std::byte)));

    dy        = std::vector<Tgpu>(dy_sz, static_cast<Tgpu>(0));
    x         = std::vector<Tgpu>(x_sz, static_cast<Tgpu>(0));
    y         = std::vector<Tgpu>(y_sz, static_cast<Tgpu>(0));
    dx        = std::vector<Tgpu>(dx_sz, static_cast<Tgpu>(0));
    error     = std::vector<Tgpu>(error_sz, static_cast<Tgpu>(0));
    dxhost    = std::vector<Tref>(dx_sz, static_cast<Tref>(0));
    errorhost = std::vector<Tref>(error_sz, static_cast<Tref>(0));

    for(int32_t i = 0; i < dy_sz; i++)
    {
        dy[i] = prng::gen_A_to_B<Tgpu>(static_cast<Tgpu>(-1.0), static_cast<Tgpu>(1.0));
    }

    for(int32_t i = 0; i < x_sz; i++)
    {
        x[i] = prng::gen_A_to_B<Tgpu>(static_cast<Tgpu>(-1.0), static_cast<Tgpu>(1.0));
    }

    for(int32_t i = 0; i < y_sz; i++)
    {
        y[i] = prng::gen_A_to_B<Tgpu>(static_cast<Tgpu>(-1.0), static_cast<Tgpu>(1.0));
    }

    for(int32_t i = 0; i < indexDescs.size(); i++)
    {
        size_t index_sz = GetTensorSize(indexDescs[i]);
        index_devs.push_back(std::unique_ptr<GPUMem>(new GPUMem(ctx, index_sz, sizeof(int32_t))));
        indexs.push_back(std::vector<int32_t>(index_sz, static_cast<int32_t>(0)));
        auto& index    = indexs.back();
        auto index_dev = index_devs.back().get();

        index[i] = prng::gen_A_to_B<int32_t>(static_cast<int32_t>(0),
                                             static_cast<int32_t>(output_dims[i]));

        if(index_dev->ToGPU(GetStream(), index.data()) != 0)
            std::cerr << "Error copying (index) to GPU, size: " << index_dev->GetSize()
                      << std::endl;
        index_devs_ptr.push_back(index_dev->GetMem());
        indexs_ptr.push_back(index.data());
    }

    if(dy_dev->ToGPU(GetStream(), dy.data()) != 0)
        std::cerr << "Error copying (dy) to GPU, size: " << dy_dev->GetSize() << std::endl;

    if(x_dev->ToGPU(GetStream(), x.data()) != 0)
        std::cerr << "Error copying (x) to GPU, size: " << x_dev->GetSize() << std::endl;

    if(y_dev->ToGPU(GetStream(), y.data()) != 0)
        std::cerr << "Error copying (y) to GPU, size: " << y_dev->GetSize() << std::endl;

    return miopenStatusSuccess;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::RunForwardGPU()
{
    return miopenStatusSuccess;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::RunBackwardGPU()
{
    float kernel_total_time = 0;
    float kernel_first_time = 0;

    Timer t;
    START_TIME

    for(int32_t i = 0; i < inflags.GetValueInt("iter"); i++)
    {
        miopenGetitemBackward(GetHandle(),
                              workspace_dev->GetMem(),
                              ws_sizeInBytes,
                              dyDesc,
                              dy_dev->GetMem(),
                              xDesc,
                              x_dev->GetMem(),
                              indexDescs.size(),
                              indexDescs.data(),
                              index_devs_ptr.data(),
                              yDesc,
                              y_dev->GetMem(),
                              dxDesc,
                              dx_dev->GetMem(),
                              errorDesc,
                              error_dev->GetMem(),
                              dims.size(),
                              dims.data(),
                              slices.size(),
                              slices_flat.data(),
                              offset);

        float time = 0;
        miopenGetKernelTime(GetHandle(), &time);
        kernel_total_time += time;
        if(i == 0)
            kernel_first_time = time;
    }

    if(inflags.GetValueInt("time") == 1)
    {
        STOP_TIME
        int32_t iter = inflags.GetValueInt("iter");
        if(WALL_CLOCK)
            std::cout << "Wall-clock Time Forward Getitem Elapsed: " << t.gettime_ms() / iter
                      << " ms\n";

        float kernel_average_time =
            iter > 1 ? (kernel_total_time - kernel_first_time) / (iter - 1) : kernel_first_time;
        std::cout << "GPU Kernel Time Forward Getitem Elapsed: " << kernel_average_time << " ms\n";
    }

    if(dx_dev->FromGPU(GetStream(), dx.data()) != 0)
        std::cerr << "Error copying (dx_dev) from GPU, size: " << dx_dev->GetSize() << std::endl;

    if(error_dev->FromGPU(GetStream(), error.data()) != 0)
        std::cerr << "Error copying (error_dev) from GPU, size: " << error_dev->GetSize()
                  << std::endl;

    return miopenStatusSuccess;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::RunBackwardCPU()
{
    mloGetitemBackwardRunHost<Tgpu, Tref>(dyDesc,
                                          xDesc,
                                          indexDescs,
                                          yDesc,
                                          dxDesc,
                                          errorDesc,
                                          dy.data(),
                                          x.data(),
                                          y.data(),
                                          indexs_ptr,
                                          dxhost.data(),
                                          errorhost.data(),
                                          dims,
                                          slices,
                                          offset,
                                          output_dims);

    return miopenStatusSuccess;
}

template <typename Tgpu, typename Tref>
Tref GetitemDriver<Tgpu, Tref>::GetTolerance()
{
    // Computation error of fp16 is ~2^13 (=8192) bigger than
    // the one of fp32 because mantissa is shorter by 13 bits.
    auto tolerance = std::is_same<Tgpu, float>::value ? 1.5e-6 : 8.2e-3;

    // bf16 mantissa has 7 bits, by 3 bits shorter than fp16.
    if(std::is_same<Tgpu, bfloat16>::value)
        tolerance *= 8.0;
    return tolerance;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::VerifyForward()
{
    return miopenStatusSuccess;
}

template <typename Tgpu, typename Tref>
int GetitemDriver<Tgpu, Tref>::VerifyBackward()
{
    RunBackwardCPU();
    const Tref tolerance = GetTolerance();

    auto error_dx = miopen::rms_range(dxhost, dx);

    if(!std::isfinite(error_dx) || error_dx > tolerance)
    {
        std::cout << "Backward Getitem FAILED: " << error_dx << " > " << tolerance << std::endl;
        return EC_VerifyBwd;
    }
    else
    {
        std::cout << "Backward Getitem Verifies OK on CPU reference (" << error_dx << " < "
                  << tolerance << ')' << std::endl;
    }

    auto error_error = miopen::rms_range(errorhost, error);

    if(!std::isfinite(error_error) || std::abs(static_cast<float>(error_error)) != 0.0f)
    {
        std::cout << "Backward Getitem FAILED: Result does not equal" << std::endl;
        return EC_VerifyBwd;
    }
    else
    {
        std::cout << "Backward Getitem Verifies OK on CPU and GPU (err=" << error << ")\n";
    }

    return miopenStatusSuccess;
}

#endif // GUARD_MIOPEN_GETITEM_DRIVER_HPP
