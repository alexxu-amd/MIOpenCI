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

#include <miopen/graphapi/opgraph.h>

namespace miopen {
namespace graphapi {

OpNode::~OpNode() = default;

OpGraph OpGraphBuilder::build()
{

    OpGraph graph;

    // key = tensor ptr, value = vec. of dest nodes
    std::unordered_map<Tensor*, EdgeInfo> e_map;

    for(OpNode* n : mNodes)
    {

        for(Tensor* i : n->getInTensors())
        {
            auto [iter, _ignore] =
                e_map.emplace(i, EdgeInfo{}); // add empty EdgeInfo if not present

            iter->second.mDests.emplace_back(n);
        }

        for(Tensor* o : n->getOutTensors())
        {
            auto [iter, _ignore] =
                e_map.emplace(o, EdgeInfo{}); // add empty EdgeInfo if not present

            assert(iter->second.mSrc == nullptr);
            iter->second.mSrc = n;
        }
    }

    graph.addNodes(std::move(mNodes));

    for(const auto& [tens_ptr, edge_info] : e_map)
    {

        if(edge_info.mSrc != nullptr && !edge_info.mDests.empty())
        {
            for(const auto& d : edge_info.mDests)
            {
                graph.addEdge(edge_info.mSrc, tens_ptr, d);
            }
        }
        else if(edge_info.mSrc == nullptr)
        {

            // tens_ptr is a non-virtual input tensor
            assert(!tens_ptr->isVirtual()); // tensor pointer can't be virtual
            assert(!edge_info.mDests.empty());

            // NOTE(Amber): we may take out this step if we decide not to add a source
            // and sink in the graph
            for(const auto& d : edge_info.mDests)
            {
                graph.addEdgeFromSrc(d, tens_ptr);
            }
        }
        else if(edge_info.mDests.empty())
        {
            // tens_ptr is a non-virtual output tensor
            assert(!tens_ptr->isVirtual());
            assert(edge_info.mSrc != nullptr);
            graph.addEdgeToSink(edge_info.mSrc, tens_ptr);
        }
        else
        {
            // both can't be true at the same time
            assert(!(edge_info.mSrc == nullptr && edge_info.mDests.empty()));
        }
    }

    return graph;
}


namespace internal {

bool checkSameDegreeVecs(const OpGraph& left, const OpGraph& right) {
  auto l_degs = left.getInOutDegrees();
  auto r_degs = right.getInOutDegrees();

  /*
  auto sort_deg_vec = [] (auto& deg_vec) {

      std::sort(deg_vec.begin(), deg_vec.end(), 
          [] (const auto& left, const auto& right) {
            if (left.first == right.first) {
              return left.second < right.second;
            }
            return left.first < right.first;
          });

  };
  sort_deg_vec(l_degs);
  sort_deg_vec(r_degs);
  */
  std::sort(l_degs.begin(), l_degs.end());
  std::sort(r_degs.begin(), r_degs.end());
  return l_degs == r_degs;
}

auto groupBySize(VecOfPaths&& all_paths) {
    MapSizeToPathVec paths_by_size;

    for (auto& p: all_paths) {
      auto [it, _ignore]  = paths_by_size.emplace(p.size(), VecOfPaths{});
      it->second.emplace_back(std::move(p));
    }

    return paths_by_size;
}

auto sumPathSizes(const VecOfPaths& all_paths) {
    size_t ret = 0;
    for (const auto& p: all_paths) {
      ret += p.size();
    }
    return ret;
}

bool checkEqualPaths(VecOfPaths& left, VecOfPaths& right) { 
    if (left.size() != right.size()) {
      return false;
    }

    std::sort(left.begin(), left.end());
    std::sort(right.begin(), right.end());

    return left == right;
}


bool checkSamePaths(const OpGraph& left, const OpGraph& right) {


  MapSizeToPathVec l_paths_by_sz{};
  auto r_paths_by_sz = l_paths_by_sz;

  {
    auto l_paths = left.getAllPaths();
    auto r_paths = right.getAllPaths();

    if (l_paths.size() != r_paths.size()) {
      return false;
    }

    if (SumPathSizes(l_paths) != SumPathSizes(r_paths)) {
      return false;
    }

    l_paths_by_sz = groupBySize(std::move(l_paths));
    r_paths_by_sz = groupBySize(std::move(r_paths));
  }

  auto get_keys = [] (const MapSizeToPathVec& paths_by_size) {
    std::vector<size_t> keys{};
    for (const auto& [k, v]: paths_by_size) {
      keys.emplace_back(k);
    }
    return keys;
  };

  auto l_keys = get_keys(l_paths_by_sz);
  auto r_keys = get_keys(r_paths_by_sz);

  if (l_keys != r_keys) {
    return false;
  }

  for (size_t k: l_keys) {
    if (!checkEqualPaths(l_paths_by_sz[k], r_paths_by_sz[k])) {
      return false;
    }
  }

  return true;

}

} // end namespace internal


inline bool isIsomorphic(const OpGraph& left, const OpGraph& right) {
  if (left.numNodes() != right.numNodes()) {
    return false;
  }

  if (left.numEdges() != right.numEdges()) {
    return false;
  }

  if (!internal::checkSameDegreeVecs(left, right)) {
    return false;
  }

  if (!internal::checkSamePaths(left, right)) {
    return false;
  }

  return true;
}


} // end namespace graphapi
} // end namespace miopen
