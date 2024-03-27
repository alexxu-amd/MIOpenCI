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
#pragma once

#include <miopen/graphapi/tensor.hpp>

#include <algorithm>
<<<<<<< HEAD
||||||| merged common ancestors
#include <vector>
=======
#include <deque>
#include <vector>
>>>>>>> WIP: implement matching tests for op graphs
#include <unordered_map>
#include <vector>

#include <cassert>

namespace miopen {
namespace graphapi {

namespace internal {
template <typename C>
bool contains(const C& container, const typename C::value_type& val) noexcept
{
    return std::find(container.cbegin(), container.cend(), val) != container.cend();
}
} // end namespace internal

class OpGraphBuilder;
class OpGraph;

class OpNode
{
public:
    using Edge = std::pair<OpNode*, Tensor*>;
    virtual ~OpNode();

private:
    std::vector<Edge> mInEdges;
    std::vector<Edge> mOutEdges;

    friend class OpGraphBuilder;
    friend class OpGraph;

protected:
    virtual const std::string& signName() const = 0;

    virtual std::vector<Tensor*> getInTensors() const = 0;

    virtual std::vector<Tensor*> getOutTensors() const = 0;

    const auto& iterateInEdges() const { return mInEdges; }

    const auto& iterateOutEdges() const { return mOutEdges; }

    bool hasInEdge(OpNode* src, Tensor* tens_ptr) const
    {
        Edge e{src, tens_ptr};
        return internal::contains(mInEdges, e);
    }

    bool hasOutEdge(OpNode* dst, Tensor* tens_ptr) const
    {
        Edge e{dst, tens_ptr};
        return internal::contains(mOutEdges, e);
    }

    void addOutEdge(OpNode* dst, Tensor* tens_ptr)
    {
        assert(dst);
        if(!hasOutEdge(dst, tens_ptr))
        {
            mOutEdges.emplace_back(dst, tens_ptr);
        }
    }

    void addInEdge(OpNode* src, Tensor* tens_ptr)
    {
        assert(src);
        if(!hasInEdge(src, tens_ptr))
        {
            mInEdges.emplace_back(src, tens_ptr);
        }
    }

    size_t getInDegree() const { return mInEdges.size(); }
    size_t getOutDegree() const { return mOutEdges.size(); }
};

using OpEdge = OpNode::Edge;

class SourceOpNode : public OpNode
{
protected:
    std::vector<Tensor*> mOutTensors;
    friend class OpGraph;

    const std::string& signName() const final
    {
        static const std::string s = "INTERNAL::SRC";
        return s;
    }

    std::vector<Tensor*> getInTensors() const final { return {}; }

    std::vector<Tensor*> getOutTensors() const final { return mOutTensors; }

    bool hasOutTensor(Tensor* tensor) const { return internal::contains(mOutTensors, tensor); }

    void addOutTensor(Tensor* tens_ptr)
    {
        assert(!hasOutTensor(tens_ptr));
        mOutTensors.emplace_back(tens_ptr);
    }
};

class SinkOpNode : public OpNode
{
protected:
    std::vector<Tensor*> mInTensors;
    friend class OpGraph;

    const std::string& signName() const final
    {
        static const std::string s = "INTERNAL::SINK";
        return s;
    }

    std::vector<Tensor*> getInTensors() const final { return mInTensors; }

    std::vector<Tensor*> getOutTensors() const final { return {}; }

    bool hasInTensor(Tensor* tensor) const { return internal::contains(mInTensors, tensor); }

    void addInTensor(Tensor* tens_ptr)
    {
        assert(!hasInTensor(tens_ptr));
        mInTensors.emplace_back(tens_ptr);
    }
};

using Path = std::vector<OpNode*>;
using VecOfPaths = std::vector<Path>;

class OpGraph
{
    SourceOpNode mSrcNode{};
    SinkOpNode mSinkNode{};
    std::vector<OpNode*> mNodes{};

public:
    bool hasNode(OpNode* n) const { return internal::contains(mNodes, n); }

    bool hasEdge(OpNode* src, Tensor* tens_ptr, OpNode* dst) const
    {
        assert(src);
        assert(dst);
        return src->hasOutEdge(dst, tens_ptr) && dst->hasInEdge(src, tens_ptr);
    }

    size_t numNodes() const { 
      return mNodes.size();
    }

    size_t numEdges() const { 
      size_t ret = 0;
      for (OpNode* n: mNodes) {
        ret += n->getOutDegree();
      }
      // ignore the edges that lead to mSinkNode
      assert(ret >= mSinkNode.getInDegree());
      ret -= mSinkNode.getInDegree();

      return ret;
    }

    std::vector<std::pair<size_t, size_t>> getInOutDegrees() const { 
      std::vector<std::pair<size_t, size_t>> ret;
      for (OpNode* n: mNodes) {
        ret.emplace_back(n->getInDegree(), n->getOutDegree());
      }
      return ret;
    }

    VecOfPaths getAllPaths() const {
      // TODO(Amber): does not check for cycles. Use DFS to first check for cycles
      // at construction time perhaps. 
      VecOfPaths all_paths;

      std::deque<Path> paths_to_explore = {mSrcNode};

      while (!paths_to_explore.empty()) {
        Path path = paths_to_explore.front();
        paths_to_explore.pop_front();

        OpNode* last_node = path.back();
        if (last_node->iterateOutEdges().empty()) {
          // all paths should terminate at the sink
          assert(last_node == &mSinkNode);
          all_paths.emplace_back(std::move(path));
        } else {
          for (auto& [dst, tens_ptr]: last_node->iterateOutEdges()) {
            Path newPath{path};
            newPath.emplace_back(dst);
            paths_to_explore.emplace_back(std::move(newPath));
          }
        }
      } // end while

      return all_paths;
    }

private:
    friend class OpGraphBuilder;

    void addNodes(std::vector<OpNode*>&& nodes) { mNodes = std::move(nodes); }

    void addEdge(OpNode* src, Tensor* tens_ptr, OpNode* dst)
    {
        assert(src);
        assert(dst);
        src->addOutEdge(dst, tens_ptr);
        dst->addInEdge(src, tens_ptr);
    }

    void addEdgeFromSrc(OpNode* dst, Tensor* tens_ptr)
    {
        mSrcNode.addOutTensor(tens_ptr);
        addEdge(&mSrcNode, tens_ptr, dst);
    }

    void addEdgeToSink(OpNode* src, Tensor* tens_ptr)
    {
        mSinkNode.addInTensor(tens_ptr);
        addEdge(src, tens_ptr, &mSinkNode);
    }
};

class OpGraphBuilder
{
private:
    std::vector<OpNode*> mNodes;

public:
    bool hasNode(OpNode* node) const { return internal::contains(mNodes, node); }

    void addNode(OpNode* node)
    {
        assert(!hasNode(node));
        mNodes.emplace_back(node);
    }

    struct EdgeInfo
    {
        OpNode* mSrc = nullptr;
        std::vector<OpNode*> mDests{};
    };

    OpGraph build();
};


inline bool DegreeEqualityTest(const OpGraph& left, const OpGraph& right) {
  auto l_degs = left.getInOutDegrees();
  auto r_degs = right.getInOutDegrees();

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
  return l_degs == r_degs;
}

inline bool PathEqualityTest(const OpGraph& left, const OpGraph& right) {

  using MapSizeToPathVec = std::unordered_map<size_t, VecOfPaths>;

  auto group_by_size = [] (VecOfPaths&& all_paths) {
    MapSizeToPathVec paths_by_size;

    for (auto& p: all_paths) {
      auto [it, _ignore]  = paths_by_size.emplace(p.size(), VecOfPaths{});
      it->second.emplace_back(std::move(p));
    }

    return paths_by_size;
  };

  MapSizeToPathVec l_paths_by_sz{};
  auto r_paths_by_sz = l_paths_by_sz;

  {
    auto l_paths = left.getAllPaths();
    auto r_paths = right.getAllPaths();

    if (l_paths.size() != r_paths.size()) {
      return false;
    }

    auto sum_paths = [] (const VecOfPaths all_paths) {
      size_t ret = 0;
      for (const auto& p: all_paths) {
        ret += p.size();
      }
      return ret;
    };

    if (sum_paths(l_paths) != sum_paths(r_paths)) {
      return false;
    }

    l_paths_by_sz = group_by_size(std::move(l_paths));
    r_paths_by_sz = group_by_size(std::move(r_paths));
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

  auto check_equal_path_vecs = [](VecOfPaths& left, const VecOfPaths& right) {
    if (left.size() != right.size()) {
      return false;
    }

    std::sort(left.begin(), left.end());
    std::sort(right.begin(), right.end());

    return left == right;
  }

  for (size_t k: l_keys) {
    if (!check_equal_path_vecs(l_paths_by_sz[k], r_paths_by_sz[k])) {
      return false;
    }
  }

  return true;

}


inline bool isIsomorphic(const OpGraph& left, const OpGraph& right) {
  if (left.numNodes() != right.numNodes()) {
    return false;
  }

  if (!DegreeEqualityTest(left, right)) {
    return false;
  }
}




} // end namespace graphapi
} // end namespace miopen
