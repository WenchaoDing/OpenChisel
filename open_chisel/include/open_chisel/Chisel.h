// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CHISEL_H_
#define CHISEL_H_

#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include <open_chisel/camera/FisheyeCamera.h>

namespace chisel
{

class Chisel
{
  public:
    Chisel();
    Chisel(const Eigen::Vector3i &chunkSize, float voxelResolution);
    virtual ~Chisel();

    inline const ChunkManager &GetChunkManager() const
    {
        return chunkManager;
    }
    inline ChunkManager &GetMutableChunkManager()
    {
        return chunkManager;
    }
    inline void SetChunkManager(const ChunkManager &manager)
    {
        chunkManager = manager;
    }

    template <class DataType, class ColorType>
    void IntegrateDepthScanColor(const ProjectionIntegrator &integrator, const std::shared_ptr<const DepthImage<DataType>> &depthImage,
                                 const Transform &extrinsic, const std::shared_ptr<const ColorImage<ColorType>> &colorImage,
                                 const FisheyeCamera &camera, int number_of_threads)
    {
        printf("CHISEL: Integrating a color scan\n");
        auto wall_time = std::chrono::system_clock::now();
        Frustum frustum;
        camera.SetupFrustum(extrinsic, &frustum);

        ChunkIDList chunksIntersecting;
        chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - wall_time;
        printf("intersecting wall time: %f\n", elapsed.count() * 1000);

        wall_time = std::chrono::system_clock::now();
        int n = chunksIntersecting.size();
        std::vector<bool> isNew(n);
        std::vector<ChunkMap::iterator> newChunks(n);
        std::vector<bool> isGarbage(n);
        for (int i = 0; i < n; i++)
        {
            isNew[i] = false;
            isGarbage[i] = false;
            ChunkID chunkID = chunksIntersecting[i];
            if (!chunkManager.HasChunk(chunkID))
            {
                isNew[i] = true;
                newChunks[i] = chunkManager.CreateChunk(chunkID);
            }
        }
        printf("bucket_count: %d\n", chunkManager.GetBucketSize());
        elapsed = std::chrono::system_clock::now() - wall_time;
        printf("allocation wall time: %f\n", elapsed.count() * 1000);

        wall_time = std::chrono::system_clock::now();

        int nThread = number_of_threads;
        std::vector<int> debug_v;
        std::vector<std::thread> threads;
        std::mutex m;
        int blockSize = (n + nThread - 1) / nThread;
        printf("IntegrateDepthScanColor: n = %d, blockSize = %d, nThread = %d\n", n, blockSize, nThread);
        for (int i = 0; i < nThread; i++)
        {
            int s = i * blockSize;
            printf("thread: %d, s: %d, blockSize: %d\n", i, s, blockSize);
            threads.push_back(std::thread([s, n, blockSize, this, &m, &chunksIntersecting,
                                           &depthImage, &camera, &extrinsic, &colorImage, &integrator,
                                           &isNew, &isGarbage,
                                           &debug_v]()
                                          {
                                              for (int j = 0, k = s; j < blockSize && k < n; j++, k++)
                                              {
                                                  ChunkID chunkID = chunksIntersecting[k];
                                                  ChunkPtr chunk = this->chunkManager.GetChunk(chunkID);

                                                  bool needsUpdate = integrator.IntegrateColor(depthImage, camera, extrinsic, colorImage, chunk.get());
                                                  if (!needsUpdate && isNew[k])
                                                  {
                                                      isGarbage[k] = true;
                                                  }

                                                  if (needsUpdate)
                                                  {
                                                      m.lock();
                                                      for (int dx = -1; dx <= 1; dx++)
                                                      {
                                                          for (int dy = -1; dy <= 1; dy++)
                                                          {
                                                              for (int dz = -1; dz <= 1; dz++)
                                                              {
                                                                  this->meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                                              }
                                                          }
                                                      }
                                                      m.unlock();
                                                  }
                                              }
                                          }));
        }

        for (int i = 0; i < nThread; i++)
            threads[i].join();

        elapsed = std::chrono::system_clock::now() - wall_time;
        printf("integration wall time: %f\n", elapsed.count() * 1000);

        wall_time = std::chrono::system_clock::now();
        //ChunkIDList garbageChunks;
        for (int i = 0; i < n; i++)
            if (isGarbage[i])
            {
                chunkManager.RemoveChunk(newChunks[i]);
                //garbageChunks.push_back(chunksIntersecting[i]);
            }
        //GarbageCollect(garbageChunks);
        printf("CHISEL: Done with color scan\n");
        //chunkManager.PrintMemoryStatistics();
        elapsed = std::chrono::system_clock::now() - wall_time;
        printf("garbage wall time: %f\n", elapsed.count() * 1000);
    }

    void GarbageCollect(const ChunkIDList &chunks);
    void UpdateMeshes(int number_of_threads);

    bool SaveAllMeshesToPLY(const std::string &filename);
    void Reset();

    const ChunkSet &GetMeshesToUpdate() const
    {
        return meshesToUpdate;
    }

  protected:
    ChunkManager chunkManager;
    ChunkSet meshesToUpdate;
};
typedef std::shared_ptr<Chisel> ChiselPtr;
typedef std::shared_ptr<const Chisel> ChiselConstPtr;

} // namespace chisel

#endif // CHISEL_H_
