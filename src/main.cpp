#include <iostream>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
// #include <Recast.h>

#include "main.h"

using namespace std;

void demoFindRandomPoint(dtNavMeshQuery* navQuery, dtQueryFilter* filter) {
	float randomPoint[3];
    dtPolyRef startPolyRef;

	float halfExtents[3] = { 5.0f, 5.0f, 5.0f };
	dtPolyRef findPolyRef;

	bool valid = false;

	for (int i = 0; i < 10; i++) {
		dtStatus status = navQuery->findRandomPoint(filter, frand, &startPolyRef, randomPoint);
		if (dtStatusFailed(status)) {
			std::cerr << "Failed to find a random point. Error: " << status << std::endl;
			continue;
		}
		navQuery->findNearestPoly(randomPoint, halfExtents, filter, &findPolyRef, 0);
		valid = navQuery->isValidPolyRef(findPolyRef, filter);

		std::cout << "Random Point: (" << randomPoint[0] << ", " << randomPoint[1] << ", " << randomPoint[2] << "), valid: " << valid << std::endl;
	}
}

void demoRaycast(dtNavMeshQuery* navQuery, dtQueryFilter* filter) {
	// 点A的位置
    float startPoint[3] = {402.12, 0.182837, -428.127};
    // 移动方向B
    float moveDirection[3] = {0, 1, 0.5};
    // 要前进的距离
    float moveDistance = 10.0f;

    // 计算点D
    float endPoint[3];
    dtVmad(endPoint, startPoint, moveDirection, moveDistance);

    // 执行 raycast 操作，找到点D 位于哪个可行走区域
    const float ext[3] = {2.0f, 4.0f, 2.0f};  // 射线的扩展参数，根据需要调整
    dtPolyRef startRef;
    navQuery->findNearestPoly(startPoint, ext, filter, &startRef, nullptr);
    
    float hitNormal[3];
    float t = 0.0f;
    navQuery->raycast(startRef, startPoint, moveDirection, filter, &t, hitNormal, nullptr, nullptr, 0);

    if (t < 1.0f) {
        // 找到了可行走区域，计算点D的位置
        float hitPoint[3];
        dtVmad(hitPoint, startPoint, moveDirection, t * moveDistance);
        std::cout << "Point D is at (" << hitPoint[0] << ", " << hitPoint[1] << ", " << hitPoint[2] << ")" << std::endl;
    } else {
        std::cout << "No valid path found for point D." << std::endl;
    }
}

void demo() {
    const char* path = "testsync.bin";

    dtNavMesh* navMesh = load_mesh(path);
    
    dtNavMeshQuery* navQuery = dtAllocNavMeshQuery();

    navQuery->init(navMesh, 2048);

    dtQueryFilter filter;
    filter.setIncludeFlags(1); // You may need to adjust this flag based on your mesh.

	demoFindRandomPoint(navQuery, &filter);
	demoRaycast(navQuery, &filter);

    // Clean up resources.
	std::cout << "Cleaning up..." << std::endl;
    dtFreeNavMeshQuery(navQuery);
    dtFreeNavMesh(navMesh);
}

int main(int argc, char** argv)
{
    cout << "hello world!" << endl;

	demo();

    return 0;
}
