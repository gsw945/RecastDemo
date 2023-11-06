#include <iostream>

#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include "include/main.h"

using namespace std;

int demo() {
    const char* path = "testsync.bin";

    dtNavMesh* navMesh = load_mesh(path);
    
    dtNavMeshQuery* navQuery = dtAllocNavMeshQuery();

    navQuery->init(navMesh, 2048);

    dtQueryFilter filter;
    filter.setIncludeFlags(1); // You may need to adjust this flag based on your mesh.

    float randomPoint[3];
    dtPolyRef startPolyRef;

	float halfExtents[3] = { 5.0f, 5.0f, 5.0f };
	dtPolyRef findPolyRef;

	bool valid = false;

	for (int i = 0; i < 100; i++) {
		dtStatus status = navQuery->findRandomPoint(&filter, frand, &startPolyRef, randomPoint);
		if (dtStatusFailed(status)) {
			std::cerr << "Failed to find a random point. Error: " << status << std::endl;
			continue;
		}
		navQuery->findNearestPoly(randomPoint, halfExtents, &filter, &findPolyRef, 0);
		valid = navQuery->isValidPolyRef(findPolyRef, &filter);

		std::cout << "Random Point: (" << randomPoint[0] << ", " << randomPoint[1] << ", " << randomPoint[2] << "), valid: " << valid << std::endl;
	}

    // Clean up resources.
	std::cout << "Cleaning up..." << std::endl;
    dtFreeNavMeshQuery(navQuery);
    dtFreeNavMesh(navMesh);

    return 0;
}

int main(int argc, char** argv)
{
    cout << "hello world!" << endl;

	demo();

    return 0;
}
