#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include "main.h"
#include "demo_part01.h"
#include "demo_part02.h"

std::string demoLoopInput() {
	std::cout << "  -> 0. exit" << std::endl;
	std::cout << "  -> 1. demo part 01" << std::endl;
	std::cout << "  -> 2. demo part 02" << std::endl;
	std::cout << "choose demo part: ";
	std::string input;
	std::getline(std::cin, input);
	return input;
}

void demo(const char* path) {
    dtNavMesh* navMesh = load_mesh(path);
	if (navMesh == 0) {
		std::cout << "Failed to load navmesh [" << path << "]" << std::endl;
		return;
	}

	// 获取导航网格的参数
	const dtNavMeshParams* navParams = navMesh->getParams();

	// 输出导航网格的参数信息
	std::cout << "Navigation Mesh Parameters:" << std::endl;
	std::cout << "Max Tiles: " << navMesh->getMaxTiles() << std::endl;
	std::cout << "Params.tileWidth: " << navParams->tileWidth <<std::endl;
	std::cout << "Params.tileHeight: " << navParams->tileHeight <<std::endl;
	std::cout << "Params.maxPolys: " << navParams->maxPolys << std::endl;
    
    dtNavMeshQuery* navQuery = dtAllocNavMeshQuery();

    navQuery->init(navMesh, 16384);

    dtQueryFilter filter;
    filter.setIncludeFlags(1); // You may need to adjust this flag based on your mesh.

	std::cout << "======================================================" << std::endl;
	int count = 0;
	while (true)
	{
		count += 1;
		std::cout << "++++++++++++++++++++++++ [loop=" << count << "] ++++++++++++++++++++" << std::endl;
		std::string input = demoLoopInput();
		if (input == "0") {
			break;
		}
		if (input == "1") {
			demoPart01(navQuery, &filter);
			continue;
		}
		if (input == "2") {
			demoPart02(navQuery, &filter);
			continue;
		}
		std::cout << "invalid input" << std::endl;
	}
	std::cout << "=======================================================" << std::endl;

    // Clean up resources.
	std::cout << "Cleaning up..." << std::endl;
    dtFreeNavMeshQuery(navQuery);
    dtFreeNavMesh(navMesh);
}

int main(int argc, char** argv)
{
    std::cout << "hello world!" << std::endl;

	const char* defaultNavmesh = "D:\\proj\\xxx\\bin\\navmesh\\testsync.bin";
	char * navmesh = getenv("RD_NAVMESH");
	if (navmesh == NULL) {
		std::cout << "env RD_NAVMESH not set, use default navmesh [" << defaultNavmesh <<"]" << std::endl;
		navmesh = strdup(defaultNavmesh);
	} else {
		std::cout << "navmesh use env RD_NAVMESH [" << navmesh << "]" << std::endl;
	}

	demo(navmesh);

    return 0;
}
