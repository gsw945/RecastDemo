bool parseInputPoint(std::string input, float* point) {
	std::vector<int> vect = stringSplit(input, ',');
	int size = vect.size();
	if (size != 3) {
		std::cout << "input error" << std::endl;
		return false;
	}
	float* result = new float[size];
	for (size_t i = 0; i < size; i++)
	{
		std::cout << vect.at(i) << std::endl;
		result[i] = vect.at(i) / 100.0f;
	}
	point = result;
	return true;
}

void pathFindInput(dtNavMeshQuery* navQuery, dtQueryFilter* filter) {
	std::string start;
	std::cout << "input start point(format: x,y,z, eg: 22,0,34): ";
	std::cin >> start;
	float* startPos = nullptr;
	if (!parseInputPoint(start, startPos)) {
		std::cout << "parseInputPoint(start) failed" << std::endl;
		return;
	}
	
	std::string end;
	std::cout << "input end point(format: x,y,z, eg: 22,0,34): ";
	std::cin >> end;
	float* endPos = nullptr;
	if (!parseInputPoint(end, endPos)) {
		std::cout << "parseInputPoint(end) failed" << std::endl;
		return;
	}

	dtPolyRef startRef;
	dtPolyRef endRef;
	float extents[3] = { 2, 4, 2 };

	if (navQuery->findNearestPoly(startPos, extents, filter, &startRef, 0) != DT_SUCCESS) {
		std::cout << "findNearestPoly failed: startPos" << std::endl;
		return;
	}
	if (navQuery->findNearestPoly(endPos, extents, filter, &endRef, 0) != DT_SUCCESS) {
		std::cout << "findNearestPoly failed: endPos" << std::endl;
		return;
	}

	static const int MAX_POLYS = 256;

	// Find path
	dtPolyRef path[MAX_POLYS];
	int pathCount;
	if (navQuery->findPath(startRef, endRef, startPos, endPos, filter, path, &pathCount, 256) != DT_SUCCESS) {
		std::cout << "findPath failed" << std::endl;
		return;
	}
	std::cout << "pathCount: " << pathCount << std::endl;
	if (pathCount < 1) {
		return;
	}

	// Find straight path
	float* straightPath = nullptr;
	int straightPathCount = 0;
	if (navQuery->findStraightPath(startPos, endPos, path, pathCount, straightPath, 0, 0, &straightPathCount, MAX_POLYS) != DT_SUCCESS) {
		std::cout << "findStraightPath failed" << std::endl;
		return;
	}
	std::cout << "straightPathCount: " << straightPathCount << std::endl;
	if (straightPathCount <= 0 || straightPathCount %3 != 0) {
		return;
	}

	// Copy results
	for (int i = 0; i < straightPathCount * 3; i++)
	{
		std::cout << i << " -> (" << straightPath[i] << "," << straightPath[i+1] << "," << straightPath[i+2] << ")";
	}
	std::cout << std::endl;
}

void demoPart02(dtNavMeshQuery* navQuery, dtQueryFilter* filter) {
	std::cout << "-----------------[stringSplit]----------------" << std::endl;
	std::vector<int> result = stringSplit("100,23434,3333", ',');
	for (std::size_t i = 0; i < result.size(); i++) {
		std::cout << result[i] << std::endl;
	}
	std::cout << "-----------------[pathFindInput]----------------" << std::endl;
	pathFindInput(navQuery, filter);
}