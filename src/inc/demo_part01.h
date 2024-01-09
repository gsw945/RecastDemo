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
	const dtNavMesh* navMesh = navQuery->getAttachedNavMesh();
	// 点A的位置
    float startPoint[3] = {-1230.68f, 120.91f, 853.76f};
	float targetPoint[3] = {-1252.89f, 124.57f, 853.29f};
    // 移动方向B
    float moveDirection[3];
	// 计算方向向量
	dtVsub(moveDirection, targetPoint, startPoint);
	// 目标距离
	float targetDistance = dtVlen(moveDirection);
	std::cout << "targetPoint is at (" << targetPoint[0] << ", " << targetPoint[1] << ", " << targetPoint[2] << ")" << std::endl;
	std::cout << "targetDistance: " << targetDistance << std::endl;
	std::cout << "moveDirection: (" << moveDirection[0] << ", " << moveDirection[1] << ", " << moveDirection[2] << ")"  << std::endl;
	// 向量归一化
	dtVnormalize(moveDirection);
	std::cout << "moveDirection normalize: (" << moveDirection[0] << ", " << moveDirection[1] << ", " << moveDirection[2] << ")"  << std::endl;
	// 向量长度日志
	std::cout << "moveDirection length: " << dtVlen(moveDirection) << std::endl;
    // 要前进的最大距离
    float moveDistance = 10.0f;

    // 计算点D
    float endPoint[3];
    dtVmad(endPoint, startPoint, moveDirection, moveDistance);
	std::cout << "Point A is at (" << startPoint[0] << ", " << startPoint[1] << ", " << startPoint[2] << ")" << std::endl;
	std::cout << "endPoint is at (" << endPoint[0] << ", " << endPoint[1] << ", " << endPoint[2] << ")" << std::endl;
	std::cout << "distance from startPoint to endPoint is " << dtVdist(startPoint, endPoint) << std::endl;
	float direction[3];
	dtVsub(direction, endPoint, startPoint);
	std::cout << "direction from startPoint to endPoint is " << direction[0] << ", " << direction[1] << ", " << direction[2] << ")"  << std::endl;
	float dir[3];
	dtVset(dir, direction[0], direction[1], direction[2]);
	dtVnormalize(dir);
	std::cout << "direction normalize: (" << dir[0] << ", " << dir[1] << ", " << dir[2] << ")"  << std::endl;

    // 执行 raycast 操作，找到点D 位于哪个可行走区域
    const float ext[3] = {2.0f, 4.0f, 2.0f};  // 射线的扩展参数，根据需要调整

    dtPolyRef startRef;
	float startPt[3];
    dtStatus status = navQuery->findNearestPoly(startPoint, ext, filter, &startRef, startPt);
	if (dtStatusSucceed(status)) {
		// 根据startRef查询tile
		const dtMeshTile* start_tile = nullptr;
		const dtPoly* start_poly = nullptr;
		navMesh->getTileAndPolyByRefUnsafe(startRef, &start_tile, &start_poly);
		if (start_poly == nullptr || start_poly->firstLink == DT_NULL_LINK) {
			std::cout << "Point A is not in any polygon." << std::endl;
			return;
		} else {
			std::cout << "start_poly->firstLink: " << start_poly->firstLink << ", start_poly->vertCount: " << (int32_t)(start_poly->vertCount) << ", DT_NULL_LINK: " << DT_NULL_LINK << std::endl;
		}
		std::cout << "Point A is in polygon " << startRef << ", pt: (" << startPt[0] << ", " << startPt[1] << ", " << startPt[2] << ")" << std::endl;
	} else {
		std::cout << "Point A is not in any polygon." << std::endl;
		return;
	}

	dtPolyRef endRef;
	float endPt[3];
	status = navQuery->findNearestPoly(endPoint, ext, filter, &endRef, endPt);
	if (dtStatusSucceed(status)) {
		// 根据endRef查询tile
		const dtMeshTile* end_tile = nullptr;
		const dtPoly* end_poly = nullptr;
		navMesh->getTileAndPolyByRefUnsafe(endRef, &end_tile, &end_poly);
		if (end_poly == nullptr || end_poly->firstLink == DT_NULL_LINK) {
			std::cout << "endPoint is not in any polygon." << std::endl;
			return;
		} else {
			std::cout << "end_poly->firstLink: " << end_poly->firstLink << ", end_poly->vertCount: " << (int32_t)(end_poly->vertCount) << std::endl;
		}
		std::cout << "endPoint is in polygon " << endRef << ", pt: (" << endPt[0] << ", " << endPt[1] << ", " << endPt[2] << ")" << std::endl;
	} else {
		std::cout << "endPoint is not in any polygon." << std::endl;
		return;
	}
    
    float hitNormal[3];
    float t = 0.0f;
	const int MAX_POLYS = 256;
	dtPolyRef path[MAX_POLYS];
	int pathCount = 0;
    status = navQuery->raycast(startRef, startPoint, endPoint, filter, &t, hitNormal, path, &pathCount, MAX_POLYS);
	if (dtStatusFailed(status)) {
		std::cout << "Raycast failed. status: " << status << std::endl;

		bool result = !navMesh->isValidPolyRef(startRef) || !startPoint || !dtVisfinite(startPoint) || !endPoint || !dtVisfinite(endPoint) || !filter;
		std::cout << "result: " << result << std::endl;
		std::cout << "!navMesh->isValidPolyRef(startRef): " << !navMesh->isValidPolyRef(startRef) << std::endl;
		std::cout << "!startPoint: " << !startPoint << std::endl;
		std::cout << "!dtVisfinite(startPoint): " << !dtVisfinite(startPoint) << std::endl;
		std::cout << "!endPoint: " << !endPoint << std::endl;
		std::cout << "!dtVisfinite(endPoint): " << !dtVisfinite(endPoint) << std::endl;
		std::cout << "!filter: " << !filter << std::endl;
		return;
	}
    if (0 < t && t <= 1.0f) {
		std::cout << "hitNormal: (" << hitNormal[0] << ", " << hitNormal[1] << ", " << hitNormal[2] << "), t: " << t << std::endl;
        // 找到了可行走区域，计算点D的位置
        float hitPoint[3];
        // dtVmad(hitPoint, startPoint, moveDirection, t * moveDistance);
		dtVlerp(hitPoint, startPoint, endPoint, t);
		// Adjust height.
		if (pathCount > 0)
		{
			float h = 0;
			navQuery->getPolyHeight(path[pathCount-1], hitPoint, &h);
			hitPoint[1] = h;
		}
        std::cout << "Point D is at (" << hitPoint[0] << ", " << hitPoint[1] << ", " << hitPoint[2] << ")" << std::endl;
		std::cout << "distance from A to D is " << t * moveDistance << std::endl;
		std::cout << "distance manual: " << dtVdist(startPoint, hitPoint) << std::endl;
	} else {
        std::cout << "No valid path found for point D. t: " << t << ", pathCount: " << pathCount << std::endl;
    }
}

void demoAlongMove(dtNavMeshQuery* navQuery, dtQueryFilter* filter) {
	const dtNavMesh* navMesh = navQuery->getAttachedNavMesh();
	// 点A的位置
	float agentPosition[3] = {-1230.68f, 120.91f, 853.76f};
	float refPoint[3] = {-1252.89f, 124.57f, 853.29f};
    // 移动方向B
    float agentDirection[3];
	// 计算方向向量
	dtVsub(agentDirection, refPoint, agentPosition);
	// 向量归一化
	dtVnormalize(agentDirection);
	
	// 移动速度，假设为5m/s
	float moveSpeed = 5.0f;

	// 计算移动的距离
	float moveDistance = moveSpeed * 5.0f;

	const float ext[3] = {2.0f, 4.0f, 2.0f};  // 射线的扩展参数，根据需要调整
	// 查找Agent所在的多边形
	dtPolyRef startRef;

	dtStatus status = navQuery->findNearestPoly(agentPosition, ext, filter, &startRef, nullptr);
	if (dtStatusFailed(status)) {
		std::cout << "findNearestPoly failed." << std::endl;
		return;
	}
	std::cout << "is startRef valid: " << navQuery->isValidPolyRef(startRef, filter) << std::endl;
	float height;
	status = navQuery->getPolyHeight(startRef, agentPosition, &height);
	if (dtStatusFailed(status)) {
		std::cout << "getPolyHeight failed." << std::endl;
		return;
	} else {
		std::cout << "height of agent's ploy: " << height << std::endl;
	}
	float endPoint[3];
    dtVmad(endPoint, agentPosition, agentDirection, moveDistance);
	// 计算新的位置P2
	float newPosition[3] = {0.0f, 0.0f, 0.0f};
	dtPolyRef visited;
	int visitedCount;
	int maxVisitedSize = 0;
	status = navQuery->moveAlongSurface(startRef, agentPosition, endPoint, filter, newPosition, &visited, &visitedCount, maxVisitedSize);
	if (dtStatusFailed(status)) {
		std::cout << "moveAlongSurface failed." << std::endl;
		return;
	}

	// 计算新的朝向D2
	float newDirection[3];
	status = navQuery->getPolyWallSegments(startRef, filter, newDirection, nullptr, nullptr, 0);
	if (dtStatusFailed(status)) {
		std::cout << "getPolyWallSegments failed." << std::endl;
		return;
	}

	// 输出新的位置和朝向
	std::cout << "Agent的新位置P2: (" << newPosition[0] << ", " << newPosition[1] << ", " << newPosition[2] << ")" << std::endl;
	std::cout << "Agent的新朝向D2: (" << newDirection[0] << ", " << newDirection[1] << ", " << newDirection[2] << ")" << std::endl;
}

void demoAlongGround(dtNavMeshQuery* navQuery, dtQueryFilter* filter) {
	const dtNavMesh* navMesh = navQuery->getAttachedNavMesh();
	float agentPosition[3] = {-1244.92f, 124.57f, 850.95f};
	float startPoint[3];
	dtVcopy(startPoint, agentPosition);
	// startPoint[1] = FLT_MAX; // y 置 FLT_MAX
	dtPolyRef startRef;
	float startPt[3];
	// float halfExtents[3] = {2.0f, 4.0f, 2.0f};  // 射线的扩展参数，根据需要调整
	float halfExtents[3] = {2.0f, 24.0f, 2.0f};  // 射线的扩展参数，根据需要调整
	dtStatus status;
	startPoint[1] -= 20.0f;
	status = navQuery->findNearestPoly(startPoint, halfExtents, filter, &startRef, startPt);
	std::cout << "startPoint: (" << startPoint[0] << ", " << startPoint[1] << ", " << startPoint[2] << ")" << std::endl;
	std::cout << "startPt: (" << startPt[0] << ", " << startPt[1] << ", " << startPt[2] << ")" << std::endl;
	/*
	int i = FLT_MAX;
	while (i -- > - FLT_MAX) {
		startPoint[1] += 0.001f;
		status = navQuery->findNearestPoly(startPoint, halfExtents, filter, &startRef, startPt);
		if (dtStatusFailed(status) || startRef == 0) {
			continue;
		} else {
			break;
		}
	}
	if (i <= - FLT_MAX) {
		std::cout << "findNearestPoly(startPoint) failed" << std::endl;
		return;
	} else {
		std::cout << "startRef: " << startRef << ", i: " << i << ", startPoint: (" << startPt[0] << ", " << startPt[1] << ", " << startPt[2] << ")" << std::endl;
		std::cout << "startPoint: (" << startPoint[0] << ", " << startPoint[1] << ", " << startPoint[2] << ")" << std::endl;
		std::cout << "startPt: (" << startPt[0] << ", " << startPt[1] << ", " << startPt[2] << ")" << std::endl;
	}
	// 沿着y轴向下移动，直到到达地面
	float endPos[3];
	dtVcopy(endPos, agentPosition);
	endPos[1] = 0.0f - FLT_MAX;
	float newPosition[3] = {0.0f, 0.0f, 0.0f};
	dtPolyRef visited;
	int visitedCount;
	int maxVisitedSize = 256;
	status = navQuery->moveAlongSurface(startRef, startPoint, endPos, filter, newPosition, &visited, &visitedCount, maxVisitedSize);
	if (dtStatusFailed(status)) {
		std::cout << "moveAlongSurface() failed, visitedCount: " << visitedCount << std::endl;
		return;
	} else {
		std::cout << "newPosition: (" << newPosition[0] << ", " << newPosition[1] << ", " << newPosition[2] << ")" << std::endl;
	}
	*/
}

void demoPart01(dtNavMeshQuery* navQuery, dtQueryFilter* filter) {
	std::cout << "-----------------[demoFindRandomPoint]----------------" << std::endl;
	demoFindRandomPoint(navQuery, filter);
	std::cout << "---------------------[demoRaycast]--------------------" << std::endl;
	demoRaycast(navQuery, filter);
	std::cout << "---------------------[demoAlongMove]-------------------" << std::endl;
	demoAlongMove(navQuery, filter);
	std::cout << "---------------------[demoAlongGround]-----------------" << std::endl;
	demoAlongGround(navQuery, filter);
}