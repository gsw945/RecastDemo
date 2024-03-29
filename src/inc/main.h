#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <DetourNavMesh.h>

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS
};

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

// Vector3 struct
template <typename T>
struct Vector3 {
    T x;
	T y;
	T z;
	Vector3 operator+(const Vector3& v) const {
		Vector3 result;
		result.x = x + v.x;
		result.y = y + v.y;
		result.z = z + v.z;
		return result;
	}
	Vector3 operator-(const Vector3& v) const {
		Vector3 result;
		result.x = x - v.x;
		result.y = y - v.y;
		result.z = z - v.z;
		return result;
	}
	Vector3 operator*(const T& f) const {
		Vector3 result;
		result.x = x * f;
		result.y = y * f;
		result.z = z * f;
		return result;
	}
	Vector3 operator/(const T& f) const {
		Vector3 result;
		result.x = x / f;
		result.y = y / f;
		result.z = z / f;
		return result;
	}
};

static float frand() {
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

dtNavMesh* load_mesh(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		std::cerr << "Failed to load NavMesh. Error: " << status << std::endl;
        return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

std::vector<int> stringSplit(std::string str, std::string delimiter = ", ") {
	std::vector<int> vect;
	std::stringstream ss(str);

	for (int i; ss >> i;) {
		vect.push_back(i);
		auto peek = ss.peek();
		for (size_t j = 0; j < delimiter.length(); j++)
		{
			if (peek == delimiter[j]) {
				ss.ignore();
			}
		}
	}

	return vect;
}

template <typename T>
std::string joinArray(T* array, size_t length, const char* delimiter = ", ") {
	std::ostringstream oss;
	for (int i = 0; i < length; i++) {
		if (i > 0) {
			oss << delimiter;
		}
		oss << array[i];
	}
	return oss.str();
}

template <typename T>
void printArray(T* array, size_t length, const char* delimiter = ", ") {
	std::cout << '[' << joinArray(array, length, delimiter) << ']' << std::endl;
}
