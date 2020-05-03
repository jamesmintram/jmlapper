#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>

#include <assimp/postprocess.h>

#include <vector>

#include "assimp/scene.h"
#include "assimp/mesh.h"

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

struct Interval
{
	float start;
	float end;
};

struct LineSegment
{
	aiVector3D pos;
	aiVector3D end;
};

struct Plane
{
	aiVector3D  normal;
	float dist;
};

struct Aabb
{
	aiVector3D min;
	aiVector3D max;
};

struct Triangle
{
	aiVector3D v0;
	aiVector3D v1;
	aiVector3D v2;
};

struct face
{
	uint32_t face_count;
	Triangle *world_verts;
	Aabb aabb;
	uint32_t *node_idx;
	uint32_t *mesh_idx;
};

inline aiVector3D calcNormal(const aiVector3D& _va, const aiVector3D& _vb, const aiVector3D& _vc)
{
	const aiVector3D ba = (_vb - _va);
	const aiVector3D ca = (_vc - _va);
	aiVector3D baxca = (ba ^ ca);

	baxca.NormalizeSafe();
	return baxca;
}

bool is_degenerate(Triangle const& tri)
{
	const aiVector3D ba = (tri.v1 - tri.v0);
	const aiVector3D ca = (tri.v2 - tri.v0);

	aiVector3D baxca = (ba ^ ca);

	return baxca.SquareLength() == 0;
}

inline void calcPlane(Plane& _outPlane, const aiVector3D& _normal, const aiVector3D& _pos)
{
	_outPlane.normal = _normal;
	_outPlane.dist = -(_normal * _pos);
}

inline void calcPlane(Plane& _outPlane, const aiVector3D& _va, const aiVector3D& _vb, const aiVector3D& _vc)
{
	aiVector3D normal = calcNormal(_va, _vb, _vc);
	calcPlane(_outPlane, normal, _va);
}

void calcPlane(Plane& _outPlane, const Triangle& _triangle)
{
	calcPlane(_outPlane, _triangle.v0, _triangle.v1, _triangle.v2);
}

aiVector3D getCenter(const Aabb& _aabb)
{
	return (_aabb.min + _aabb.max) * 0.5f;
}

aiVector3D getExtents(const Aabb& _aabb)
{
	return (_aabb.max - _aabb.min) * 0.5f;
}

inline float distance(const Plane& _plane, const aiVector3D& _pos)
{
	return (_plane.normal * _pos) + _plane.dist;
}

float distance(const Plane& _plane, const LineSegment& _line)
{
	const float pd = distance(_plane, _line.pos);
	const float ed = distance(_plane, _line.end);
	return min(min(max(pd*ed, 0.0f), fabs(pd)), fabs(ed));
}

aiVector3D vfabs(aiVector3D const& vec)
{
	return {
		fabs(vec.x),
		fabs(vec.y),
		fabs(vec.z)
	};
}

bool overlap(const Aabb& _aabb, const Plane& _plane)
{
	const aiVector3D center = getCenter(_aabb);
	const float dist = distance(_plane, center);

	const aiVector3D extents = getExtents(_aabb);
	const aiVector3D normal = vfabs(_plane.normal);
	const float radius = (extents * normal);

	return fabs(dist) <= radius;
}

Interval projectToAxis(const aiVector3D& _axis, const Triangle& _triangle)
{
	const float a0 = (_axis * _triangle.v0);
	const float a1 = (_axis * _triangle.v1);
	const float a2 = (_axis * _triangle.v2);
	return
	{
		min(min(a0, a1), a2),
		max(max(a0, a1), a2),
	};
}

aiVector3D vmax(aiVector3D const& lhs, aiVector3D const& rhs)
{
	return { max(lhs.x, rhs.x), max(lhs.y, rhs.y), max(lhs.z,rhs.z) };
}
aiVector3D vmin(aiVector3D const& lhs, aiVector3D const& rhs)
{
	return { min(lhs.x, rhs.x), min(lhs.y, rhs.y), min(lhs.z,rhs.z) };
}



Interval projectToAxis(const aiVector3D& _axis, const Aabb& _aabb)
{
	const float extent = fabs(vfabs(_axis) * getExtents(_aabb));
	const float center = (_axis * getCenter(_aabb));
	return
	{
		center - extent,
		center + extent,
	};
}

void toAabb(Aabb& _outAabb, const Triangle& _triangle)
{
	_outAabb.min = vmin(_triangle.v0, vmin(_triangle.v1, _triangle.v2));
	_outAabb.max = vmax(_triangle.v0, vmax(_triangle.v1, _triangle.v2));
}

bool overlap(const Aabb& _aabbA, const Aabb& _aabbB)
{
	return true
		&& _aabbA.max.x > _aabbB.min.x
		&& _aabbB.max.x > _aabbA.min.x
		&& _aabbA.max.y > _aabbB.min.y
		&& _aabbB.max.y > _aabbA.min.y
		&& _aabbA.max.z > _aabbB.min.z
		&& _aabbB.max.z > _aabbA.min.z
		;
}

bool overlap(const Interval& _a, const Interval& _b)
{
	return _a.end >= _b.start
		&& _b.end >= _a.start
		;
}

static const aiVector3D kAxis[] =
{
	{ 1.0f, 0.0f, 0.0f },
	{ 0.0f, 1.0f, 0.0f },
	{ 0.0f, 0.0f, 1.0f },
};

bool overlap(const Aabb& _aabb, const Triangle& _triangle)
{
	Aabb triAabb;
	toAabb(triAabb, _triangle);

	if (!overlap(_aabb, triAabb))
	{
		return false;
	}

	Plane plane;
	calcPlane(plane, _triangle);

	if (!overlap(_aabb, plane))
	{
		return false;
	}

	const aiVector3D center = getCenter(_aabb);
	const aiVector3D v0 = (_triangle.v0 - center);
	const aiVector3D v1 = (_triangle.v1 - center);
	const aiVector3D v2 = (_triangle.v2 - center);

	const aiVector3D edge[] =
	{
		(v1 - v0),
		(v2 - v1),
		(v0 - v2),
	};

	for (uint32_t ii = 0; ii < 3; ++ii)
	{
		for (uint32_t jj = 0; jj < 3; ++jj)
		{
			const aiVector3D axis = (kAxis[ii] ^ edge[jj]);

			const Interval aabbR = projectToAxis(axis, _aabb);
			const Interval triR = projectToAxis(axis, _triangle);

			if (!overlap(aabbR, triR))
			{
				return false;
			}
		}
	}

	return true;
}

__forceinline aiVector3D rcp(aiVector3D const& vec)
{
	return
	{
		1.0f / vec.x,
		1.0f / vec.y,
		1.0f / vec.z,
	};
}

__forceinline aiVector3D vmul(aiVector3D const& lhs, aiVector3D const& rhs)
{
	return {
		lhs.x * rhs.x,
		lhs.y * rhs.y,
		lhs.z * rhs.z
	};
}

bool intersect(aiVector3D const& orig, aiVector3D const& dir, const Aabb& _aabb)
{
	const aiVector3D invDir = rcp(dir);
	const aiVector3D tmp0 = (_aabb.min - orig);
	const aiVector3D t0 = vmul(tmp0, invDir);
	const aiVector3D tmp1 = (_aabb.max - orig);
	const aiVector3D t1 = vmul(tmp1, invDir);

	const aiVector3D mn = vmin(t0, t1);
	const aiVector3D mx = vmax(t0, t1);

	const float tmin = max(max(mn.x, mn.y), mn.z);
	const float tmax = min(min(mx.x, mx.y), mx.z);

	if (0.0f > tmax
		|| tmin > tmax)
	{
		return false;
	}

	//if (NULL != _hit)
	//{
	//	_hit->plane.normal.x = float((t1.x == tmin) - (t0.x == tmin));
	//	_hit->plane.normal.y = float((t1.y == tmin) - (t0.y == tmin));
	//	_hit->plane.normal.z = float((t1.z == tmin) - (t0.z == tmin));

	//	_hit->plane.dist = tmin;
	//	_hit->pos = getPointAt(_ray.dir, tmin);
	//}

	return true;
}

bool intersect(
	const aiVector3D &orig,
	const aiVector3D &dir,

	const aiVector3D &v0,
	const aiVector3D &v1,
	const aiVector3D &v2,

	float &tt)
{

	const aiVector3D edge10 = (v1 - v0);
	const aiVector3D edge02 = (v0 - v2);
	const aiVector3D normal = (edge02 ^ edge10);
	const aiVector3D vo = (v0 - orig);
	const aiVector3D dxo = (dir ^ vo);
	const float det = (normal * dir);

	const float invDet = 1.0f / det;
	const float bz = (dxo * edge02) * invDet;
	const float by = (dxo * edge10) * invDet;
	const float bx = 1.0f - by - bz;

	if (0.0f > bx
		|| 0.0f > by
		|| 0.0f > bz)
	{
		return false;
	}

	tt = (normal * vo) * invDet;
	return true;
}


struct OctreeBin
{
	Aabb aabb;
	uint32_t first_idx;
	uint32_t count;
};

struct Octree
{
	OctreeBin *bins;
	std::vector<Triangle> tris;
	uint32_t bin_count;
};


bool occluded(const face* faces, aiVector3D const& start, aiVector3D const& end)
{
	//aiNode const* root = scene->mRootNode;
	aiVector3D const rayVector = (end - start);
	aiVector3D  rayDir = rayVector;
	rayDir.NormalizeSafe();

	for (uint32_t face_idx = 0; face_idx < faces->face_count; face_idx++)
	{
		float tt;
		//Can we do 16 at a time?
		const bool intersects = intersect(
			start,
			rayDir,
			faces->world_verts[face_idx].v0,
			faces->world_verts[face_idx].v1,
			faces->world_verts[face_idx].v2,
			tt);

		if (intersects && tt > 0.01f && tt < rayVector.Length())
		{
			return true;
		}
	}

	return false;
}

bool oct_occluded(Octree const& octree, aiVector3D const& start, aiVector3D const& end)
{
	aiVector3D const rayVector = (end - start);
	aiVector3D  rayDir = rayVector;
	rayDir.NormalizeSafe();

	//Figure out which bins we intersect
	//	Check that bin

	for (uint32_t bin_idx = 0; bin_idx < octree.bin_count; bin_idx++)
	{
		if (intersect(start, rayDir, octree.bins[bin_idx].aabb))
		{
			const uint32_t first = octree.bins[bin_idx].first_idx;
			const uint32_t last = first + octree.bins[bin_idx].count;

			for (int face_idx = first; face_idx < last; face_idx++)
			{
				float tt;
				//Can we do 16 at a time?
				const bool intersects = intersect(
					start,
					rayDir,
					octree.tris[face_idx].v0,
					octree.tris[face_idx].v1,
					octree.tris[face_idx].v2,
					tt);

				if (intersects && tt > 0.01f && tt < rayVector.Length())
				{
					return true;
				}
			}
		}
	}

	return false;
}


face *buildWorldFaces(aiScene const *scene)
{
	aiNode const* root = scene->mRootNode;

	//Count faces
	uint32_t faceCount = 0;

	for (uint32_t nodeIdx = 0; nodeIdx < root->mNumChildren; nodeIdx++)
	{
		aiNode* node = root->mChildren[nodeIdx];

		for (uint32_t meshIdx = 0; meshIdx < node->mNumMeshes; meshIdx++)
		{
			aiMesh const* mesh = scene->mMeshes[node->mMeshes[meshIdx]];
			faceCount += mesh->mNumFaces;
		}
	}

	//Alloc
	face* faces = new face;
	faces->face_count = faceCount;
	faces->mesh_idx = new uint32_t[faceCount];
	faces->node_idx = new uint32_t[faceCount];
	faces->world_verts = new Triangle[faceCount];

	uint32_t newFaceIdx = 0;

	//Populate faces
	for (uint32_t nodeIdx = 0; nodeIdx < root->mNumChildren; nodeIdx++)
	{
		aiNode const* node = root->mChildren[nodeIdx];

		for (uint32_t meshIdx = 0; meshIdx < node->mNumMeshes; meshIdx++)
		{
			aiMesh const* mesh = scene->mMeshes[node->mMeshes[meshIdx]];

			for (uint32_t faceIdx = 0; faceIdx < mesh->mNumFaces; faceIdx++)
			{
				aiFace const &face = mesh->mFaces[faceIdx];

				if (face.mNumIndices != 3)
				{
					continue;
				}

				aiVector3D world_pos[] = {
					node->mTransformation * mesh->mVertices[face.mIndices[0]],
					node->mTransformation * mesh->mVertices[face.mIndices[1]],
					node->mTransformation * mesh->mVertices[face.mIndices[2]]
				};

				faces->mesh_idx[newFaceIdx] = meshIdx;
				faces->node_idx[newFaceIdx] = nodeIdx;

				memcpy(&faces->world_verts[newFaceIdx], world_pos, sizeof(world_pos));

				// Update scene Aabb
				for (int vidx = 0; vidx < 3; vidx++)
				{
					faces->aabb.min = vmin(world_pos[vidx], faces->aabb.min);
					faces->aabb.max = vmax(world_pos[vidx], faces->aabb.max);
				}

				newFaceIdx++;
			}
		}
	}

	return faces;
}


void build_octree(face* faces, uint32_t tree_depth, Octree &octree)
{
	const uint32_t axis_div = 1 << tree_depth;
	const uint32_t bin_count = axis_div * axis_div * axis_div;

	aiVector3D scene_aabb = (faces->aabb.max - faces->aabb.min);
	aiVector3D bin_size = {
		scene_aabb.x / axis_div,
		scene_aabb.y / axis_div,
		scene_aabb.z / axis_div,
	};

	octree.bin_count = bin_count;
	octree.bins = new OctreeBin[bin_count];
	octree.tris.reserve(int(faces->face_count * 1.1f)); // Reserve enough for all faces + 10% (ie faces in more than 1 bin)

	uint32_t degenerate_count = 0;
	uint32_t octree_tri_idx = 0;
	uint32_t bin_idx = 0;

	for (uint32_t x = 0; x < axis_div; x++)
	{
		for (uint32_t y = 0; y < axis_div; y++)
		{
			for (uint32_t z = 0; z < axis_div; z++)
			{
				octree.bins[bin_idx].first_idx = octree_tri_idx;
				octree.bins[bin_idx].aabb = {
					{	faces->aabb.min.x + x * bin_size.x,
						faces->aabb.min.y + y * bin_size.y,
						faces->aabb.min.z + z * bin_size.z },
					{	faces->aabb.min.x + (x + 1) * bin_size.x,
						faces->aabb.min.y + (y + 1) * bin_size.y,
						faces->aabb.min.z + (z + 1) * bin_size.z },
				};

				// Add all triangles that intersect this bin
				// TODO: Might need to update this to traverse scene again and do a AABB test before a triangle test
				for (uint32_t faceIdx = 0; faceIdx < faces->face_count; faceIdx++)
				{
					if (is_degenerate(faces->world_verts[faceIdx]))
					{
						degenerate_count++;
						continue;
					}

					//Overlap test between face (triangle) and bin (aabb)
					if (overlap(octree.bins[bin_idx].aabb, faces->world_verts[faceIdx]))
					{
						octree.tris.push_back(faces->world_verts[faceIdx]);
						octree_tri_idx++;
					}
				}

				octree.bins[bin_idx].count = octree_tri_idx - octree.bins[bin_idx].first_idx;
				bin_idx++;
			}
		}
	}


	//TODO: We could organize our bins in a recursive tree fashion
	//		splitting each axis in half.
	//		This would accellerate the usage of the Octree significantly
	//		when the depth > 3
}

int main(int argc, char* argv[])
{
	Assimp::Exporter exporter;
	Assimp::Importer importer;

	const char* filePath = "scene_pack_hi_res.fbx";

	const aiScene* scene = importer.ReadFile(filePath, aiProcess_Triangulate);

	aiNode* root = scene->mRootNode;

	face* faces = buildWorldFaces(scene);

	Octree octree;
	build_octree(faces, 3, octree);

	for (uint32_t lightIdx = 0; lightIdx < scene->mNumLights; lightIdx++)
	{
		aiLight *light = scene->mLights[lightIdx];
		aiNode* lightNode = root->FindNode(light->mName);

		if (lightNode != nullptr)
		{
			aiMatrix4x4 &tx = lightNode->mTransformation;

			//TODO: DOesn't do hierarchy
			aiVector3D worldLightPos = lightNode->mTransformation * light->mPosition;

			light->mPosition = worldLightPos;
		}
	}


	//TODO: For each light
	//TODO:  Find all Octree bins which intersect 
	//TODO:    For each bin, calculate light contribution
	//--------- Done ---- Probably MUCH faster, maybe even ultra fast?
	//TODO: If light sphere + mesh AABB do not intersect, continue


	// This could be out?
	for (uint32_t nodeIdx = 0; nodeIdx < root->mNumChildren; nodeIdx++)
	{
		printf("Node: %d/%d \n", nodeIdx, root->mNumChildren);
		aiNode* node = root->mChildren[nodeIdx];
		aiMatrix3x3t<float> const rotMat(node->mTransformation);
		auto temp = rotMat;
		temp.Inverse();	//Evil
		aiMatrix3x3t<float> const rotInvMat = temp;


		for (uint32_t meshIdx = 0; meshIdx < node->mNumMeshes; meshIdx++)
		{
			//printf("Mesh: %d / %d\n", meshIdx, node->mNumMeshes);
			aiMesh* mesh = scene->mMeshes[node->mMeshes[meshIdx]];
			if (mesh->HasVertexColors(0) == false)
			{
				mesh->mColors[0] = new aiColor4D[mesh->mNumVertices];
			}



			for (uint32_t vertIdx = 0; vertIdx < mesh->mNumVertices; vertIdx++)
			{
				//printf("Vert %d/%d\n", vertIdx, mesh->mNumVertices);
				aiVector3D vertPos = mesh->mVertices[vertIdx];
				aiVector3D vertNorm = {
					mesh->mNormals[vertIdx].x,
					mesh->mNormals[vertIdx].y,
					mesh->mNormals[vertIdx].z
				};

				aiColor4D lightAccum;

				aiVector3D worldVertPos = node->mTransformation * vertPos;

				for (uint32_t lightIdx = 0; lightIdx < scene->mNumLights; lightIdx++)
				{
					aiLight* light = scene->mLights[lightIdx];
					aiVector3D const lightVector = (light->mPosition - worldVertPos);
					aiVector3D lightDir = lightVector; lightDir.NormalizeSafe();

					float lightDist = lightVector.Length();

					if (lightDist > 400) continue;

					//Now we need to check it this vert can "see" the light
					if (oct_occluded(octree, worldVertPos, light->mPosition)) continue;


					aiVector3D localLight = rotInvMat * lightDir;
					//localLight.NormalizeSafe();

					float nDotL = localLight * vertNorm;

					float intensity = max(0, nDotL);

					if (intensity == 0)
					{
						printf("");
					}

					const float falloff = (400.0f - lightDist) / 400.0f;

					aiColor3D col = {
						min(1, light->mColorDiffuse.r / 100.0f),
						min(1, light->mColorDiffuse.g / 100.0f),
						min(1, light->mColorDiffuse.b / 100.0f),
					};
					lightAccum.r += (col.r * intensity) * falloff;// / (lightDist * 0.02f);
					lightAccum.g += (col.g * intensity) * falloff;// / (lightDist * 0.02f);
					lightAccum.b += (col.b * intensity) * falloff; // / (lightDist * 0.02f);
				}

				mesh->mColors[0][vertIdx] = lightAccum;
				mesh->mColors[0][vertIdx].a = 1;
			}
		}
	}




	exporter.Export(scene, "glb2", "scene_lit.glb");

	if (scene != nullptr)
	{
		printf("Done");
	}
}



//
//
//
//const char* filePath = "";
//
//ImportData import;
//
//
//
//// import the main model
//const aiScene* scene = ImportModel(import, filePath);
//if (!scene) {
//	printf("assimp info: Unable to load input file %s\n",
//		in.c_str());
//	return 5;
//}
//
//aiMemoryInfo mem;
//globalImporter->GetMemoryRequirements(mem);
//
//
//static const char* format_string =
//"Memory consumption: %i B\n"
//"Nodes:              %i\n"
//"Maximum depth       %i\n"
//"Meshes:             %i\n"
//"Animations:         %i\n"
//"Textures (embed.):  %i\n"
//"Materials:          %i\n"
//"Cameras:            %i\n"
//"Lights:             %i\n"
//"Vertices:           %i\n"
//"Faces:              %i\n"
//"Bones:              %i\n"
//"Animation Channels: %i\n"
//"Primitive Types:    %s\n"
//"Average faces/mesh  %i\n"
//"Average verts/mesh  %i\n"
//"Minimum point      (%f %f %f)\n"
//"Maximum point      (%f %f %f)\n"
//"Center point       (%f %f %f)\n"
//
//;
//
//aiVector3D special_points[3];
//FindSpecialPoints(scene, special_points);
//printf(format_string,
//	mem.total,
//	CountNodes(scene->mRootNode),
//	GetMaxDepth(scene->mRootNode),
//	scene->mNumMeshes,
//	scene->mNumAnimations,
//	scene->mNumTextures,
//	scene->mNumMaterials,
//	scene->mNumCameras,
//	scene->mNumLights,
//	CountVertices(scene),
//	CountFaces(scene),
//	CountBones(scene),
//	CountAnimChannels(scene),
//	FindPTypes(scene).c_str(),
//	GetAvgFacePerMesh(scene),
//	GetAvgVertsPerMesh(scene),
//	special_points[0][0], special_points[0][1], special_points[0][2],
//	special_points[1][0], special_points[1][1], special_points[1][2],
//	special_points[2][0], special_points[2][1], special_points[2][2]
//)
//;
//
//if (silent)
//{
//	printf("\n");
//	return 0;
//}
//
//// meshes
//if (scene->mNumMeshes) {
//	printf("\nMeshes:  (name) [vertices / bones / faces | primitive_types]\n");
//}
//for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
//	const aiMesh* mesh = scene->mMeshes[i];
//	printf("    %d (%s)", i, mesh->mName.C_Str());
//	printf(
//		": [%d / %d / %d |",
//		mesh->mNumVertices,
//		mesh->mNumBones,
//		mesh->mNumFaces
//	);
//	const unsigned int ptypes = mesh->mPrimitiveTypes;
//	if (ptypes & aiPrimitiveType_POINT) { printf(" point"); }
//	if (ptypes & aiPrimitiveType_LINE) { printf(" line"); }
//	if (ptypes & aiPrimitiveType_TRIANGLE) { printf(" triangle"); }
//	if (ptypes & aiPrimitiveType_POLYGON) { printf(" polygon"); }
//	printf("]\n");
//}
//
//// materials
//unsigned int total = 0;
//for (unsigned int i = 0; i < scene->mNumMaterials; ++i) {
//	aiString name;
//	if (AI_SUCCESS == aiGetMaterialString(scene->mMaterials[i], AI_MATKEY_NAME, &name)) {
//		printf("%s\n    \'%s\'", (total++ ? "" : "\nNamed Materials:"), name.data);
//	}
//}
//if (total) {
//	printf("\n");
//}
//
//// textures
//total = 0;
//for (unsigned int i = 0; i < scene->mNumMaterials; ++i) {
//	aiString name;
//	static const aiTextureType types[] = {
//		aiTextureType_NONE,
//		aiTextureType_DIFFUSE,
//		aiTextureType_SPECULAR,
//		aiTextureType_AMBIENT,
//		aiTextureType_EMISSIVE,
//		aiTextureType_HEIGHT,
//		aiTextureType_NORMALS,
//		aiTextureType_SHININESS,
//		aiTextureType_OPACITY,
//		aiTextureType_DISPLACEMENT,
//		aiTextureType_LIGHTMAP,
//		aiTextureType_REFLECTION,
//		aiTextureType_UNKNOWN
//	};
//	for (unsigned int type = 0; type < sizeof(types) / sizeof(types[0]); ++type) {
//		for (unsigned int idx = 0; AI_SUCCESS == aiGetMaterialString(scene->mMaterials[i],
//			AI_MATKEY_TEXTURE(types[type], idx), &name); ++idx) {
//			printf("%s\n    \'%s\'", (total++ ? "" : "\nTexture Refs:"), name.data);
//		}
//	}
//}
//if (total) {
//	printf("\n");
//}
//
//// animations
//total = 0;
//for (unsigned int i = 0; i < scene->mNumAnimations; ++i) {
//	if (scene->mAnimations[i]->mName.length) {
//		printf("%s\n     \'%s\'", (total++ ? "" : "\nNamed Animations:"), scene->mAnimations[i]->mName.data);
//	}
//}
//if (total) {
//	printf("\n");
//}
//
//// node hierarchy
//printf("\nNode hierarchy:\n");
//PrintHierarchy(scene->mRootNode, "", verbose);
//
//printf("\n");
//
//
//
//
//return 0;