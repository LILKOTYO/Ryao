#ifndef VIEWER_H
#define VIEWER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Camera.h>
#include <Shader.h>
#include <ViewerMesh.h>
#include <Logger.h>
#include <RYAO.h>
#include <vector>

namespace Ryao {

class Viewer {
public:
	Viewer() {}
	~Viewer() {}

	void addViewerMesh(ViewerMesh* vmesh)	{ _viewerMeshList.push_back(vmesh); }
private:
	std::vector<ViewerMesh*> _viewerMeshList;
};

} // Ryao

#endif // !VIEWER_H
