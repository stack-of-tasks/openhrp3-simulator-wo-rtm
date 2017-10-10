/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */
/**
   @author Shin'ichiro Nakaoka
*/

#ifndef HRPGEPCOLLISION_COLDET_MODEL_H_INCLUDED
#define HRPGEPCOLLISION_COLDET_MODEL_H_INCLUDED

#include "config.h"
#include <string>
#include <hrpGepUtil/Eigen3d.h>
#include <hrpGepUtil/Referenced.h>
#include <vector>
#include <map>


namespace IceMaths {
  class Matrix4x4;
}

namespace hrpGep {

  class ColdetModelSharedDataSet;
  class Edge
  {
    int vertex[2];
  public :
    Edge(int vertexIndex1, int vertexIndex2){
      if(vertexIndex1 < vertexIndex2){
	vertex[0] = vertexIndex1;
	vertex[1] = vertexIndex2;
      } else {
	vertex[0] = vertexIndex2;
	vertex[1] = vertexIndex1;
      }
    }
    bool operator<(const Edge& rhs) const {
      if(vertex[0] < rhs.vertex[0]){
	return true;
      } else if(vertex[0] == rhs.vertex[0]){
	return (vertex[1] < rhs.vertex[1]);
      } else {
	return false;
      }
    }
  };

  struct trianglePair {
    int t[2];
  };

  typedef std::map< Edge, trianglePair > EdgeToTriangleMap;

  class HRP_COLLISION_EXPORT ColdetModel : public Referenced
  {
  public:
    enum PrimitiveType { SP_MESH, SP_BOX, SP_CYLINDER, SP_CONE, SP_SPHERE, SP_PLANE };

    /**
     * @brief constructor
     */
    ColdetModel();

    /**
     * @brief copy constructor
     *
     * Shape information stored in dataSet is shared with org
     */
    ColdetModel(const ColdetModel& org);

    /**
     * @brief destructor
     */
    virtual ~ColdetModel();

    /**
     * @brief set name of this model
     * @param name name of this model 
     */
    void setName(const std::string& name) { name_ = name; }

    /**
     * @brief get name of this model
     * @return name name of this model 
     */
    const std::string& name() const { return name_; }

    /**
     * @brief set the number of vertices
     * @param n the number of vertices
     */
    void setNumVertices(int n);

    /**
     * @brief get the number of vertices
     * @return the number of vertices
     */
    int getNumVertices() const;

    /**
     * @brief set the number of triangles
     * @param n the number of triangles
     */
    void setNumTriangles(int n);

    int getNumTriangles() const;

    /**
     * @brief add a vertex
     * @param index index of the vertex
     * @param x x position of the vertex
     * @param y y position of the vertex
     * @param z z position of the vertex
     */
    void setVertex(int index, float x, float y, float z);

    /**
       add a vertex to the end of the vector
    */
    void addVertex(float x, float y, float z);

    /**
     * @brief get a vertex
     * @param index index of the vertex
     * @param out_x x position of the vertex
     * @param out_y y position of the vertex
     * @param out_z z position of the vertex
     */
    void getVertex(int index, float& out_x, float& out_y, float& out_z) const;

    /**
     * @brief add a triangle
     * @param index index of the triangle
     * @param v1 index of the first vertex
     * @param v2 index of the second vertex
     * @param v3 index of the third vertex
     */
    void setTriangle(int index, int v1, int v2, int v3);

    /**
       add a triangle to the end of the vector
    */
    void addTriangle(int v1, int v2, int v3);

    void getTriangle(int index, int& out_v1, int& out_v2, int& out_v3) const;

    /**
     * @brief build tree of bounding boxes to accelerate collision check
     *
     * This method must be called before doing collision check
     */
    void build();

    /**
     * @brief check if build() is already called or not
     * @return true if build() is already called, false otherwise
     */
    bool isValid() const { return isValid_; }

    /**
     * @brief set position and orientation of this model
     * @param R new orientation 
     * @param p new position
     */
    void setPosition(const Matrix33& R, const Vector3& p);

    /**
     * @brief set position and orientation of this model
     * @param R new orientation (length = 9)  
     * @param p new position (length = 3)
     */
    void setPosition(const double* R, const double* p);

    /**
     * @brief set primitive type
     * @param ptype primitive type
     */
    void setPrimitiveType(PrimitiveType ptype);

    /**
     * @brief get primitive type
     * @return primitive type
     */
    PrimitiveType getPrimitiveType() const;

    /**
     * @brief set the number of parameters of primitive
     * @param nparam the number of parameters of primitive
     */
    void setNumPrimitiveParams(unsigned int nparam);

    /**
     * @brief set a parameter of primitive
     * @param index index of the parameter
     * @param value value of the parameter
     * @return true if the parameter is set successfully, false otherwise
     */
    bool setPrimitiveParam(unsigned int index, float value);

    /**
     * @brief get a parameter of primitive
     * @param index index of the parameter
     * @param value value of the parameter
     * @return true if the parameter is gotten successfully, false otherwise
     */
    bool getPrimitiveParam(unsigned int index, float &value) const;

    /**
     * @brief set position and orientation of primitive
     * @param R orientation relative to link (length = 9)  
     * @param p position relative to link (length = 3)
     */
    void setPrimitivePosition(const double* R, const double* p);
        
    /**
     * @brief compute distance between a point and this mesh along ray
     * @param point a point
     * @param dir direction of ray
     * @return distance if ray collides with this mesh, FLT_MAX otherwise
     */
    double computeDistanceWithRay(const double *point, const double *dir);

    /**
     * @brief check collision between this triangle mesh and a point cloud
     * @param i_cloud points
     * @param i_radius radius of spheres assigned to the points
     * @return true if colliding, false otherwise
     */
    bool checkCollisionWithPointCloud(const std::vector<Vector3> &i_cloud,
				      double i_radius);

    void getBoundingBoxData(const int depth, std::vector<Vector3>& out_boxes);
        
    int getAABBTreeDepth();
    int getAABBmaxNum();
    int numofBBtoDepth(int minNumofBB);

    ColdetModelSharedDataSet *getDataSet() { return dataSet; }
  private:
    /**
     * @brief common part of constuctors
     */
    void initialize();
    void setNeighborTriangle(int triangle, int vertex0, int vertex1, int vertex2);
    void initNeighbor(int n);

        
    ColdetModelSharedDataSet* dataSet;
    IceMaths::Matrix4x4* transform;
    IceMaths::Matrix4x4* pTransform; ///< transform of primitive
    std::string name_;
    bool isValid_;
    EdgeToTriangleMap triangleMap;

    bool setNeighborTriangleSub(int triangle, int vertex0, int vertex1);
    void setNeighbor(int triangle0, int triangle1);

    friend class ColdetModelPair;
  };

  typedef boost::intrusive_ptr<ColdetModel> ColdetModelPtr;
}


#endif
