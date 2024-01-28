#ifndef  _BVH_H_
#define  _BVH_H_

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <iomanip>
#include <Eigen/core>
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <GL/glut.h>

using namespace  std;

class  BVH
{
public:

	enum  ChannelEnum
	{
		X_ROTATION, Y_ROTATION, Z_ROTATION,
		X_POSITION, Y_POSITION, Z_POSITION
	};
	struct  Joint;

	struct  Channel
	{
		Joint* joint;

		ChannelEnum          type;

		int                  index;
	};

	struct  Joint
	{
		string               name;
		int                  index;

		Joint* parent;
		vector< Joint* >    children;

		double               offset[3];

		bool                 has_site;
		double               site[3];

		vector< Channel* >  channels;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector3d global_coor;
		Eigen::Matrix<double, 4, 4> TR;

		int				     write_mark;
		bool                 isWritten;
	};


private:
	bool                     is_load_success;

	string                   file_name;
	string                   motion_name;

	int                      num_channel;
	vector< Channel* >      channels;
	vector< Joint* >        joints;
	map< string, Joint* >   joint_index;

	int                      num_frame;
	double                   interval;
	double* motion;


public:
	BVH();
	BVH(const char* bvh_file_name);
	~BVH();

	void  Clear();
	void  Load(const char* bvh_file_name);

public:

	bool  IsLoadSuccess() const { return is_load_success; }

	const string& GetFileName() const { return file_name; }
	const string& GetMotionName() const { return motion_name; }

	const int       GetNumJoint() const { return  joints.size(); }
	const Joint* GetJoint(int no) const { return  joints[no]; }
	const int       GetNumChannel() const { return  channels.size(); }
	const Channel* GetChannel(int no) const { return  channels[no]; }

	const Joint* GetJoint(const string& j) const {
		map< string, Joint* >::const_iterator  i = joint_index.find(j);
		return  (i != joint_index.end()) ? (*i).second : NULL;
	}
	const Joint* GetJoint(const char* j) const {
		map< string, Joint* >::const_iterator  i = joint_index.find(j);
		return  (i != joint_index.end()) ? (*i).second : NULL;
	}

	int     GetNumFrame() const { return  num_frame; }
	double  GetInterval() const { return  interval; }
	double  GetMotion(int f, int c) const { return  motion[f * num_channel + c]; }

	void  SetMotion(int f, int c, double v) { motion[f * num_channel + c] = v; }

public:

	void  RenderFigure(int frame_no, float scale = 1.0f);

	static void  RenderFigure(const Joint* root, const double* data, float scale = 1.0f);

	static void  RenderBone(float x0, float y0, float z0, float x1, float y1, float z1);

	//used to do FK
	void computeFK(int frame_no, double scale);

	void ComputeIK(string joint_name, double x, double y, double z, int frame_no);
	void ComputeDampIK(string joint_name, double x, double y, double z, int frame_no);
	void ComputeIK2(string joint_name1, double x1, double y1, double z1, string joint_name2, double x2, double y2, double z2, int frame_no);
	void addingControlIK(string joint_name, double x, double y, double z, int frame_no);

	//used to write BVH file
	void writeFile();
};



#endif // _BVH_H_
