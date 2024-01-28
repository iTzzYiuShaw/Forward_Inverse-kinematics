#include "BVH.h"

#define pi 3.1415926


BVH::BVH()
{
	motion = NULL;
	Clear();
}

BVH::BVH(const char* bvh_file_name)
{
	motion = NULL;
	Clear();

	Load(bvh_file_name);
}

BVH::~BVH()
{
	Clear();
}


void  BVH::Clear()
{
	int  i;
	for (i = 0; i < channels.size(); i++)
		delete  channels[i];
	for (i = 0; i < joints.size(); i++)
		delete  joints[i];
	if (motion != NULL)
		delete  motion;

	is_load_success = false;

	file_name = "";
	motion_name = "";

	num_channel = 0;
	channels.clear();
	joints.clear();
	joint_index.clear();

	num_frame = 0;
	interval = 0.0;
	motion = NULL;
}



void  BVH::Load(const char* bvh_file_name)
{
#define  BUFFER_LENGTH  1024*32

	ifstream  file;					//Files name
	char      line[BUFFER_LENGTH];//按行读入
	char* token;				//被分割后的信息存放在token里
	char      separater[] = " :,\t";//用来分割的字符
	vector< Joint* >   joint_stack;//joint临时存放的栈
	Joint* joint = NULL;			//上一个读到的joint
	Joint* new_joint = NULL;		//当前读到的joint
	bool      is_site = false;
	double    x, y, z;
	int       i, j;

	Clear();

	//设置文件名file_name和动作名motion_name（文件名去掉后缀）
	file_name = bvh_file_name;
	const char* mn_first = bvh_file_name;
	const char* mn_last = bvh_file_name + strlen(bvh_file_name);
	if (strrchr(bvh_file_name, '\\') != NULL)
		mn_first = strrchr(bvh_file_name, '\\') + 1;
	else if (strrchr(bvh_file_name, '/') != NULL)
		mn_first = strrchr(bvh_file_name, '/') + 1;
	if (strrchr(bvh_file_name, '.') != NULL)
		mn_last = strrchr(bvh_file_name, '.');
	if (mn_last < mn_first)
		mn_last = bvh_file_name + strlen(bvh_file_name);
	motion_name.assign(mn_first, mn_last);

	//打开文件，读入
	file.open(bvh_file_name, ios::in);
	if (file.is_open() == 0)  return;//没打开文件，退出函数

	while (!file.eof())//如果没有异常（打开成功）
	{
		if (file.eof())
			goto bvh_error;

		file.getline(line, BUFFER_LENGTH);//按行读入 存储到line
		token = strtok(line, separater);//按字符":"和"\t"分割，被分割后的信息存放在token里

		if (token == NULL)  continue;//没有数据 读下一行


		if ((strcmp(token, "ROOT") == 0) ||
			(strcmp(token, "JOINT") == 0))//读到单词ROOT或者JOINT说明开始有一个新的joint
		{
			new_joint = new Joint();//创建一个新的joint
			new_joint->index = joints.size();//编号取决于当前joints数组里的个数
			new_joint->parent = joint;//父节点是上一个读取的joint，如果是ROOT就没有
			new_joint->has_site = false;//暂且定当前的joint不是末端
			new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
			new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;//先初始化以上三个属性

			new_joint->write_mark = 0;
			new_joint->isWritten = false;
			new_joint->global_coor(0) = 0.0f;
			new_joint->global_coor(1) = 0.0f;
			new_joint->global_coor(2) = 0.0f;

			joints.push_back(new_joint);//创建好后将新的joint存入数组joints
			if (joint)//如果存在上一个joint，那新的joint就是上一个joint的子节点
				joint->children.push_back(new_joint);

			token = strtok(NULL, "");
			while (*token == ' ')  token++;
			new_joint->name = token;

			joint_index[new_joint->name] = new_joint;//添加joint名字的映射关系
			continue;
		}

		if (strcmp(token, "{") == 0)//读到"{"说明有一个新的joint，把上一个读到的joint入栈
		{
			joint_stack.push_back(joint);
			joint = new_joint;
			continue;
		}
		if (strcmp(token, "}") == 0)//读到"}"说明当前joint读取完毕，这个joint不是末端，出栈
		{
			joint = joint_stack.back();
			joint_stack.pop_back();
			is_site = false;
			continue;
		}

		if ((strcmp(token, "End") == 0))//读到单词End说明当前joint是末端，开始设置末端偏移参数
		{
			new_joint = joint;
			is_site = true;
			continue;
		}

		if (strcmp(token, "OFFSET") == 0)//读到offset，设置偏移参数
		{
			token = strtok(NULL, separater);
			x = token ? atof(token) : 0.0;
			token = strtok(NULL, separater);
			y = token ? atof(token) : 0.0;
			token = strtok(NULL, separater);
			z = token ? atof(token) : 0.0;

			if (is_site)//这个值为true表示现在读到的是末端的偏移参数
			{
				joint->has_site = true;
				joint->site[0] = x;
				joint->site[1] = y;
				joint->site[2] = z;
			}
			else//否则就是joint的偏移参数
			{
				joint->offset[0] = x;
				joint->offset[1] = y;
				joint->offset[2] = z;
			}
			continue;
		}

		if (strcmp(token, "CHANNELS") == 0)//读到单词CHANNELS开始设置channel的信息
		{
			token = strtok(NULL, separater);
			joint->channels.resize(token ? atoi(token) : 0);

			for (i = 0; i < joint->channels.size(); i++)
			{
				Channel* channel = new Channel();
				channel->joint = joint;
				channel->index = channels.size();
				channels.push_back(channel);
				joint->channels[i] = channel;

				token = strtok(NULL, separater);
				if (strcmp(token, "Xrotation") == 0)
					channel->type = X_ROTATION;
				else if (strcmp(token, "Yrotation") == 0)
					channel->type = Y_ROTATION;
				else if (strcmp(token, "Zrotation") == 0)
					channel->type = Z_ROTATION;
				else if (strcmp(token, "Xposition") == 0)
					channel->type = X_POSITION;
				else if (strcmp(token, "Yposition") == 0)
					channel->type = Y_POSITION;
				else if (strcmp(token, "Zposition") == 0)
					channel->type = Z_POSITION;
			}
		}

		if (strcmp(token, "MOTION") == 0)//读到单词MOTION说明骨骼层次信息读取完毕，跳出
			break;
	}

	//读Frames行
	file.getline(line, BUFFER_LENGTH);
	token = strtok(line, separater);
	if (strcmp(token, "Frames") != 0)
		goto bvh_error;
	token = strtok(NULL, separater);
	if (token == NULL)
		goto bvh_error;
	num_frame = atoi(token);

	//读Frame time行
	file.getline(line, BUFFER_LENGTH);
	token = strtok(line, ":");
	if (strcmp(token, "Frame Time") != 0)
		goto bvh_error;
	token = strtok(NULL, separater);
	if (token == NULL)
		goto bvh_error;
	interval = atof(token);

	//设置motion数组的信息，存放运动数据，数组大小是frame数 * channel数
	num_channel = channels.size();
	motion = new double[num_frame * num_channel];

	for (i = 0; i < num_frame; i++)
	{
		file.getline(line, BUFFER_LENGTH);
		token = strtok(line, separater);
		for (j = 0; j < num_channel; j++)
		{
			if (token == NULL)
				goto bvh_error;
			motion[i * num_channel + j] = atof(token);
			token = strtok(NULL, separater);
		}
	}

	file.close();

	is_load_success = true;

	return;

bvh_error:
	file.close();
}


void  BVH::RenderFigure(int frame_no, float scale)
{
	RenderFigure(joints[0], motion + frame_no * num_channel, scale);
}


void  BVH::RenderFigure(const Joint* joint, const double* data, float scale)
{
	glPushMatrix();

	if (joint->parent == NULL)
	{
		glTranslatef(data[0] * scale, data[1] * scale, data[2] * scale);
	}
	else
	{
		glTranslatef(joint->offset[0] * scale, joint->offset[1] * scale, joint->offset[2] * scale);
	}

	int  i, j;
	for (i = 0; i < joint->channels.size(); i++)
	{
		Channel* channel = joint->channels[i];
		if (channel->type == X_ROTATION)
			glRotatef(data[channel->index], 1.0f, 0.0f, 0.0f);
		else if (channel->type == Y_ROTATION)
			glRotatef(data[channel->index], 0.0f, 1.0f, 0.0f);
		else if (channel->type == Z_ROTATION)
			glRotatef(data[channel->index], 0.0f, 0.0f, 1.0f);
	}

	if (joint->children.size() == 0)
	{
		RenderBone(0.0f, 0.0f, 0.0f, joint->site[0] * scale, joint->site[1] * scale, joint->site[2] * scale);
	}
	if (joint->children.size() == 1)
	{
		Joint* child = joint->children[0];
		RenderBone(0.0f, 0.0f, 0.0f, child->offset[0] * scale, child->offset[1] * scale, child->offset[2] * scale);
	}
	if (joint->children.size() > 1)
	{
		float  center[3] = { 0.0f, 0.0f, 0.0f };
		for (i = 0; i < joint->children.size(); i++)
		{
			Joint* child = joint->children[i];
			center[0] += child->offset[0];
			center[1] += child->offset[1];
			center[2] += child->offset[2];
		}
		center[0] /= joint->children.size() + 1;
		center[1] /= joint->children.size() + 1;
		center[2] /= joint->children.size() + 1;

		RenderBone(0.0f, 0.0f, 0.0f, center[0] * scale, center[1] * scale, center[2] * scale);

		for (i = 0; i < joint->children.size(); i++)
		{
			Joint* child = joint->children[i];
			RenderBone(center[0] * scale, center[1] * scale, center[2] * scale,
				child->offset[0] * scale, child->offset[1] * scale, child->offset[2] * scale);
		}
	}

	for (i = 0; i < joint->children.size(); i++)
	{
		RenderFigure(joint->children[i], data, scale);
	}

	glPopMatrix();
}


void  BVH::RenderBone(float x0, float y0, float z0, float x1, float y1, float z1)
{

	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);

	static GLUquadricObj* quad_obj = NULL;
	if (quad_obj == NULL)
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle(quad_obj, GLU_FILL);
	gluQuadricNormals(quad_obj, GLU_SMOOTH);

	glPushMatrix();

	glTranslated(x0, y0, z0);


	double  length;
	length = sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
	if (length < 0.0001) {
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;

	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;

	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;

	length = sqrt(side_x * side_x + side_y * side_y + side_z * side_z);
	if (length < 0.0001) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;

	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
						up_x,   up_y,   up_z,   0.0,
						dir_x,  dir_y,  dir_z,  0.0,
						0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd(m);

	GLdouble radius = 0.4;
	GLdouble slices = 8.0;
	GLdouble stack = 3.0;

	gluCylinder(quad_obj, radius, radius, bone_length, slices, stack);

	glPopMatrix();
}

Eigen::Matrix<double, 4, 4> getRotationX(double angle) {

	Eigen::Matrix<double, 4, 4> rm;
	angle = angle * pi / 180.0;

	rm << 1.0, 0.0, 0.0, 0.0,
		0.0, cos(angle), -sin(angle), 0.0,
		0.0, sin(angle), cos(angle), 0.0,
		0.0, 0.0, 0.0, 1.0;

	return rm;
}
Eigen::Matrix<double, 4, 4> getRotationY(double angle) {

	Eigen::Matrix<double, 4, 4> rm;
	angle = angle * pi / 180.0;

	rm << cos(angle), 0.0, sin(angle), 0.0,
		0.0, 1.0, 0.0, 0.0,
		-sin(angle), 0.0, cos(angle), 0.0,
		0.0, 0.0, 0.0, 1.0;

	return rm;
}
Eigen::Matrix<double, 4, 4> getRotationZ(double angle) {

	Eigen::Matrix<double, 4, 4> rm;
	angle = angle * pi / 180.0;

	rm << cos(angle), -sin(angle), 0.0, 0.0,
		sin(angle), cos(angle), 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0;

	return rm;
}
Eigen::Matrix<double, 4, 4> getTranslation(double* offset) {

	Eigen::Matrix<double, 4, 4> tm;

	tm << 1.0, 0.0, 0.0, offset[0],
		0.0, 1.0, 0.0, offset[1],
		0.0, 0.0, 1.0, offset[2],
		0.0, 0.0, 0.0, 1.0;

	return tm;
}


void BVH::computeFK(int frame_no, double scale) {

	//use DFS to update global corrdinates
	int count = 0;
	Joint* myJoint = joints[0];
	double global_offsetX = motion[frame_no * num_channel];
	double global_offsetY = motion[frame_no * num_channel + 1];
	double global_offsetZ = motion[frame_no * num_channel + 2];
	while (count != joints.size() || myJoint->index != 0) {

		if (!myJoint->isWritten) {

			Eigen::Vector4d homo4(myJoint->offset[0], myJoint->offset[1], myJoint->offset[2], 1);

			if (myJoint->parent == NULL) {
				myJoint->global_coor(0) = global_offsetX;
				myJoint->global_coor(1) = global_offsetY;
				myJoint->global_coor(2) = global_offsetZ;
				myJoint->TR = getTranslation(myJoint->offset) * getRotationZ(motion[3]) * getRotationY(motion[4]) * getRotationX(motion[5]);
			}
			else {
				myJoint->TR = myJoint->parent->TR *
					getTranslation(myJoint->offset) *
					getRotationZ(motion[myJoint->index * 3 + 3]) *
					getRotationY(motion[myJoint->index * 3 + 4]) *
					getRotationX(motion[myJoint->index * 3 + 5]);

				Eigen::Vector4d new_homo4 = myJoint->TR * homo4;

				myJoint->global_coor(0) = new_homo4(0) + global_offsetX;
				myJoint->global_coor(1) = new_homo4(1) + global_offsetY;
				myJoint->global_coor(2) = new_homo4(2) + global_offsetZ;
			}

			count++;
			myJoint->isWritten = true;
		}

		if (myJoint->write_mark != myJoint->children.size()) {

			myJoint = myJoint->children[myJoint->write_mark];
		}
		else {
			myJoint = myJoint->parent;
			myJoint->write_mark++;
		}
	}

	for (size_t i = 0; i < joints.size(); i++)
	{
		joints[i]->isWritten = false;
		joints[i]->write_mark = 0;
	}
}

void BVH::ComputeIK(string joint_name, double x, double y, double z, int frame_no)
{
	int count = 0;
	double miss = INFINITY;
	while (count < 1000 && miss > 0.01) {
		computeFK(frame_no, 1.0f);

		Eigen::Vector3d x1 = GetJoint(joint_name)->global_coor;
		Eigen::Vector3d x2(x, y, z);
		
		Eigen::VectorXd theta1(num_channel - 3);
		Eigen::VectorXd theta2(num_channel - 3);
		Eigen::VectorXd thetaDot(num_channel - 3);
		for (size_t i = 0; i < theta1.size(); i++)
		{
			theta1(i) = motion[frame_no * num_channel + 3 + i];
		}

		Eigen::MatrixXd Jacobian(3, num_channel - 3);
		Eigen::Vector3d axis[3];
		Eigen::Vector3d xAxis(1.0, 0.0, 0.0);
		Eigen::Vector3d yAxis(0.0, 1.0, 0.0);
		Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
		axis[0] = zAxis;
		axis[1] = yAxis;
		axis[2] = xAxis;
		for (size_t i = 0; i < Jacobian.cols(); i++)
		{
			Eigen::Vector3d temp = axis[i % 3].cross(x1 - joints[i / 3]->global_coor);
			Jacobian.col(i) = temp;
		}

		thetaDot = Jacobian.transpose() * (Jacobian * Jacobian.transpose()).inverse() * (x2 - x1);

		theta2 = thetaDot + theta1;

		for (size_t i = 0; i < theta2.size(); i++)
		{
			SetMotion(frame_no, i + 3, theta2(i));
		}

		miss = (x2 - x1).norm();
		count++;
	}
}

void BVH::ComputeDampIK(string joint_name, double x, double y, double z, int frame_no)
{
	int count = 0;
	double miss = INFINITY;
	double lamda = 1.0;
	Eigen::Matrix3d I;
	while (count < 1000 && miss > 0.01) {

		computeFK(frame_no, 1.0f);

		Eigen::Vector3d x1 = GetJoint(joint_name)->global_coor;
		Eigen::Vector3d x2(x, y, z);

		Eigen::VectorXd theta1(num_channel - 3);
		Eigen::VectorXd theta2(num_channel - 3);
		Eigen::VectorXd thetaDot(num_channel - 3);
		for (size_t i = 0; i < theta1.size(); i++)
		{
			theta1(i) = motion[frame_no * num_channel + 3 + i];
		}

		Eigen::MatrixXd Jacobian(3, num_channel - 3);
		Eigen::Vector3d axis[3];
		Eigen::Vector3d xAxis(1.0, 0.0, 0.0);
		Eigen::Vector3d yAxis(0.0, 1.0, 0.0);
		Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
		axis[0] = zAxis;
		axis[1] = yAxis;
		axis[2] = xAxis;
		for (size_t i = 0; i < Jacobian.cols(); i++)
		{
			Eigen::Vector3d temp = axis[i % 3].cross(x1 - joints[i / 3]->global_coor);
			Jacobian.col(i) = temp;
		}

		thetaDot = Jacobian.transpose() * ((Jacobian * Jacobian.transpose()) + (pow(lamda, 2) * I.setIdentity())).inverse() * (x2 - x1);

		theta2 = thetaDot + theta1;

		for (size_t i = 0; i < theta2.size(); i++)
		{
			SetMotion(frame_no, i + 3, theta2(i));
		}

		miss = (x2 - x1).norm();
		count++;
	}
	
}

//This is for task4 and I only picked two joints so that it is easy operating 
//The vector changes from 3 x 1 to 6 x 1
void BVH::ComputeIK2(string joint_name1, double x1, double y1, double z1, string joint_name2, double x2, double y2, double z2, int frame_no)
{
	double miss = INFINITY;
	int count = 0;
	double lamda = 0.5;
	Eigen::MatrixXd I(6,6);
	while (count < 500 && miss > 0.01)
	{
		computeFK(frame_no, 1.0f);

		Eigen::VectorXd V1(6);
		for (size_t i = 0; i < 3; i++)
		{
			V1(i) = GetJoint(joint_name1)->global_coor(i);
		}
		for (size_t i = 0; i < 3; i++)
		{
			V1(i + 3) = GetJoint(joint_name2)->global_coor(i);
		}
		Eigen::VectorXd V2(6);
		V2(0) = x1; V2(1) = y1; V2(2) = z1;
		V2(3) = x2; V2(4) = y2; V2(5) = z2;

		Eigen::Vector3d v1 = GetJoint(joint_name1)->global_coor;
		Eigen::Vector3d v2 = GetJoint(joint_name2)->global_coor;

		Eigen::VectorXd theta1(num_channel - 3);
		Eigen::VectorXd theta2(num_channel - 3);
		Eigen::VectorXd thetaDot(num_channel - 3);
		for (size_t i = 0; i < theta1.size(); i++)
		{
			theta1(i) = motion[frame_no * num_channel + 3 + i];
		}

		Eigen::MatrixXd Jacobian(6, num_channel - 3);

		Eigen::MatrixXd Jacobian1(3, num_channel - 3);
		Eigen::MatrixXd Jacobian2(3, num_channel - 3);
		Eigen::Vector3d axis[3];
		Eigen::Vector3d xAxis(1.0, 0.0, 0.0);
		Eigen::Vector3d yAxis(0.0, 1.0, 0.0);
		Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
		axis[0] = zAxis;
		axis[1] = yAxis;
		axis[2] = xAxis;
		for (size_t i = 0; i < Jacobian1.cols(); i++)
		{
			Eigen::Vector3d temp = axis[i % 3].cross(v1 - joints[i / 3]->global_coor);
			Jacobian1.col(i) = temp;
		}
		for (size_t i = 0; i < Jacobian2.cols(); i++)
		{
			Eigen::Vector3d temp = axis[i % 3].cross(v2 - joints[i / 3]->global_coor);
			Jacobian2.col(i) = temp;
		}
		for (size_t i = 0; i < Jacobian.cols(); i++)
		{
			Eigen::VectorXd temp(6);
			temp(0) = Jacobian1(0, i); temp(1) = Jacobian1(1, i); temp(2) = Jacobian1(2, i);
			temp(3) = Jacobian2(0, i); temp(4) = Jacobian2(1, i); temp(5) = Jacobian2(2, i);
			Jacobian.col(i) = temp;
		}

		thetaDot = Jacobian.transpose() * ((Jacobian * Jacobian.transpose()) + (pow(lamda, 2) * I.setIdentity())).inverse() * (V2 - V1);

		theta2 = thetaDot + theta1;

		for (size_t i = 0; i < theta2.size(); i++)
		{
			SetMotion(frame_no, i + 3, theta2(i));
		}

		miss = (V2 - V1).norm();
		count++;
	}

}

//It has some problem
void BVH::addingControlIK(string joint_name, double x, double y, double z, int frame_no) {
	int count = 0;
	double miss = INFINITY;
	double z_focus = 0.8;
	double z_ignore = 0.2;
	double lamda = 0.5;
	Eigen::Matrix3d I;
	while (count < 500 && miss > 0.01) {
		computeFK(frame_no, 1.0f);

		Eigen::VectorXd gain(num_channel - 3);
		Eigen::VectorXd Z(num_channel - 3);

		Eigen::Vector3d x1 = GetJoint(joint_name)->global_coor;
		Eigen::Vector3d x2(x, y, z);

		Eigen::VectorXd theta1(num_channel - 3);
		Eigen::VectorXd theta_desired(num_channel - 3);
		Eigen::VectorXd theta2(num_channel - 3);
		Eigen::VectorXd thetaDot(num_channel - 3);

		for (size_t i = 0; i < theta1.size(); i++)
		{
			theta1(i) = motion[frame_no * num_channel + 3 + i];
		}

		theta_desired = theta1;
		theta_desired(GetJoint("LeftArm")->index * 3) = 90.0;
		//theta_desired(GetJoint("LeftArm")->index * 3 + 1) = 90.0;
		//theta_desired(GetJoint("LeftArm")->index * 3 + 2) = 90.0;

		for (size_t i = 0; i < gain.size(); i++)
		{
			gain(i) = z_ignore;
		}
		gain(GetJoint("LeftArm")->index * 3) = z_focus;
		gain(GetJoint("LeftArm")->index * 3 + 1) = z_focus;
		gain(GetJoint("LeftArm")->index * 3 + 2) = z_focus;
		
		for (size_t i = 0; i < Z.size(); i++)
		{
			Z(i) = gain(i) * pow((theta1(i) * pi / 180.0) - (theta_desired(i) * pi / 180.0), 2);
		}

		Eigen::MatrixXd Jacobian(3, num_channel - 3);
		Eigen::Vector3d axis[3];
		Eigen::Vector3d xAxis(1.0, 0.0, 0.0);
		Eigen::Vector3d yAxis(0.0, 1.0, 0.0);
		Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
		axis[0] = zAxis;
		axis[1] = yAxis;
		axis[2] = xAxis;
		for (size_t i = 0; i < Jacobian.cols(); i++)
		{
			Eigen::Vector3d temp = axis[i % 3].cross(x1 - joints[i / 3]->global_coor);
			Jacobian.col(i) = temp;
		}

		thetaDot = (Jacobian.transpose() * (((Jacobian * Jacobian.transpose() + (pow(lamda, 2) * I.setIdentity())).inverse()) * ((x2 - x1) + (Jacobian * Z)))) - Z;

		theta2 = thetaDot + theta1;

		for (size_t i = 0; i < theta2.size(); i++)
		{
			SetMotion(frame_no, i + 3, theta2(i));
		}

		miss = (x2 - x1).norm();
		count++;
	}
}

string getChannelName(int num)
{
	switch (num)
	{
	default:
		return string();
	case 0:
		return "Xrotation";
	case 1:
		return "Yrotation";
	case 2:
		return "Zrotation";
	case 3:
		return "Xposition";
	case 4:
		return "Yposition";
	case 5:
		return "Zposition";
	}
}

void BVH::writeFile()
{
	string outfile_path = "../writeoutput/" + GetMotionName() + ".bvh";
	ofstream outfile;
	outfile.open(outfile_path);

	int count = 0;
	int depth = 0;
	Joint* myJoint = joints[0];
	outfile << "HIERARHY" << endl;
	while (true) {

		//写joint头信息
		if (!myJoint->isWritten) {
			for (size_t i = 0; i < depth; i++)
			{
				outfile << "\t";
			}
			if (myJoint->channels.size() == 6)
				outfile << "ROOT " << myJoint->name << endl;
			else
				outfile << "JOINT " << myJoint->name << endl;

			for (size_t i = 0; i < depth; i++)
			{
				outfile << "\t";
			}
			outfile << "{" << endl;

			for (size_t i = 0; i < depth + 1; i++)
			{
				outfile << "\t";
			}
			outfile << "OFFSET " + to_string(myJoint->offset[0]) + " " + to_string(myJoint->offset[1]) + " " + to_string(myJoint->offset[2]) << endl;

			for (size_t i = 0; i < depth + 1; i++)
			{
				outfile << "\t";
			}
			outfile << "CHANNELS " + to_string(myJoint->channels.size()) + " ";
			for (size_t i = 0; i < myJoint->channels.size() - 1; i++)
			{
				outfile << getChannelName(myJoint->channels[i]->type) << " ";
			}
			outfile << getChannelName(myJoint->channels[myJoint->channels.size() - 1]->type) << endl;

			count++;
			myJoint->isWritten = true;
		}

		if (count == joints.size() && myJoint->index == 0)
			break;

		if (!myJoint->has_site && myJoint->write_mark != myJoint->children.size()) {

			myJoint = myJoint->children[myJoint->write_mark];
			depth++;
		}

		else if (myJoint->has_site) {

			//写endsite信息
			for (size_t i = 0; i < depth + 1; i++)
			{
				outfile << "\t";
			}
			outfile << "End Site" << endl;

			for (size_t i = 0; i < depth + 1; i++)
			{
				outfile << "\t";
			}
			outfile << "{" << endl;

			for (size_t i = 0; i < depth + 2; i++)
			{
				outfile << "\t";
			}
			outfile << "OFFSET " + to_string(myJoint->site[0]) + " " + to_string(myJoint->site[1]) + " " + to_string(myJoint->site[2]) << endl;


			for (size_t i = 0; i < depth + 1; i++)
			{
				outfile << "\t";
			}
			outfile << "}" << endl;

			for (size_t i = 0; i < depth; i++)
			{
				outfile << "\t";
			}
			outfile << "}" << endl;
			myJoint = myJoint->parent;
			myJoint->write_mark++;
			depth--;
		}
		else {

			for (size_t i = 0; i < depth; i++)
			{
				outfile << "\t";
			}
			outfile << "}" << endl;
			myJoint = myJoint->parent;
			myJoint->write_mark++;
			depth--;
		}
	}
	outfile << "}" << endl;

	outfile << "MOTION" << endl;
	outfile << "Frames: " << num_frame << endl;
	outfile << "Frame Time: " << interval << endl;
	for (size_t i = 0; i < num_frame; i++)
	{
		for (size_t j = 0; j < num_channel; j++)
		{
			outfile << setprecision(8) << GetMotion(i, j) << " ";
		}
		outfile << "\n";
	}

	for (size_t i = 0; i < joints.size(); i++)
	{
		joints[i]->isWritten = false;
		joints[i]->write_mark = 0;
	}
}

