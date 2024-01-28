#include "Forward_Inversekinematics.h"

static float   camera_yaw = 0.0f;
static float   camera_pitch = -40.0f;
static float   camera_distance = 70.0f;

static int     last_mouse_x, last_mouse_y;


Forward_Inversekinematics::Forward_Inversekinematics(QWidget *parent)
    : QOpenGLWidget(parent)
{
    ui.setupUi(this);
}

Forward_Inversekinematics::~Forward_Inversekinematics()
{}


void Forward_Inversekinematics::loadFile(string file_path)
{
    filename = file_path;
    if (bvh)
        delete  bvh;
    bvh = new BVH(filename.c_str());
    if (!bvh->IsLoadSuccess())
    {
        delete  bvh;
        bvh = NULL;
    }

    animation_time = 0.0f;
    frame_no = 0;
}

void Forward_Inversekinematics::initializeGL()
{
    initializeOpenGLFunctions();
}

void Forward_Inversekinematics::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (double)w / h, 1, 500);
}

void Forward_Inversekinematics::paintGL()
{
    glClearColor(2.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -camera_distance);
    glRotatef(-camera_pitch, 1.0, 0.0, 0.0);
    glRotatef(-camera_yaw, 0.0, 1.0, 0.0);


    if (bvh)
    {
        glColor3f(2.0f, 0.3f, 0.3f);

        bvh->RenderFigure(frame_no);

        if (bvh && on_animation)
        {
            animation_time += bvh->GetInterval();
            frame_no = animation_time / bvh->GetInterval();
            frame_no = frame_no % bvh->GetNumFrame();
        }

        update();
    }
}

void Forward_Inversekinematics::updateFK()
{
    bvh->computeFK(frame_no, 1.0f);
    x = bvh->GetJoint(name)->global_coor(0);
    y = bvh->GetJoint(name)->global_coor(1);
    z = bvh->GetJoint(name)->global_coor(2);
    ui.DampX->setValue(x);
    ui.JointX->setValue(x);
    ui.DampY->setValue(y);
    ui.JointY->setValue(y);
    ui.DampZ->setValue(z);
    ui.JointZ->setValue(z);
}

void Forward_Inversekinematics::loadFile(QString file_path)
{
    this->loadFile(file_path.toStdString());

    if (bvh)
        this->updateFK();
    update();
}

void Forward_Inversekinematics::writeFile()
{
    if (bvh) {
        bvh->writeFile();
    }
}

void Forward_Inversekinematics::rootTranslateX(int value)
{
    if (bvh) {
        bvh->SetMotion(frame_no, 0, (double)value);
        this->updateFK();
    }

}

void Forward_Inversekinematics::rootTranslateY(int value)
{
    if (bvh) {
        bvh->SetMotion(frame_no, 1, (double)value);
        this->updateFK();
    }

}

void Forward_Inversekinematics::rootTranslateZ(int value)
{
    if (bvh) {
        bvh->SetMotion(frame_no, 2, (double)value);
        this->updateFK();
    }

}

void Forward_Inversekinematics::getJoint(QString name)
{
    if (bvh) {
        bvh->computeFK(frame_no, 1.0f);
        this->name = name.toStdString();
        this->updateFK();
    }
}

void Forward_Inversekinematics::changePositionX(double value)
{
    if (bvh) {

        bvh->ComputeIK(name, value, y, z, frame_no);
        this->updateFK();

    }
}

void Forward_Inversekinematics::changePositionY(double value)
{
    if (bvh) {

        bvh->ComputeIK(name, x, value, z, frame_no);
        this->updateFK();
    }

}

void Forward_Inversekinematics::changePositionZ(double value)
{
    if (bvh) {

        bvh->ComputeIK(name, x, y, value, frame_no);
        this->updateFK();
    }

}

void Forward_Inversekinematics::changePositionX_Damp(double value)
{
    if (bvh) {

        bvh->ComputeDampIK(name, value, y, z, frame_no);
        this->updateFK();
    }

}

void Forward_Inversekinematics::changePositionY_Damp(double value)
{
    if (bvh) {

        bvh->ComputeDampIK(name, x, value, z, frame_no);
        this->updateFK();
    }

}

void Forward_Inversekinematics::changePositionZ_Damp(double value)
{
    if (bvh) {

        bvh->ComputeDampIK(name, x, y, value, frame_no);
        this->updateFK();
    }
}

void Forward_Inversekinematics::IK2()
{
    if (bvh) {
        bvh->computeFK(frame_no, 1.0f);
        bvh->ComputeIK2("RightFingerBase",
            bvh->GetJoint("RightFingerBase")->global_coor(0),
            bvh->GetJoint("RightFingerBase")->global_coor(1) - 3.0,
            bvh->GetJoint("RightFingerBase")->global_coor(2),
            "LeftFingerBase",
            bvh->GetJoint("LeftFingerBase")->global_coor(0),
            bvh->GetJoint("LeftFingerBase")->global_coor(1) - 3.0,
            bvh->GetJoint("LeftFingerBase")->global_coor(2), frame_no);
        this->updateFK();
    }

}

void Forward_Inversekinematics::addControlIK()
{
    if (bvh) {
        bvh->computeFK(frame_no, 1.0f);
        bvh->addingControlIK("LeftFingerBase",
            bvh->GetJoint("LeftFingerBase")->global_coor(0),
            bvh->GetJoint("LeftFingerBase")->global_coor(1),
            bvh->GetJoint("LeftFingerBase")->global_coor(2), frame_no);
        this->updateFK();
    }
}

void Forward_Inversekinematics::Stop_Start()
{
    on_animation = !on_animation;
    update();
}

void Forward_Inversekinematics::mouseMoveEvent(QMouseEvent* event)
{
    if (Qt::RightButton == (event->buttons() & Qt::RightButton))
    {

        float xoffset = event->pos().x() - last_mouse_x;
        float yoffset = event->pos().y() - last_mouse_y;
        camera_yaw -= (xoffset) * 0.5;
        if (camera_yaw < 0.0)
            camera_yaw += 360.0;
        else if (camera_yaw > 360.0)
            camera_yaw -= 360.0;

        camera_pitch -= (yoffset) * 0.5;
        if (camera_pitch < -90.0)
            camera_pitch = -90.0;
        else if (camera_pitch > 90.0)
            camera_pitch = 90.0;
        last_mouse_x = event->pos().x();
        last_mouse_y = event->pos().y();
        update();
    }
}

void Forward_Inversekinematics::mousePressEvent(QMouseEvent* event)
{
    switch (event->button()) {
    case Qt::RightButton:
    {
        last_mouse_x = event->pos().x();
        last_mouse_y = event->pos().y();
    }
    break;
    default:
        break;
    }

}