#pragma once

#include <QtWidgets/QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Compatibility>
#include <QMouseEvent>

#include "BVH.h"
#include "ui_Forward_Inversekinematics.h"

class Forward_Inversekinematics : public QOpenGLWidget, QOpenGLFunctions_3_3_Compatibility
{
    Q_OBJECT

public:
    Forward_Inversekinematics(QWidget *parent = nullptr);
    ~Forward_Inversekinematics();
        string filename;

    bool   on_animation = true;

    float  animation_time = 0.0f;

    int frame_no = 0;

    BVH* bvh = NULL;

    double x, y, z;
    string name = "Hips";

    void loadFile(string file_path);
    void updateFK();

protected:

    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);


private:
    Ui::Forward_InversekinematicsClass ui;
    
private slots:
    void loadFile(QString file_path);
    void writeFile();
    void rootTranslateX(int value);
    void rootTranslateY(int value);
    void rootTranslateZ(int value);
    void getJoint(QString name);
    void changePositionX(double value);
    void changePositionX_Damp(double value);
    void changePositionY(double value);
    void changePositionY_Damp(double value);
    void changePositionZ(double value);
    void changePositionZ_Damp(double value);
    void IK2();
    void addControlIK();
    void Stop_Start();

};
