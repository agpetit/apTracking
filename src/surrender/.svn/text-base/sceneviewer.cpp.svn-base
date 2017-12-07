#include "sceneviewer.h"
#include "scenemanager.h"

namespace luxifer
{
    SceneViewer::SceneViewer(QWidget *parent)
        : QGLWidget(parent)
    {
        mgr = NULL;
    }

    SceneViewer::SceneViewer(QGLContext *context, QWidget *parent)
        : QGLWidget(context, parent)
    {
        mgr = NULL;
    }

    SceneViewer::~SceneViewer()
    {

    }

    void SceneViewer::initializeGL()
    {
        glViewport(0, 0, width(), height());
        glDisable(GL_BLEND);
        glEnable(GL_COLOR_MATERIAL);
        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);
        glDisable(GL_TEXTURE_2D);
        glColor4ub(0xFF,0xFF,0xFF,0xFF);
    }

    void SceneViewer::paintGL()
    {
        if (!mgr)
        {
            glClear(GL_COLOR_BUFFER_BIT);
            return;
        }

        GLuint tex = mgr->getColorBuffer();

        glViewport(0, 0, width(), height());

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, 1, 1, 0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        glEnable(GL_COLOR_MATERIAL);
        glColor4ub(0xFF,0xFF,0xFF,0xFF);
        glBindTexture(GL_TEXTURE_2D, tex);

        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 1.0f);    glVertex2f(0.0f, 0.0f);
        glTexCoord2f(1.0f, 1.0f);    glVertex2f(1.0f, 0.0f);
        glTexCoord2f(1.0f, 0.0f);    glVertex2f(1.0f, 1.0f);
        glTexCoord2f(0.0f, 0.0f);    glVertex2f(0.0f, 1.0f);
        glEnd();
    }

    void SceneViewer::resizeGL(int w, int h)
    {
        glViewport(0, 0, w, h);
    }

    void SceneViewer::setSceneManager(SceneManager *mgr)
    {
        if (this->mgr == mgr)
            return;
        if (this->mgr)
            disconnect(mgr, SIGNAL(updated()), this, SLOT(repaint()));
        this->mgr = mgr;
        if (mgr)
            connect(mgr, SIGNAL(updated()), this, SLOT(repaint()));
    }

}
