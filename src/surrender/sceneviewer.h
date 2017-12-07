#ifndef SCENEVIEWER_H
#define SCENEVIEWER_H

#include <QGLWidget>
#include <GL/glu.h>
#include <GL/glut.h>

namespace luxifer
{
    class SceneManager;

    /** \brief a QGLWidget which displays images rendered from a SceneManager
      *
      * The SceneManager must use the SceneViewer OpenGL context or a context which
      * shares its data with it in order to share image data.
      */
    class SceneViewer : public QGLWidget
    {
        Q_OBJECT;
    public:
        SceneViewer(QWidget *parent = NULL);
        SceneViewer(QGLContext *context, QWidget *parent = NULL);
        ~SceneViewer();

    public slots:
        void setSceneManager(SceneManager *mgr);

    protected:
        virtual void initializeGL();
        virtual void paintGL();
        virtual void resizeGL(int w, int h);

    private:
        SceneManager *mgr;
    };

}
#endif // SCENEVIEWER_H
