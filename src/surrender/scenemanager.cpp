#include <GL/glew.h>
#include "scenemanager.h"
#include <fstream>
#include <osg/Texture2D>
#include <osg/Geometry>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Transform>
#include <osg/PositionAttitudeTransform>
#include <osg/ClearNode>
#include <osg/CullFace>
#include <osg/PolygonOffset>
#include <osg/Light>
#include <osg/LightSource>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include "stateset.h"
#include "logs.h"
#include "utils.h"
#include "modelloader.h"
#include <visp/vpHomogeneousMatrix.h>
#include "../apRend.h"

// Some compilers may not define these macros ... part of C89 standard :/
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

using namespace std;
using namespace osg;

namespace luxifer
{

    SceneManager::SceneManager(QGLContext *pQContext)
    {
        const bool bOwnContext = !pQContext;
        if (!pQContext)
            pQContext = new QGLContext(QGLFormat::defaultFormat());

        pContext = new GLContextManager(pQContext);
        pContext->setOwnContext(bOwnContext);

        camera_matrix = Matrixd::identity();

        pFBO_postproc = NULL;

        pFBO_tmp = NULL;

        image_w = 640;
        image_h = 480;

        fov = 30.0;
        aspect_ratio = 4.0 / 3.0;
    }

    SceneManager::~SceneManager()
    {
        clear();
    }

    void SceneManager::clear()
    {
        scene_node = NULL;
        color_buffer = NULL;
        normal_buffer = NULL;
        edge_buffer = NULL;

        compositor_program = NULL;

        pContext->clear();

        if (pFBO_postproc)
            delete pFBO_postproc;
        pFBO_postproc = NULL;

        if (pFBO_tmp)
            delete pFBO_tmp;
        pFBO_tmp = NULL;
    }

    ref_ptr<Node> SceneManager::getSceneNode()
    {
        return scene_node;
    }

    void SceneManager::setCamera(const Matrixd &mat)
    {
        camera_matrix = mat;
    }

    void SceneManager::setCameraPosition(const osg::Vec3d &pos)
    {
        camera_matrix = Matrixd::rotate(camera_matrix.getRotate());
        camera_matrix.postMult(Matrixd::translate(-pos));
    }

    void SceneManager::setCameraAttitude(const osg::Quat &attitude)
    {
        camera_matrix = Matrixd::translate(camera_matrix.getTrans());
        camera_matrix.preMult(Matrixd::rotate(attitude.conj()));
    }

    Matrixd SceneManager::getCameraMatrix()
    {
        return camera_matrix;
    }

    Light *createLight(Vec4 color)
    {
    Light *light = new Light();
    // each light must have a unique number
    light->setLightNum(0);
    // we set the light's position via a PositionAttitudeTransform object
    light->setPosition(Vec4(0.0, 1.0, 1.0, 1));
    light->setDiffuse(color);
    light->setSpecular(Vec4(1.0, 0.0, 1.0, 1.0));
    light->setAmbient( Vec4(0.0, 0.0, 1.0, 1.0));
    return light;
    }

    Material *createSimpleMaterial(Vec4 color)
    {
    Material *material = new Material();
    material->setDiffuse(Material::FRONT,  Vec4(1.0, 1.0, 1.0, 1.0));
    material->setEmission(Material::FRONT, color);
    return material;
    }


    void SceneManager::load(const std::string &filename)
    {
        pContext->makeCurrent();
        clear();
        scene_node = ModelLoader::load(filename);

        const int _w = image_w;
        const int _h = image_h;



        // The buffer which contain local 3D fragment normal
        normal_buffer = new Texture2D;
        normal_buffer->setInternalFormat(GL_RGBA16);
        normal_buffer->setFilter(Texture2D::MIN_FILTER, Texture2D::NEAREST);
        normal_buffer->setFilter(Texture2D::MAG_FILTER, Texture2D::NEAREST);
        normal_buffer->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        normal_buffer->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        normal_buffer->setTextureSize(_w, _h);

        normal_buffer_big = new Texture2D;
        normal_buffer_big->setInternalFormat(GL_RGBA32F);
        normal_buffer_big->setFilter(Texture2D::MIN_FILTER, Texture2D::NEAREST);
        normal_buffer_big->setFilter(Texture2D::MAG_FILTER, Texture2D::NEAREST);
        normal_buffer_big->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        normal_buffer_big->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        normal_buffer_big->setTextureSize(_w * 2, _h * 2);

        // The buffer which contain local 3D fragment normal
        edge_buffer = new Texture2D;
        edge_buffer->setInternalFormat(GL_RGBA8);
        edge_buffer->setFilter(Texture2D::MIN_FILTER, Texture2D::NEAREST);
        edge_buffer->setFilter(Texture2D::MAG_FILTER, Texture2D::NEAREST);
        edge_buffer->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        edge_buffer->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        edge_buffer->setTextureSize(_w, _h);

        // The buffer we're rendering to
        color_buffer = new Texture2D;
        color_buffer->setInternalFormat(GL_RGBA8);
        color_buffer->setFilter(Texture2D::MIN_FILTER, Texture2D::LINEAR);
        color_buffer->setFilter(Texture2D::MAG_FILTER, Texture2D::LINEAR);
        color_buffer->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        color_buffer->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        color_buffer->setTextureSize(_w, _h);

        pFBO_postproc = new FBO(GL_TEXTURE_2D, pContext->getTextureID(color_buffer), _w, _h);

        pFBO_tmp = new FBO(_w, _h, GL_RGB8);

        // Make a camera for the viewer
        color_cam = new Camera(pContext, color_buffer);
        color_cam->setFOV(fov);
        color_cam->setAspectRatio(double(_w) / double(_h));
        ref_ptr<Group> color_cam_grp = new Group;
        color_cam_grp->addChild(scene_node);
        color_cam->setScene(color_cam_grp);

        /*osg::PositionAttitudeTransform *lightTransform;
        osg::Geode *lightMarker;

        lightMarker = new Geode();
        //lightMarker->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
        lightMarker->getOrCreateStateSet()->setAttribute(createSimpleMaterial(osg::Vec4(1.0, 0.0, 0.0, 1.0)));


        osg::StateSet *lightStateSet;
        lightStateSet = color_cam_grp->getOrCreateStateSet();
        lightsource = new LightSource();
        lightsource->setLight(createLight(Vec4(1.0, 0.0, 0.0, 1.0)));
        lightsource->setLocalStateSetModes(StateAttribute::ON);
        lightsource->setStateSetModes(*lightStateSet, StateAttribute::ON);

        lightTransform = new PositionAttitudeTransform();
        lightTransform->addChild(lightsource);
        lightTransform->addChild(lightMarker);
        lightTransform->setPosition(Vec3(0, 0, -5));
        //lightTransform->setScale(Vec3(0.1,0.1,0.1));

        color_cam_grp->addChild(lightTransform);
        color_cam->setScene(color_cam_grp);*/

        ref_ptr<Program> color_program = new Program;
        ref_ptr<Shader> color_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> color_fshader = new Shader(Shader::FRAGMENT);
        color_program->setName("Color program");
        color_program->addShader(color_vshader);
        color_program->addShader(color_fshader);
        color_vshader->loadShaderSourceFromFile("Materials/colors.vert");
        color_fshader->loadShaderSourceFromFile("Materials/colors.frag");
        color_cam_grp->getOrCreateStateSet()->setAttributeAndModes(color_program, StateAttribute::ON | StateAttribute::OVERRIDE);

        // Make a camera to compute local normals for local lighting effects (spots, raytracing, ...)
        normal_cam = new Camera(pContext, normal_buffer_big);
        normal_cam->setFOV(fov);
        normal_cam->setAspectRatio(double(_w) / double(_h));
        ref_ptr<Group> normal_cam_grp = new Group;
        normal_cam_grp->addChild(scene_node);
        normal_cam->setScene(normal_cam_grp);

        //normal_cam->setBackground(normal_buffer);

        ref_ptr<Program> normal_program = new Program;
        ref_ptr<Shader> normal_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> normal_fshader = new Shader(Shader::FRAGMENT);
        normal_program->setName("Normal program");
        normal_program->addShader(normal_vshader);
        normal_program->addShader(normal_fshader);
        normal_vshader->loadShaderSourceFromFile("Materials/normals.vert");
        normal_fshader->loadShaderSourceFromFile("Materials/normals.frag");
        normal_cam_grp->getOrCreateStateSet()->setAttributeAndModes(normal_program, StateAttribute::ON | StateAttribute::OVERRIDE);
        u_fNear = new osg::Uniform("fNear", 0.0f);
        u_fFar = new osg::Uniform("fFar", 0.0f);
        normal_cam_grp->getOrCreateStateSet()->addUniform(u_fNear);
        normal_cam_grp->getOrCreateStateSet()->addUniform(u_fFar);

        /*compositor_program1 = new Program;
        ref_ptr<Shader> compositor_fshader1 = new Shader(Shader::FRAGMENT);
        //compositor_program1->addShader(compositor_vshader);
        compositor_program1->addShader(compositor_fshader1);
        compositor_fshader1->loadShaderSourceFromFile("Materials/EdgeGradmap.frag");*/

        compositor_program = new Program;
        ref_ptr<Shader> compositor_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> compositor_fshader = new Shader(Shader::FRAGMENT);
        //ref_ptr<Shader> compositor_fshader1 = new Shader(Shader::FRAGMENT);
        //ref_ptr<Shader> compositor_vshader1 = new Shader(Shader::VERTEX);
        //compositor_program->addShader(compositor_vshader1);
        //compositor_program->addShader(compositor_fshader1);
        compositor_program->setName("Compositor program");
        compositor_program->addShader(compositor_vshader);
        compositor_program->addShader(compositor_fshader);
        //compositor_vshader1->loadShaderSourceFromFile("Materials/GaussianFilter.vert");
        //compositor_fshader1->loadShaderSourceFromFile("Materials/GaussianFilter.frag");
        compositor_vshader->loadShaderSourceFromFile("Materials/EdgeGradmap.vert");
        compositor_fshader->loadShaderSourceFromFile("Materials/EdgeGradmap.frag");

        fsaa_program = new Program;
        ref_ptr<Shader> fsaa_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> fsaa_fshader = new Shader(Shader::FRAGMENT);
        fsaa_program->setName("FSAA program");
        fsaa_program->addShader(fsaa_vshader);
        fsaa_program->addShader(fsaa_fshader);
        fsaa_vshader->loadShaderSourceFromFile("Materials/FSAA.vert");
        fsaa_fshader->loadShaderSourceFromFile("Materials/FSAA.frag");
    }

    void SceneManager::load(std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles)
    {
        pContext->makeCurrent();
        clear();
        scene_node = ModelLoader::load(vertices, normals, triangles);

        const int _w = image_w;
        const int _h = image_h;



        // The buffer which contain local 3D fragment normal
        normal_buffer = new Texture2D;
        normal_buffer->setInternalFormat(GL_RGBA16);
        normal_buffer->setFilter(Texture2D::MIN_FILTER, Texture2D::NEAREST);
        normal_buffer->setFilter(Texture2D::MAG_FILTER, Texture2D::NEAREST);
        normal_buffer->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        normal_buffer->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        normal_buffer->setTextureSize(_w, _h);

        normal_buffer_big = new Texture2D;
        normal_buffer_big->setInternalFormat(GL_RGBA32F);
        normal_buffer_big->setFilter(Texture2D::MIN_FILTER, Texture2D::NEAREST);
        normal_buffer_big->setFilter(Texture2D::MAG_FILTER, Texture2D::NEAREST);
        normal_buffer_big->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        normal_buffer_big->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        normal_buffer_big->setTextureSize(_w * 2, _h * 2);

        // The buffer which contain local 3D fragment normal
        edge_buffer = new Texture2D;
        edge_buffer->setInternalFormat(GL_RGBA8);
        edge_buffer->setFilter(Texture2D::MIN_FILTER, Texture2D::NEAREST);
        edge_buffer->setFilter(Texture2D::MAG_FILTER, Texture2D::NEAREST);
        edge_buffer->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        edge_buffer->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        edge_buffer->setTextureSize(_w, _h);

        // The buffer we're rendering to
        color_buffer = new Texture2D;
        color_buffer->setInternalFormat(GL_RGBA8);
        color_buffer->setFilter(Texture2D::MIN_FILTER, Texture2D::LINEAR);
        color_buffer->setFilter(Texture2D::MAG_FILTER, Texture2D::LINEAR);
        color_buffer->setWrap(Texture2D::WRAP_S, Texture2D::CLAMP_TO_EDGE);
        color_buffer->setWrap(Texture2D::WRAP_T, Texture2D::CLAMP_TO_EDGE);
        color_buffer->setTextureSize(_w, _h);

        pFBO_postproc = new FBO(GL_TEXTURE_2D, pContext->getTextureID(color_buffer), _w, _h);

        pFBO_tmp = new FBO(_w, _h, GL_RGB8);

        // Make a camera for the viewer
        color_cam = new Camera(pContext, color_buffer);
        color_cam->setFOV(fov);
        color_cam->setAspectRatio(double(_w) / double(_h));
        ref_ptr<Group> color_cam_grp = new Group;
        color_cam_grp->addChild(scene_node);
        color_cam->setScene(color_cam_grp);

        /*osg::PositionAttitudeTransform *lightTransform;
        osg::Geode *lightMarker;

        lightMarker = new Geode();
        //lightMarker->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
        lightMarker->getOrCreateStateSet()->setAttribute(createSimpleMaterial(osg::Vec4(1.0, 0.0, 0.0, 1.0)));


        osg::StateSet *lightStateSet;
        lightStateSet = color_cam_grp->getOrCreateStateSet();
        lightsource = new LightSource();
        lightsource->setLight(createLight(Vec4(1.0, 0.0, 0.0, 1.0)));
        lightsource->setLocalStateSetModes(StateAttribute::ON);
        lightsource->setStateSetModes(*lightStateSet, StateAttribute::ON);

        lightTransform = new PositionAttitudeTransform();
        lightTransform->addChild(lightsource);
        lightTransform->addChild(lightMarker);
        lightTransform->setPosition(Vec3(0, 0, -5));
        //lightTransform->setScale(Vec3(0.1,0.1,0.1));

        color_cam_grp->addChild(lightTransform);
        color_cam->setScene(color_cam_grp);*/

        ref_ptr<Program> color_program = new Program;
        ref_ptr<Shader> color_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> color_fshader = new Shader(Shader::FRAGMENT);
        color_program->addShader(color_vshader);
        color_program->addShader(color_fshader);
        color_vshader->loadShaderSourceFromFile("Materials/colors.vert");
        color_fshader->loadShaderSourceFromFile("Materials/colors.frag");
        color_cam_grp->getOrCreateStateSet()->setAttributeAndModes(color_program, StateAttribute::ON | StateAttribute::OVERRIDE);

        // Make a camera to compute local normals for local lighting effects (spots, raytracing, ...)
        normal_cam = new Camera(pContext, normal_buffer_big);
        normal_cam->setFOV(fov);
        normal_cam->setAspectRatio(double(_w) / double(_h));
        ref_ptr<Group> normal_cam_grp = new Group;
        normal_cam_grp->addChild(scene_node);
        normal_cam->setScene(normal_cam_grp);

        //normal_cam->setBackground(normal_buffer);

        ref_ptr<Program> normal_program = new Program;
        ref_ptr<Shader> normal_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> normal_fshader = new Shader(Shader::FRAGMENT);
        normal_program->addShader(normal_vshader);
        normal_program->addShader(normal_fshader);
        normal_vshader->loadShaderSourceFromFile("Materials/normals.vert");
        normal_fshader->loadShaderSourceFromFile("Materials/normals.frag");
        normal_cam_grp->getOrCreateStateSet()->setAttributeAndModes(normal_program, StateAttribute::ON | StateAttribute::OVERRIDE);
        u_fNear = new osg::Uniform("fNear", 0.0f);
        u_fFar = new osg::Uniform("fFar", 0.0f);
        normal_cam_grp->getOrCreateStateSet()->addUniform(u_fNear);
        normal_cam_grp->getOrCreateStateSet()->addUniform(u_fFar);

        /*compositor_program1 = new Program;
        ref_ptr<Shader> compositor_fshader1 = new Shader(Shader::FRAGMENT);
        //compositor_program1->addShader(compositor_vshader);
        compositor_program1->addShader(compositor_fshader1);
        compositor_fshader1->loadShaderSourceFromFile("Materials/EdgeGradmap.frag");*/

        compositor_program = new Program;
        ref_ptr<Shader> compositor_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> compositor_fshader = new Shader(Shader::FRAGMENT);
        //ref_ptr<Shader> compositor_fshader1 = new Shader(Shader::FRAGMENT);
        //ref_ptr<Shader> compositor_vshader1 = new Shader(Shader::VERTEX);
        //compositor_program->addShader(compositor_vshader1);
        //compositor_program->addShader(compositor_fshader1);
        compositor_program->addShader(compositor_vshader);
        compositor_program->addShader(compositor_fshader);
        //compositor_vshader1->loadShaderSourceFromFile("Materials/GaussianFilter.vert");
        //compositor_fshader1->loadShaderSourceFromFile("Materials/GaussianFilter.frag");
        compositor_vshader->loadShaderSourceFromFile("Materials/EdgeGradmap.vert");
        compositor_fshader->loadShaderSourceFromFile("Materials/EdgeGradmap.frag");

        fsaa_program = new Program;
        ref_ptr<Shader> fsaa_vshader = new Shader(Shader::VERTEX);
        ref_ptr<Shader> fsaa_fshader = new Shader(Shader::FRAGMENT);
        fsaa_program->addShader(fsaa_vshader);
        fsaa_program->addShader(fsaa_fshader);
        fsaa_vshader->loadShaderSourceFromFile("Materials/FSAA.vert");
        fsaa_fshader->loadShaderSourceFromFile("Materials/FSAA.frag");
    }

    void SceneManager::update(std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles)
    {
        ref_ptr<Group> color_cam_grp = new Group;
        ref_ptr<Group> normal_cam_grp = new Group;

        scene_node = ModelLoader::load(vertices, normals, triangles);

        color_cam_grp->addChild(scene_node);
        color_cam->setScene(color_cam_grp);

        normal_cam_grp->addChild(scene_node);
        normal_cam->setScene(normal_cam_grp);

    }

    osg::Vec3d SceneManager::getCameraPosition()
    {
        return -camera_matrix.getTrans();
    }

    osg::Quat SceneManager::getCameraAttitude()
    {
        return camera_matrix.getRotate().conj();
    }

    void SceneManager::render()
    {
        scene_node->computeBound();

        // Render normal buffer
        normal_cam->setFOV(fov);
        normal_cam->setAspectRatio(aspect_ratio);
        normal_cam->setDoublePrecisionMode(false);
        normal_cam->setViewMatrix(camera_matrix);
        normal_cam->setMinimalDepth(1e-1);
        (*normal_cam)();

        // Render main view
        color_cam->setFOV(fov);
        color_cam->setAspectRatio(aspect_ratio);
        color_cam->setDoublePrecisionMode(false);
        color_cam->setViewMatrix(camera_matrix);
        color_cam->setMinimalDepth(1e-1);
        (*color_cam)();

        // Post processes
        postprocess(normal_buffer_big, normal_buffer, fsaa_program);
        postprocess(normal_buffer, edge_buffer, compositor_program);

        emit updated();
    }

    void SceneManager::renderAR()
    {
        scene_node->computeBound();

        // Render main view
        color_cam->setFOV(fov);
        color_cam->setAspectRatio(aspect_ratio);
        color_cam->setDoublePrecisionMode(false);
        color_cam->setViewMatrix(camera_matrix);
        color_cam->setMinimalDepth(1e-1);
        (*color_cam)();

        // Post processes
        /*postprocess(normal_buffer_big, normal_buffer, fsaa_program);
        postprocess(normal_buffer, edge_buffer, compositor_program);*/

        emit updated();
    }

    GLuint SceneManager::getColorBuffer()
    {
        return pContext->getTextureID(color_buffer);
    }

    GLuint SceneManager::getNormalBuffer()
    {
        return pContext->getTextureID(normal_buffer);
    }

    void SceneManager::copyTextureToMemory(osg::ref_ptr<osg::Texture2D> ptex, void *data, GLuint format, GLuint type)
    {
        pContext->makeCurrent();
        glBindTexture(GL_TEXTURE_2D, pContext->getTextureID(ptex));
        glGetTexImage(GL_TEXTURE_2D, 0, format, type, data);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void SceneManager::setFOV(const double fov)
    {
        this->fov = fov;
    }

    void SceneManager::setAspectRatio(const double aspect_ratio)
    {
        this->aspect_ratio = aspect_ratio;
    }

    void SceneManager::setImageSize(const size_t image_w, const size_t image_h)
    {
        this->image_w = image_w;
        this->image_h = image_h;
    }

    void SceneManager::postprocess(osg::ref_ptr<osg::Texture2D> in, osg::ref_ptr<osg::Texture2D> out, osg::ref_ptr<osg::Program> program)
    {
        pFBO_postproc->setColorTarget(GL_TEXTURE_2D, pContext->getTextureID(out), out->getTextureWidth(), out->getTextureHeight());
        pFBO_postproc->bind();
        glDepthMask(GL_FALSE);
        glDisable(GL_DEPTH_TEST);

        const GLuint pid = pContext->getProgramID(program);
        glUseProgram(pid);

        glDisable(GL_CULL_FACE);

        glActiveTexture(GL_TEXTURE0);   glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, pContext->getTextureID(in));

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0,1,1,0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glUniform1i(glGetUniformLocation(pid, "tex"), 0);
        glUniform2f(glGetUniformLocation(pid, "dp"), 1.0 / image_w, 1.0 / image_h);
        if (glGetUniformLocation(pid, "edgeTh") >= 0)
            glUniform1f(glGetUniformLocation(pid, "edgeTh"), float(mrend->edgeR_th));
        glUniform1f(glGetUniformLocation(pid, "fNear"), fNear);
        glUniform1f(glGetUniformLocation(pid, "fFar"), fFar);

        glBegin(GL_QUADS);
        glVertex2f(0.f, 0.f);
        glVertex2f(1.f, 0.f);
        glVertex2f(1.f, 1.f);
        glVertex2f(0.f, 1.f);
        glEnd();

        glUseProgram(0);

        glActiveTexture(GL_TEXTURE1);   glDisable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE0);   glDisable(GL_TEXTURE_2D);
        pFBO_postproc->release();
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }

    void SceneManager::copyTextureToTexture(osg::ref_ptr<osg::Texture2D> dst, osg::ref_ptr<osg::Texture2D> src)
    {
        pFBO_tmp->setColorTarget(GL_TEXTURE_2D, pContext->getTextureID(src), dst->getTextureWidth(), src->getTextureHeight());
        pFBO_tmp->bind();
        glBindTexture(GL_TEXTURE_2D, pContext->getTextureID(dst));
        glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, dst->getTextureWidth(), dst->getTextureHeight());

        pFBO_tmp->release();
    }

    /**
    * Update Camera parameters from a pose calculation
    */
    void SceneManager::updateCameraParameters (const vpHomogeneousMatrix &cMo)
    {
        // As y from ViSP is -y from Ogre and z from ViSP is -z from Ogre
        // we have to change the matrix a little bit.
        // Rotations between x and y will therefore see their sign inverted
        // and so will rotations between x and z.
        // Rotations between y and z don't have to move as their relation is
        // the same in ViSP and Ogre.
        // As for translations, ty=-ty and tz=-tz as y=-y and z=-z

        vpTranslationVector translat;
        cMo.extract(translat);
        updateClipDistances(translat[2]);
        osg::Matrixd ModelView = osg::Matrixd(   cMo[0][0], -cMo[1][0], -cMo[2][0], 0,
                                                 cMo[0][1], -cMo[1][1], -cMo[2][1], 0,
                                                 cMo[0][2], -cMo[1][2], -cMo[2][2], 0,
                                                 cMo[0][3], -cMo[1][3], -cMo[2][3], 1);
        setCamera(ModelView);
    }

    void SceneManager::updateCameraParameters (const vpHomogeneousMatrix &cMo, const double Znear, const double Zfar)
    {
        // As y from ViSP is -y from Ogre and z from ViSP is -z from Ogre
        // we have to change the matrix a little bit.
        // Rotations between x and y will therefore see their sign inverted
        // and so will rotations between x and z.
        // Rotations between y and z don't have to move as their relation is
        // the same in ViSP and Ogre.
        // As for translations, ty=-ty and tz=-tz as y=-y and z=-z

        vpTranslationVector translat;
        cMo.extract(translat);
        updateClipDistances(Znear,Zfar);
        osg::Matrixd ModelView = osg::Matrixd(   cMo[0][0], -cMo[1][0], -cMo[2][0], 0,
                                                 cMo[0][1], -cMo[1][1], -cMo[2][1], 0,
                                                 cMo[0][2], -cMo[1][2], -cMo[2][2], 0,
                                                 cMo[0][3], -cMo[1][3], -cMo[2][3], 1);
        setCamera(ModelView);
    }

    void SceneManager::updateClipDistances(const double Zd)
    {
        // Camera objects use optimal dynamic clipping planes
        // ==> we must emulate clipping in the shader
        const double clipD = mrend->clipDist;
        u_fNear->set(float(Zd - clipD));
        u_fFar->set(float(Zd + clipD));
        fNear = float(Zd - clipD);
        fFar = float(Zd + clipD);
    }

    void SceneManager::updateClipDistances(const double Znear, const double Zfar)
    {
        // Camera objects use optimal dynamic clipping planes
        // ==> we must emulate clipping in the shader
        u_fNear->set(float(Znear));
        u_fFar->set(float(Zfar));
        fNear = float(Znear);
        fFar = float(Zfar);
    }

    /**
    * Update render to texture for depth edges
    * \param Inormd : RGBA map containing the normal map + depth map of the rendered scene.
    * \param Ior : depth edges map, with edge orientations
    * \param CMo : camera parameters
    */
    void SceneManager::updateRTT(vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo)
    {
        updateCameraParameters(*CMo);

        render();

        copyTextureToMemory(normal_buffer, I1.bitmap, GL_RGBA, GL_UNSIGNED_BYTE);
        copyTextureToMemory(edge_buffer, I0.bitmap, GL_RED, GL_UNSIGNED_BYTE);

        // OpenGL textures are upside down so we have to flip them
        for(int y = 0 ; y < I0.getHeight() / 2 ; ++y)
            std::swap_ranges(I0[y], I0[y] + I0.getWidth(), I0[I0.getHeight() - 1 - y]);
        for(int y = 0 ; y < I1.getHeight() / 2 ; ++y)
            std::swap_ranges(I1[y], I1[y] + I1.getWidth(), I1[I1.getHeight() - 1 - y]);
    }

    void SceneManager::updateRTT(vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo, const double znear, const double zfar)
    {
        updateCameraParameters(*CMo, znear, zfar);

        render();

        copyTextureToMemory(normal_buffer, I1.bitmap, GL_RGBA, GL_UNSIGNED_BYTE);
        copyTextureToMemory(edge_buffer, I0.bitmap, GL_RED, GL_UNSIGNED_BYTE);

        // OpenGL textures are upside down so we have to flip them
        for(int y = 0 ; y < I0.getHeight() / 2 ; ++y)
            std::swap_ranges(I0[y], I0[y] + I0.getWidth(), I0[I0.getHeight() - 1 - y]);
        for(int y = 0 ; y < I1.getHeight() / 2 ; ++y)
            std::swap_ranges(I1[y], I1[y] + I1.getWidth(), I1[I1.getHeight() - 1 - y]);
    }

    void SceneManager::updateRTTCol(vpImage<vpRGBa> &I2, vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo)
    {
        updateCameraParameters(*CMo);

        render();

        copyTextureToMemory(normal_buffer, I1.bitmap, GL_RGBA, GL_UNSIGNED_BYTE);
        copyTextureToMemory(edge_buffer, I0.bitmap, GL_RED, GL_UNSIGNED_BYTE);
        copyTextureToMemory(color_buffer, I2.bitmap, GL_RGBA, GL_UNSIGNED_BYTE);

        // OpenGL textures are upside down so we have to flip them
        for(int y = 0 ; y < I0.getHeight() / 2 ; ++y)
            std::swap_ranges(I0[y], I0[y] + I0.getWidth(), I0[I0.getHeight() - 1 - y]);
        for(int y = 0 ; y < I1.getHeight() / 2 ; ++y)
            std::swap_ranges(I1[y], I1[y] + I1.getWidth(), I1[I1.getHeight() - 1 - y]);
        for(int y = 0 ; y < I2.getHeight() / 2 ; ++y)
            std::swap_ranges(I2[y], I2[y] + I2.getWidth(), I2[I2.getHeight() - 1 - y]);
    }
    void SceneManager::updateRTTCol(vpImage<vpRGBa> &I2, vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo, const double znear, const double zfar)
    {
        updateCameraParameters(*CMo, znear, zfar);

        render();

        copyTextureToMemory(normal_buffer, I1.bitmap, GL_RGBA, GL_UNSIGNED_BYTE);
        copyTextureToMemory(edge_buffer, I0.bitmap, GL_RED, GL_UNSIGNED_BYTE);
        copyTextureToMemory(color_buffer, I2.bitmap, GL_RGBA, GL_UNSIGNED_BYTE);

        // OpenGL textures are upside down so we have to flip them
        for(int y = 0 ; y < I0.getHeight() / 2 ; ++y)
            std::swap_ranges(I0[y], I0[y] + I0.getWidth(), I0[I0.getHeight() - 1 - y]);
        for(int y = 0 ; y < I1.getHeight() / 2 ; ++y)
            std::swap_ranges(I1[y], I1[y] + I1.getWidth(), I1[I1.getHeight() - 1 - y]);
        for(int y = 0 ; y < I2.getHeight() / 2 ; ++y)
            std::swap_ranges(I2[y], I2[y] + I2.getWidth(), I2[I2.getHeight() - 1 - y]);
    }

    void SceneManager::updateRTTColAR(vpImage<vpRGBa> &I2, vpHomogeneousMatrix *CMo)
    {
        updateCameraParameters(*CMo);

        renderAR();

        copyTextureToMemory(color_buffer, I2.bitmap, GL_RGBA, GL_UNSIGNED_BYTE);

        // OpenGL textures are upside down so we have to flip them

        for(int y = 0 ; y < I2.getHeight() / 2 ; ++y)
            std::swap_ranges(I2[y], I2[y] + I2.getWidth(), I2[I2.getHeight() - 1 - y]);
    }

    void SceneManager::setApRend(const apRend *mrend)
    {
        this->mrend = mrend;
    }
}
