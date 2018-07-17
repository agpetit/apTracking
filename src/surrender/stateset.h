#ifndef STATESET_H
#define STATESET_H

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif 
#include <osg/Uniform>
#include <osg/StateSet>
#include <osg/Material>
#include <map>
#include <string>
#include <utility>

namespace luxifer
{
    /** \brief Object representing an OpenGL state (the minimum needed to render our scene)
      *
      * It stores current material properties:
      * - colors
      * - textures
      * - GLSL programs
      * - uniforms
      */
    class StateSet
    {
        struct Material
        {
            osg::Vec4 diffuse;
            osg::Vec4 specular;
            osg::Vec4 emission;
            float shininess;
        };

    public:
        typedef unsigned int Mode;
    private:
        typedef std::map<std::string, std::pair<Mode, osg::ref_ptr<osg::Uniform> > > uniform_table_t;
    public:
        StateSet();
        StateSet(const StateSet &s);

        StateSet &operator=(const StateSet &s);

        void apply();

        void setTexture(size_t idx, GLuint target, GLuint htex, Mode mode);
        void setUniforms(const osg::StateSet::UniformList &uniforms);
        void setProgram(GLuint program, Mode mode);
        void setMaterial(osg::ref_ptr<const osg::Material> mtl, Mode mode);

    private:
        std::pair<Mode, std::pair<GLuint, GLuint> > hTex[16];
        uniform_table_t uniforms;
        std::pair<Mode, GLuint> hProgram;
        std::pair<Mode, Material> material;
    };
}

#endif // STATESET_H
